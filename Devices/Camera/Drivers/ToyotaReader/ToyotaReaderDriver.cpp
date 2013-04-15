
#include "ToyotaReaderDriver.h"
#include <Mvlpp/Utils.h>  // for FindFiles and PrintError
#include <boost/format.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "RPG/Devices/Camera/Drivers/Dvi2Pci/SDK/includes/s_fio.h"	// for imread()

//using namespace boost;
using namespace std;

///////////////////////////////////////////////////////////////////////////////
ToyotaReaderDriver::ToyotaReaderDriver(){}

///////////////////////////////////////////////////////////////////////////////
ToyotaReaderDriver::~ToyotaReaderDriver() {
    m_CaptureThread->interrupt();
    m_CaptureThread->join();

    // Close files
    for( unsigned int ii = 0; ii < m_uNumChannels; ++ii ) {
        if( m_vChannels[ii]->is_open())
                m_vChannels[ii]->close();
    }

}

///////////////////////////////////////////////////////////////////////////////
// Consumer
bool ToyotaReaderDriver::Capture( std::vector<rpg::ImageWrapper>& vImages ) {

    if( m_qImageBuffer.size() == 0 && !m_bLoop) {
        return false;
    }

    boost::mutex::scoped_lock lock(m_Mutex);

    // Wait until the buffer has data to read
    while( m_qImageBuffer.size() == 0 ) {
        m_cBufferEmpty.wait(lock);
    }

    //***************************************************
    // consume from buffer
    //***************************************************
    // allocate images if necessary
    if( vImages.size() != m_uNumChannels )
        vImages.resize( m_uNumChannels );

    // now fetch the next set of images from buffer
    for( unsigned int ii = 0; ii < m_uNumChannels; ++ii )
        vImages[ii].Image = m_qImageBuffer.front()[ii].Image.clone();

    // remove image from queue
    m_qImageBuffer.pop();

    //***************************************************
    // send notification that the buffer has space
    m_cBufferFull.notify_one();

    return true;
}


///////////////////////////////////////////////////////////////////////////////
bool ToyotaReaderDriver::Init() {

    assert(m_pPropertyMap);


    m_uNumChannels          = m_pPropertyMap->GetProperty<unsigned int>( "NumChannels", 0 );
    m_uBufferSize           = m_pPropertyMap->GetProperty<unsigned int>( "BufferSize", 35 );
    m_uStartFrame           = m_pPropertyMap->GetProperty<unsigned int>( "StartFrame",  0 ); // Not used
    m_bLoop                 = m_pPropertyMap->GetProperty<bool>( "Loop",  false );
    m_uCurrentImageIndex    = m_uStartFrame;
    m_nCvImageReadFlags     = m_pPropertyMap->GetProperty<bool>( "ForceGrayscale",  false )
            ? cv::IMREAD_GRAYSCALE : cv::IMREAD_UNCHANGED;
    m_bOutputRectified = m_pPropertyMap->GetProperty("Rectify", true);

    if( m_uNumChannels < 1 ) {
        mvl::PrintError( "ERROR: No channels specified. Set property NumChannels.\n" );
        exit(1);
    }

    // Get data path
    std::string sChannelPath = m_pPropertyMap->GetProperty( "DataSourceDir", "");

    CameraInfo cam;
    string sFilename;
    string sField;
    ifstream fin;


    // Open channel data ( .cam files for left and right images)
    for( unsigned int ii = 0; ii < m_uNumChannels; ii++ ) {
        //std::cerr << "SlamThread: Finding files channel " << ii << std::endl;
        std::string sChannelPropertyName  = (boost::format("Channel-%d")%ii).str();
        std::string sChannelFileName = m_pPropertyMap->GetProperty( sChannelPropertyName, "");

        // Read channel info (.info.yml  files)
        sFilename = sChannelPath + "/" + sChannelFileName + ".info.yml";
        fin.open(sFilename.c_str());
        if( !fin.is_open() ) {
             mvl::PrintError( "ERROR opening: %s\n", sFilename.c_str());
            exit(1);
        }
        fin >> sField >> sField >> sField;
        fin >> sField >> cam.w >> sField >> cam.h >> sField >> cam.sformat >> sField >> cam.fps >> sField >> cam.name;
        fin.close();
        cam.format = _GetImageFormat(cam.sformat);

        // Compute frame size in bytes (for reading)
        if(cam.format == RGB || cam.format == BGR )
            cam.fsize = cam.w * cam.h * 3;
        else // (BAYER)
            cam.fsize = cam.w * cam.h;

        // get camera intrinsics
        if( m_bOutputRectified ) {
            std::string sCModPropertyName  = (boost::format("CamModel-%d")%ii).str();
            std::string sCModel = sChannelPath + "/" + m_pPropertyMap->GetProperty( sCModPropertyName, "" );
            std::cout << "Loading: " << sCModel << std::endl;

            /* */
            cam.pCMod = new mvl::CameraModel( sCModel );

            if( cam.pCMod->GetModel() == NULL ) {
                std::cerr << "Error reading camera model! Not rectifying.\n" << std::endl;
                m_bOutputRectified = false;
            }
            /* */

            /*
            cv::FileStorage fs( sCModel,  cv::FileStorage::READ );
            fs["map_row"] >> cam.RectMapRow;
            fs["map_col"] >> cam.RectMapCol;
            /* */
        }
        m_vCamerasInfo.push_back(cam);

        // open channel for reading
        sFilename = sChannelPath + "/" + sChannelFileName + ".cam";
        ifstream* channel = new ifstream(sFilename.c_str(), ios::in | ios::binary);
        if( !channel->is_open() ) {
             mvl::PrintError( "ERROR opening: %s\n", sFilename.c_str());
            exit(1);
        }
        const unsigned int nStartPtr = m_uStartFrame * cam.fsize;
        channel->seekg( nStartPtr, ios::beg);
        m_vChannels.push_back(channel);
    }

    if( m_uNumChannels == 2 && m_bOutputRectified ) {
        m_Rectify.Init( *(m_vCamerasInfo[0].pCMod), *(m_vCamerasInfo[1].pCMod) );
    }

    _PrintCamInfo();

    // fill buffer
    for (unsigned int ii=0; ii < m_uBufferSize; ii++) {	_Read(); }

        // Start capturing thread
    m_CaptureThread = new boost::thread( &_ThreadCaptureFunc, this );

    return true;
}

///////////////////////////////////////////////////////////////////////////////
// Producer
void ToyotaReaderDriver::_ThreadCaptureFunc( ToyotaReaderDriver* pFR ) {

    while(1) {
        try {
            boost::this_thread::interruption_point();

            if( !pFR->_Read() ) {
                break;
            }

        } catch( boost::thread_interrupted& interruption ) {
            break;
        }
    }

}

// TODO: refactor into MVL?
void ToyotaReaderDriver::_bayer8_to_grey8_half(unsigned char* src, unsigned char* dst, unsigned int srcWidth, unsigned int srcHeight )
{
    for( int ii = 0; ii < (int)srcHeight; ii+=2 ) {
        for( int jj = 0; jj < (int)srcWidth; jj+=2 ) {
            const unsigned int a = src[ (ii * srcWidth) + jj   ];
            const unsigned int b = src[ (ii * srcWidth) + jj + 1 ];
            const unsigned int c = src[((ii + 1) * srcWidth) + jj ];
            const unsigned int d = src[((ii + 1) * srcWidth) + jj + 1 ];
            dst[ ((ii/2)*(srcWidth/2))+(jj/2) ] = ( a + b + c + d) / 4;
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
//void ToyotaReaderDriver::_Read( std::vector<rpg::ImageWrapper>& vImages)
bool ToyotaReaderDriver::_Read() {

    boost::mutex::scoped_lock lock(m_Mutex);

    // Wait until there is space in the buffer
    while(! (m_qImageBuffer.size() < m_uBufferSize) ){
        m_cBufferFull.wait(lock);
    }

    //*************************************************************************
    // produce to buffer
    //*************************************************************************

    // now fetch the next set of images
    std::vector<rpg::ImageWrapper> vImages;
    vImages.resize(m_uNumChannels);

    for( unsigned int ii = 0; ii < m_uNumChannels; ++ii ) {
        //check if we are not at the end of the file
        if( m_vChannels[ii]->eof() ) {
            if( m_bLoop ){
                m_vChannels[ii]->clear();
                m_vChannels[ii]->seekg(0,ios::beg);
                m_uCurrentImageIndex = 0;
            } else {
                return false;
            }
        }
        // allocate memory for the frame
        int format =m_vCamerasInfo[ii].format ;
        int type = ( format == RGB || format == BGR)?CV_8UC3:CV_8UC1;

        // read from file
        if( m_vChannels[ii]->is_open() ) {
            if( format > 1 ) { // BAYER
                vImages[ii].Image.create(m_vCamerasInfo[ii].h/2,m_vCamerasInfo[ii].w/2, type);
                cv::Mat imgBayer(m_vCamerasInfo[ii].h,m_vCamerasInfo[ii].w, type);
                m_vChannels[ii]->read((char*)imgBayer.data,m_vCamerasInfo[ii].fsize);
                if( type == CV_8UC3 ) {
                    // Debayer and downsample into RGB image
//                    dc1394_bayer_decoding_8bit(imgBayer.data,vImages[ii].Image.data, m_vCamerasInfo[ii].w, m_vCamerasInfo[ii].h, DC1394_COLOR_FILTER_GRBG, DC1394_BAYER_METHOD_DOWNSAMPLE );

                } else {
                    // Debayer into Greyscale, half-sampled.
//                    cv::Mat rectImage;
//                    cv::remap( imgBayer, rectImage, m_vCamerasInfo[ii].RectMapCol, m_vCamerasInfo[ii].RectMapRow, CV_INTER_LINEAR );
//                    cv::remap( imgBayer, vImages[ii].Image, m_vCamerasInfo[ii].RectMapCol, m_vCamerasInfo[ii].RectMapRow, CV_INTER_LINEAR );
//                    _bayer8_to_grey8_half(rectImage.data, vImages[ii].Image.data, m_vCamerasInfo[ii].w, m_vCamerasInfo[ii].h);
                    _bayer8_to_grey8_half(imgBayer.data, vImages[ii].Image.data, m_vCamerasInfo[ii].w, m_vCamerasInfo[ii].h);
                }
            } else {
                vImages[ii].Image.create(m_vCamerasInfo[ii].h,m_vCamerasInfo[ii].w, type);
                m_vChannels[ii]->read((char*)vImages[ii].Image.data,m_vCamerasInfo[ii].fsize);
            }
        } else {
            mvl::PrintError( "ERROR file closed \n");
        }

    }

    if( m_uNumChannels == 2 && m_bOutputRectified ) {
        cv::Mat rectImage0( vImages[0].Image.rows, vImages[0].Image.cols, CV_8UC1 );
        cv::Mat rectImage1( vImages[1].Image.rows, vImages[1].Image.cols, CV_8UC1 );
//        cv::remap( vImages[0].Image, rectImage0, m_vCamerasInfo[0].RectMapCol, m_vCamerasInfo[0].RectMapRow, CV_INTER_LINEAR );
//        cv::remap( vImages[1].Image, rectImage1, m_vCamerasInfo[1].RectMapCol, m_vCamerasInfo[1].RectMapRow, CV_INTER_LINEAR );
        m_Rectify.Rectify( vImages[0].Image, vImages[1].Image, rectImage0, rectImage1 );
        vImages[0].Image = rectImage0;
        vImages[1].Image = rectImage1;
    }


    m_uCurrentImageIndex++;
    //cout << "frame : " << m_uCurrentImageIndex << endl;
    // add images at the back of the queue
    m_qImageBuffer.push(vImages);

    //*************************************************************************
    // send notification that the buffer is not empty
    m_cBufferEmpty.notify_one();

    return true;
}

///////////////////////////////////////////////////////////////////////////////
int ToyotaReaderDriver::_GetImageFormat(string& format) {

    if( !format.compare("RGB") ) return RGB;
    if( !format.compare("BGR") ) return BGR;
    if( !format.compare("BAYER") ) return BAYER;
    if( !format.compare("BAYER_BG") ) return BAYER_BG;
    if( !format.compare("BAYER_GB") ) return BAYER_GB;
    if( !format.compare("BAYER_RG") ) return BAYER_RG;
    if( !format.compare("BAYER_GR") ) return BAYER_GR;
    return RGB;
}

///////////////////////////////////////////////////////////////////////////////
void ToyotaReaderDriver::_PrintCamInfo( ) {
     for( unsigned int ii = 0; ii < m_uNumChannels; ++ii ) {
        std::cout << "========================" << std::endl;
        std::cout << " Channel [ " << ii << " ]" << std::endl;
        std::cout << "========================" << std::endl;
        std::cout <<  std::setw(8) << std::left << "w,h:";
        std:: cout << std::setw(4) << std::left << m_vCamerasInfo[ii].w << " , " << m_vCamerasInfo[ii].h << std::endl;
        std::cout <<  std::setw(8) << "fps:" <<  std::setw(12) << std::left << m_vCamerasInfo[ii].fps << std::endl;
        std::cout <<  std::setw(8) << "format:";
        switch( m_vCamerasInfo[ii].format ) {
            case RGB:  std::cout <<  std::setw(12) << std::left << "RGB" << std::endl; break;
            case BGR:  std::cout <<  std::setw(12) <<std::left << "BGR" << std::endl; break;
            case BAYER:  std::cout <<  std::setw(12) <<std::left << "BAYER" << std::endl; break;
            case BAYER_BG:  std::cout <<  std::setw(12) << std::left << "BAYER_BG" << std::endl; break;
            case BAYER_GB:  std::cout <<  std::setw(12) <<  std::left << "BAYER_GB" << std::endl; break;
            case BAYER_RG:  std::cout <<  std::setw(12) <<  std::left << "BAYER_RG" << std::endl; break;
            case BAYER_GR:  std::cout <<  std::setw(12) <<  std::left << "BAYER_GR" << std::endl; break;
        }
        std::cout <<  std::setw(8) << "name:" <<  std::setw(12) <<  std::left << m_vCamerasInfo[ii].name<< std::endl;
     }
}
