
#include "ToyotaReaderDriver.h"
#include <Mvlpp/Utils.h>  // for FindFiles and PrintError
#include <boost/format.hpp>
#include <opencv.hpp>

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
    for( unsigned int ii = 0; ii < m_uNumChannels; ++ii ) {
        vImages[ii].Image = m_qImageBuffer.front()[ii].Image.clone();
    }

    // remove image from queue
    m_qImageBuffer.pop();

    //***************************************************
    // send notification that the buffer has space
    m_cBufferFull.notify_one();

    // sanity check
    // TODO: fix _Read so that if timestamps differ, then "align" reads
    if( vImages[0].Map.GetProperty("Timestamp", 0) != vImages[1].Map.GetProperty("Timestamp", 0) ) {
        std::cerr << "ERROR: discrepancies in timestamps!!!" << std::endl;
        return false;
    }

    return true;
}


///////////////////////////////////////////////////////////////////////////////
bool ToyotaReaderDriver::Init() {

    assert(m_pPropertyMap);

    m_uNumChannels          = m_pPropertyMap->GetProperty<unsigned int>( "NumChannels", 2 );
    m_uBufferSize           = m_pPropertyMap->GetProperty<unsigned int>( "BufferSize", 35 );
    m_uStartFrame           = m_pPropertyMap->GetProperty<unsigned int>( "StartFrame",  0 );
    m_bLoop                 = m_pPropertyMap->GetProperty<bool>( "Loop",  false );
    m_uCurrentImageIndex    = m_uStartFrame;
    m_bOutputRectified      = m_pPropertyMap->GetProperty<bool>( "Rectify", true );
    m_bReadTimestamps       = true;

    if( m_uNumChannels < 1 ) {
        mvl::PrintError( "ERROR: No channels specified. Set property NumChannels.\n" );
        exit(1);
    }

    // Get data path
    std::string sChannelPath = m_pPropertyMap->GetProperty( "DataSourceDir", "" );

    // Open channel data ( .cam files for left and right images)
    for( unsigned int ii = 0; ii < m_uNumChannels; ii++ ) {

        string sFilename;
        std::string sChannelPropertyName  = (boost::format("Channel-%d")%ii).str();
        std::string sChannelFileName = m_pPropertyMap->GetProperty( sChannelPropertyName, "");

        // Read channel info (.info.yml  files)
        sFilename = sChannelPath + "/" + sChannelFileName + ".info.yml";
        cv::FileStorage fInfo( sFilename,  cv::FileStorage::READ );
        if( fInfo.isOpened() == false ) {
            mvl::PrintError( "ERROR opening: %s\n", sFilename.c_str() );
            return false;
        }

        CameraInfo* pCam = new CameraInfo;
        fInfo["width"] >> pCam->w;
        fInfo["height"] >> pCam->h;
        fInfo["fps"] >> pCam->fps;
        fInfo["format"] >> pCam->sformat;
        fInfo["name"] >> pCam->name;
        pCam->format = _GetImageFormat( pCam->sformat );

        fInfo.release();

        // Compute frame size in bytes (for reading)
        if(pCam->format == RGB || pCam->format == BGR ) {
            pCam->fsize = pCam->w * pCam->h * 3;
        }
        else { // (BAYER)
            pCam->fsize = pCam->w * pCam->h;
        }

        // get camera intrinsics
        if( m_bOutputRectified ) {
            std::string sCModel = sChannelPath + "/" + sChannelFileName + ".yml";
            std::cout << "Loading: " << sCModel << std::endl;

            cv::FileStorage fCMod( sCModel,  cv::FileStorage::READ );

            if( fCMod.isOpened() ) {
                fCMod["map_row"] >> pCam->RectMapRow;
                fCMod["map_col"] >> pCam->RectMapCol;
                fCMod.release();
            } else {
                std::cerr << "Error reading camera model! Not rectifying.\n" << std::endl;
                m_bOutputRectified = false;
            }
        }
        m_vCamerasInfo.push_back( pCam );

        // open channel for reading
        sFilename = sChannelPath + "/" + sChannelFileName + ".cam";
        ifstream* pChannel = new ifstream(sFilename.c_str(), ios::in | ios::binary);
        if( !pChannel->is_open() ) {
             mvl::PrintError( "ERROR opening: %s\n", sFilename.c_str());
             return false;
        }
        unsigned int nStartPtr = m_uStartFrame * pCam->fsize;
        pChannel->seekg( nStartPtr, ios::beg );
        m_vChannels.push_back( pChannel );

        // open channel for reading
        sFilename = sChannelPath + "/" + sChannelFileName + ".log";
        ifstream* pTimes = new ifstream(sFilename.c_str(), ios::in | ios::binary);
        if( !pTimes->is_open() ) {
            std::cerr << "Error reading timestamps files! Not keeping track of timestamps.\n" << std::endl;
            m_bReadTimestamps = false;
        } else {
            nStartPtr = m_uStartFrame * 8;
            pTimes->seekg( nStartPtr, ios::beg );
            m_vTimes.push_back( pTimes );
        }
    }

    _PrintCamInfo();

    // fill buffer
    for( unsigned int ii = 0; ii < m_uBufferSize; ii++ ) {	_Read(); }

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
    vImages.resize( m_uNumChannels );

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

        int ImgFormat = m_vCamerasInfo[ii]->format;

        if( m_vChannels[ii]->is_open() ) {

            if( ImgFormat == GRAY ) {
                vImages[ii].Image.create( m_vCamerasInfo[ii]->h, m_vCamerasInfo[ii]->w, CV_8UC1 );
                m_vChannels[ii]->read( (char*)vImages[ii].Image.data, m_vCamerasInfo[ii]->fsize );
            } else if( ImgFormat == RGB || ImgFormat == BGR ) {
                cv::Mat ImgColor( m_vCamerasInfo[ii]->h, m_vCamerasInfo[ii]->w, CV_8UC3 );
                m_vChannels[ii]->read( (char*)ImgColor.data, m_vCamerasInfo[ii]->fsize );
                cv::cvtColor( ImgColor, vImages[ii].Image, CV_RGB2GRAY );
            } else if( ImgFormat == BAYER || ImgFormat == BAYER_GB ) {
                cv::Mat ImgBayer( m_vCamerasInfo[ii]->h, m_vCamerasInfo[ii]->w, CV_8UC1 );
                m_vChannels[ii]->read( (char*)ImgBayer.data, m_vCamerasInfo[ii]->fsize );
                cv::cvtColor( ImgBayer, vImages[ii].Image, CV_BayerGB2GRAY );
            } else if( ImgFormat == BAYER_GR ) {
                cv::Mat ImgBayer( m_vCamerasInfo[ii]->h, m_vCamerasInfo[ii]->w, CV_8UC1 );
                m_vChannels[ii]->read( (char*)ImgBayer.data, m_vCamerasInfo[ii]->fsize );
                cv::cvtColor( ImgBayer, vImages[ii].Image, CV_BayerGR2GRAY );
            } else if( ImgFormat == BAYER_BG ) {
                cv::Mat ImgBayer( m_vCamerasInfo[ii]->h, m_vCamerasInfo[ii]->w, CV_8UC1 );
                m_vChannels[ii]->read( (char*)ImgBayer.data, m_vCamerasInfo[ii]->fsize );
                cv::cvtColor( ImgBayer, vImages[ii].Image, CV_BayerBG2GRAY );
            } else if( ImgFormat == BAYER_RG ) {
                cv::Mat ImgBayer( m_vCamerasInfo[ii]->h, m_vCamerasInfo[ii]->w, CV_8UC1 );
                m_vChannels[ii]->read( (char*)ImgBayer.data, m_vCamerasInfo[ii]->fsize );
                cv::cvtColor( ImgBayer, vImages[ii].Image, CV_BayerRG2GRAY );
            }


            if( m_bOutputRectified ) {
                cv::Mat rectImage;
                cv::remap( vImages[ii].Image, rectImage, m_vCamerasInfo[ii]->RectMapCol, m_vCamerasInfo[ii]->RectMapRow, CV_INTER_LINEAR );
                vImages[ii].Image = rectImage;
            }

            if( m_bReadTimestamps ) {
                double dTimestamp;
                m_vTimes[ii]->read( (char*)&dTimestamp, 8 );
                vImages[ii].Map.SetProperty("Timestamp", dTimestamp );
            }

        } else {
            mvl::PrintError( "ERROR file closed \n");
        }

    }

    m_uCurrentImageIndex++;

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
    return GRAY;
}

///////////////////////////////////////////////////////////////////////////////
void ToyotaReaderDriver::_PrintCamInfo( ) {
     for( unsigned int ii = 0; ii < m_uNumChannels; ++ii ) {
        std::cout << "========================" << std::endl;
        std::cout << " Channel [ " << ii << " ]" << std::endl;
        std::cout << "========================" << std::endl;
        std::cout <<  std::setw(8) << std::left << "w,h:";
        std:: cout << std::setw(4) << std::left << m_vCamerasInfo[ii]->w << " , " << m_vCamerasInfo[ii]->h << std::endl;
        std::cout <<  std::setw(8) << "fps:" <<  std::setw(12) << std::left << m_vCamerasInfo[ii]->fps << std::endl;
        std::cout <<  std::setw(8) << "format:" << std::setw(12) << std::left << m_vCamerasInfo[ii]->sformat << std::endl;
        std::cout <<  std::setw(8) << "name:" <<  std::setw(12) <<  std::left << m_vCamerasInfo[ii]->name << std::endl;
     }
}
