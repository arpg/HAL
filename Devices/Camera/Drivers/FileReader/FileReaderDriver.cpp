
#include "FileReaderDriver.h"
#include <Mvlpp/Utils.h>  // for FindFiles and PrintError
#include <boost/format.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "RPG/Devices/Camera/Drivers/Dvi2Pci/SDK/includes/s_fio.h"	// for imread()

//using namespace boost;
using namespace std;

///////////////////////////////////////////////////////////////////////////////
FileReaderDriver::FileReaderDriver(){}

///////////////////////////////////////////////////////////////////////////////
FileReaderDriver::~FileReaderDriver()
{
    m_CaptureThread->interrupt();
    m_CaptureThread->join();
}

///////////////////////////////////////////////////////////////////////////////
// Consumer
bool FileReaderDriver::Capture( std::vector<rpg::ImageWrapper>& vImages )
{
    if( m_nCurrentImageIndex >= m_nNumImages && m_qImageBuffer.size() == 0 && !m_bLoop) {
        return false;
    }

    boost::mutex::scoped_lock lock(m_Mutex);

    // Wait until the buffer has data to read
    while(m_qImageBuffer.size() == 0){
        m_cBufferEmpty.wait(lock);
    }

    //***************************************************
    // consume from buffer
    //***************************************************

    // allocate images if necessary
    if( vImages.size() != m_nNumChannels )
        vImages.resize( m_nNumChannels );

    // now fetch the next set of images from buffer
    for( unsigned int ii = 0; ii < m_nNumChannels; ii++ )
        vImages[ii].Image = m_qImageBuffer.front()[ii].Image.clone();


    // remove image from queue
    m_qImageBuffer.pop();

    //***************************************************

    // send notification that the buffer has space
    m_cBufferFull.notify_one();

    return true;
}


///////////////////////////////////////////////////////////////////////////////
bool FileReaderDriver::Init()
{
    // clear variables if previously initialized
     //m_qImageBuffer.clear();
     m_vFileList.clear();


    assert(m_pPropertyMap);
//    m_pPropertyMap->PrintPropertyMap();

    m_nNumChannels       = m_pPropertyMap->GetProperty<unsigned int>( "NumChannels", 0 );
    m_nBufferSize        = m_pPropertyMap->GetProperty<unsigned int>( "BufferSize", 35 );
    m_nStartFrame        = m_pPropertyMap->GetProperty<unsigned int>( "StartFrame",  0 );
    m_bLoop              = m_pPropertyMap->GetProperty<bool>( "Loop",  false );
    m_nCurrentImageIndex = m_nStartFrame;


    if(m_nNumChannels < 1) {
        mvl::PrintError( "ERROR: No channels specified. Set property NumChannels.\n" );
        exit(1);
    }

    m_vFileList.resize( m_nNumChannels );

    // Get data path
     std::string sChannelPath = m_pPropertyMap->GetProperty( "DataSourceDir", "");

    for( unsigned int ii = 0; ii < m_nNumChannels; ii++ ) {
        //std::cerr << "SlamThread: Finding files channel " << ii << std::endl;
        std::string sChannelName  = (boost::format("Channel-%d")%ii).str();
        std::string sChannelRegex = m_pPropertyMap->GetProperty( sChannelName, "");

        // check if regular expression has a subdirectory
        size_t pos = sChannelRegex.find("/");
        std::string sSubDirectory;

        if(pos != string::npos)
        {
            sSubDirectory = sChannelRegex.substr(0,pos);
            sChannelRegex = sChannelRegex.substr(pos+1);
        }

        // Now generate the list of files for each channel
        std::vector< std::string>& vFiles = m_vFileList[ii];

        if(mvl::FindFiles(sChannelPath + "/" + sSubDirectory, sChannelRegex, vFiles) == false){
        //if( mvl::FindFiles( sChannelRegex, vFiles ) == false ) {
            mvl::PrintError( "ERROR: No files found from regexp\n" );
            exit(1);
        }
    }

    // make sure each channel has the same number of images
    m_nNumImages = m_vFileList[0].size();
    for( unsigned int ii = 1; ii < m_nNumChannels; ii++ ){
        if( m_vFileList[ii].size() != m_nNumImages ){
            mvl::PrintError( "ERROR: uneven number of files\n" );
            exit(1);
        }
    }

    for (unsigned int ii=0; ii < m_nBufferSize; ii++) {	_Read(); }


    m_CaptureThread = new boost::thread( &_ThreadCaptureFunc, this );

    return true;
}

///////////////////////////////////////////////////////////////////////////////
// Producer
void FileReaderDriver::_ThreadCaptureFunc( FileReaderDriver* pFR )
{
    while(1){
        try {
            boost::this_thread::interruption_point();

            if(!pFR->_Read()) {
                break;
            }

        } catch( boost::thread_interrupted& interruption ) {
            break;
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
//void FileReaderDriver::_Read( std::vector<rpg::ImageWrapper>& vImages)
bool FileReaderDriver::_Read()
{
    boost::mutex::scoped_lock lock(m_Mutex);

    // Wait until there is space in the buffer
    while(! (m_qImageBuffer.size() < m_nBufferSize) ){
        m_cBufferFull.wait(lock);
    }

    //*************************************************************************
    // produce to buffer
    //*************************************************************************

    // loop over if we finished our files!
    if( m_nCurrentImageIndex == m_nNumImages ) {
        if(m_bLoop == true) {
            m_nCurrentImageIndex = m_nStartFrame;
        }else{
            return false;
        }
    }

    // now fetch the next set of images
    std::string sFileName;
    std::vector<rpg::ImageWrapper> vImages;
    vImages.resize(m_nNumChannels);

    for( unsigned int ii = 0; ii < m_nNumChannels; ii++ ) {
        sFileName = m_vFileList[ii][m_nCurrentImageIndex];
        // TODO: this only reads grayscale '0'.. not sure if we need more than that tho
        std::string sExtension = sFileName.substr( sFileName.rfind( "." ) + 1 );
        // check if it is our own "portable depth map" format
        if( sExtension == "pdm" ) {
            vImages[ii].Image = _OpenPDM( sFileName );
        } else {
            // otherwise let OpenCV open it
            vImages[ii].Image = cv::imread( sFileName, 0);
        }
    }

    m_nCurrentImageIndex++;

    // add images at the back of the queue
    m_qImageBuffer.push(vImages);

    //*************************************************************************

    // send notification that the buffer is not empty
    m_cBufferEmpty.notify_one();

    return true;
}

cv::Mat FileReaderDriver::_OpenPDM( const std::string& FileName )
{
    // magic number P7, portable depthmap, binary
    ifstream File( FileName.c_str() );

    unsigned int nImgWidth;
    unsigned int nImgHeight;
    long unsigned int nImgSize;

    cv::Mat DepthImg;

    if( File.is_open() ) {
        string sType;
        File >> sType;
        File >> nImgWidth;
        File >> nImgHeight;
        File >> nImgSize;
//		nImgSize++;

//		nImgSize = (log( nImgSize ) / log(2)) / 8.0;
        nImgSize = 4 * nImgWidth * nImgHeight;
        DepthImg = cv::Mat( nImgHeight, nImgWidth, CV_32FC1 );
        File.seekg(File.tellg()+(ifstream::pos_type)1, ios::beg);
        File.read( (char*)DepthImg.data, nImgSize );
        File.close();
    }
    return DepthImg;
}
