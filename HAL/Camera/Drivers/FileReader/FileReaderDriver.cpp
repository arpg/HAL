
#include "FileReaderDriver.h"

#include <HAL/Devices/DeviceTime.h>
#include <HAL/Devices/DeviceException.h>
#include <HAL/Utils/StringUtils.h>

#include <opencv2/opencv.hpp>

#include "ReadImage.h"

using namespace std;

namespace hal
{

///////////////////////////////////////////////////////////////////////////////
FileReaderDriver::~FileReaderDriver()
{
    m_bShouldRun = false;
    if( m_CaptureThread ) {
        while( !m_qImageBuffer.empty() ) {
            m_qImageBuffer.pop();
        }
        m_cBufferFull.notify_one();
        m_CaptureThread->join();
    }
}

///////////////////////////////////////////////////////////////////////////////
// Consumer
bool FileReaderDriver::Capture( pb::CameraMsg& vImages )
{
    if( m_nCurrentImageIndex >= m_nNumImages && m_qImageBuffer.size() == 0 && !m_bLoop) {
        return false;
    }

    std::unique_lock<std::mutex> lock(m_Mutex);

    // Wait until the buffer has data to read
    while(m_qImageBuffer.size() == 0) {
        m_cBufferEmpty.wait(lock);
    }

    DeviceTime::WaitForTime( _GetNextTime() );

    //***************************************************
    // consume from buffer
    //***************************************************


    // now fetch the next set of images from buffer
    vImages = m_qImageBuffer.front();

    // remove image from buffer queue
    m_qImageBuffer.pop();

    // send notification that the buffer has space
    m_cBufferFull.notify_one();

    // push next timestamp to queue now that we popped from the buffer
    DeviceTime::PopAndPushTime( _GetNextTime() );

    return true;
}

///////////////////////////////////////////////////////////////////////////////
FileReaderDriver::FileReaderDriver(const std::vector<std::string>& ChannelRegex, size_t StartFrame, bool Loop, size_t BufferSize, int cvFlags)
    : m_bShouldRun(false),
      m_nNumChannels(ChannelRegex.size()),
      m_nStartFrame(StartFrame),
      m_nCurrentImageIndex(StartFrame),
      m_bLoop(Loop),
      m_nBufferSize(BufferSize),
      m_iCvImageReadFlags(cvFlags)
{
    // clear variables if previously initialized
    m_vFileList.clear();

    if(m_nNumChannels < 1) {
        throw DeviceException( "No channels specified." );
    }

    m_vFileList.resize( m_nNumChannels );

    for( unsigned int ii = 0; ii < m_nNumChannels; ii++ ) {
        // Now generate the list of files for each channel
        std::vector< std::string >& vFiles = m_vFileList[ii];

        if( !hal::WildcardFileList(ChannelRegex[ii], vFiles) ) {
//        if( !hal::FindFiles(ChannelRegex[ii], vFiles) ){
            throw DeviceException("No files found from regexp", ChannelRegex[ii] );
        }
    }

    // make sure each channel has the same number of images
    m_nNumImages = m_vFileList[0].size();
    for( unsigned int ii = 1; ii < m_nNumChannels; ii++ ){
        if( m_vFileList[ii].size() != m_nNumImages ){
            throw DeviceException("Uneven number of files" );
        }
    }

    // fill buffer
    m_nHead = m_nTail = 0;
    m_vBuffer.resize( m_nBufferSize );
    for (unsigned int ii=0; ii < m_nBufferSize; ii++) {	_Read(); }

    // push timestamp of first image into the Virtual Device Queue
    DeviceTime::PushTime( _GetNextTime() );

    // run thread to keep buffer full
    m_bShouldRun = true;
    m_CaptureThread = new std::thread( &_ThreadCaptureFunc, this );
}

///////////////////////////////////////////////////////////////////////////////
// Producer
void FileReaderDriver::_ThreadCaptureFunc( FileReaderDriver* pFR )
{
    while( pFR->m_bShouldRun ) {
        if( !pFR->_Read() ) {
            break;
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
bool FileReaderDriver::_Read()
{
    std::unique_lock<std::mutex> lock(m_Mutex);

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

    pb::CameraMsg vImages;
    for( unsigned int ii = 0; ii < m_nNumChannels; ++ii ) {
        pb::ImageMsg* pbImg = vImages.add_image();
        sFileName = m_vFileList[ii][m_nCurrentImageIndex];
        cv::Mat cvImg = _ReadFile( sFileName, m_iCvImageReadFlags );

//        pb::ReadCvMat( cvImg, pbImg );
        pbImg->set_height( cvImg.rows );
        pbImg->set_width( cvImg.cols );

        // TODO this is BAD since 4 bytes can be int, or float, etc, etc
        if( cvImg.elemSize1() == 1 ) {
            pbImg->set_type( pb::PB_UNSIGNED_BYTE );
        }
        if( cvImg.elemSize1() == 2 ) {
            pbImg->set_type( pb::PB_UNSIGNED_SHORT );
        }
        if( cvImg.elemSize1() == 4 ) {
            pbImg->set_type( pb::PB_FLOAT );
        }

        if( cvImg.channels() == 1 ) {
            pbImg->set_format( pb::PB_LUMINANCE );
        }
        if( cvImg.channels() == 3 ) {
            pbImg->set_format( pb::PB_RGB );
        }
        pbImg->set_data( (const char*)cvImg.data, cvImg.rows * cvImg.cols * cvImg.elemSize1() * cvImg.channels() );
    }

    m_nCurrentImageIndex++;

    // add images at the back of the queue
    m_qImageBuffer.push(vImages);

    //*************************************************************************

    // send notification that the buffer is not empty
    m_cBufferEmpty.notify_one();

    return true;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double FileReaderDriver::_GetNextTime()
{
    if( m_qImageBuffer.size() == 0 ) {
        return -1;
    }
    return 0;
//    return m_qImageBuffer.front()[0].Map.GetProperty<double>( m_sTimeKeeper, 0 );
}

}
