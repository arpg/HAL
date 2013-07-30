
#include "WebcamDriver.h"

#include <HAL/Utils/TicToc.h>

using namespace cv;
using namespace hal;

///////////////////////////////////////////////////////////////////////////////
WebcamDriver::WebcamDriver( unsigned int nCamId, bool bForceGrey )
{
    m_bForceGreyscale = bForceGrey;
    if( m_Cam.open(nCamId) == false ) {
        std::cerr << "HAL: Error opening webcam!" << std::endl;
    }

    cv::Mat         cvImg;
    if( m_Cam.read(cvImg) == false ) {
        std::cerr << "HAL: Error reading initial image!" << std::endl;
    }

    m_nImgHeight = cvImg.rows;
    m_nImgWidth = cvImg.cols;
}

///////////////////////////////////////////////////////////////////////////////
WebcamDriver::~WebcamDriver()
{

}

///////////////////////////////////////////////////////////////////////////////
bool WebcamDriver::Capture( pb::CameraMsg& vImages )
{
    if(m_Cam.isOpened() == false ) {
            std::cerr << "HAL: Error reading from camera." << std::endl;
            return false;
    }

    cv::Mat         cvImg;
    pb::ImageMsg*   pbImg = vImages.add_image();

    bool success = false;
    double systemTime = 0;
    int numChans = 0;

    if(m_bForceGreyscale) {
        static Mat temp;
        success = m_Cam.read(temp);
        systemTime = Tic();
        if(success) {
            cvtColor(temp, cvImg, CV_RGB2GRAY);
        }
        pbImg->set_format( pb::PB_LUMINANCE );
        numChans = 1;
    }else{
        success = m_Cam.read(cvImg);
        // TODO(jmf) don't know why this is needed but it is =P
        cv::transpose( cvImg, cvImg );
        cv::transpose( cvImg, cvImg );
        systemTime = Tic();
        pbImg->set_format( pb::PB_RGB );
        numChans = 3;
    }

    pbImg->set_type(pb::PB_UNSIGNED_BYTE);
    pbImg->set_height( m_nImgHeight );
    pbImg->set_width( m_nImgWidth );
    pbImg->set_data( (const char*)cvImg.data, m_nImgHeight * m_nImgWidth * numChans );

    vImages.set_devicetime( systemTime );

    return success;
}

size_t WebcamDriver::NumChannels() const
{
    return 1;
}

size_t WebcamDriver::Width( size_t /*idx*/) const
{
    return m_nImgWidth;
}

size_t WebcamDriver::Height( size_t /*idx*/) const
{
    return m_nImgHeight;
}
