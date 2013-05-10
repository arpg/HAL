
#include "WebcamDriver.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include "opencv2/highgui/highgui.hpp"	// for cap

#include <HAL/Utils/TicToc.h>

using namespace cv;
using namespace hal;

///////////////////////////////////////////////////////////////////////////////
WebcamDriver::WebcamDriver()
{

}

///////////////////////////////////////////////////////////////////////////////
WebcamDriver::~WebcamDriver()
{

}

///////////////////////////////////////////////////////////////////////////////
bool WebcamDriver::Capture( pb::CameraMsg& vImages )
{

    cv::Mat         cvImg;
    pb::ImageMsg*   pbImg = vImages.add_image();

    bool success = false;
    double systemTime = 0;
    int numChans = 0;

    if(m_bForceGreyscale) {
        static Mat temp;
        success = m_pCam.read(temp);
        systemTime = Tic();
        if(success) {
            cvtColor(temp,cvImg, CV_RGB2GRAY);
        }
        pbImg->set_format( pb::ImageMsg_Format_PB_LUMINANCE );
        numChans = 1;
    }else{
        success = m_pCam.read(cvImg);
        // TODO don't know why this is needed but it is =P
        cv::transpose( cvImg, cvImg );
        cv::transpose( cvImg, cvImg );
        systemTime = Tic();
        pbImg->set_format( pb::ImageMsg_Format_PB_RGB );
        numChans = 3;
    }

    pbImg->set_type(pb::ImageMsg_Type_PB_BYTE);
    pbImg->set_height( cvImg.rows );
    pbImg->set_width( cvImg.cols );
    pbImg->set_data( (const char*)cvImg.data, cvImg.rows * cvImg.cols * numChans );

    vImages.set_devicetime( systemTime );

    return success;
}


///////////////////////////////////////////////////////////////////////////////
void WebcamDriver::PrintInfo() {

    std::cout <<
    "\nWEBCAM\n"
    "--------------------------------\n"
    "Opens a webcam either built-in or connected via USB."
    "\n"
    "Options:\n"
    "   -camid          <id of webcam> [default 0]\n"
    "\n"
    "Flags:\n"
    "   -greyscale      If the driver should return images in greyscale.\n"
    "\n"
    "Example:\n"
    "./Exec  -idev Webcam   -greyscale\n\n";
}


///////////////////////////////////////////////////////////////////////////////
bool WebcamDriver::Init()
{
    assert(m_pPropertyMap);
    m_bForceGreyscale = m_pPropertyMap->GetProperty<bool>( "ForceGreyscale",  false );
    const int camId = m_pPropertyMap->GetProperty<bool>( "CamId",  0 );
    return m_pCam.open(camId);
}
