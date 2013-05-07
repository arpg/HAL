
#include "WebcamDriver.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include "opencv2/highgui/highgui.hpp"	// for cap

#include <RPG/Utils/TicToc.h>

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

    if(m_bForceGreyscale) {
        static Mat temp;
        success = m_pCam.read(temp);
        systemTime = Tic();
        if(success) {
            cvtColor(temp,cvImg, CV_RGB2GRAY);
        }
        pbImg->set_format( pb::ImageMsg_Format_PB_LUMINANCE );
    }else{
        success = m_pCam.read(cvImg);
        systemTime = Tic();
        pbImg->set_format( pb::ImageMsg_Format_PB_RGB );
    }

    pbImg->set_type(pb::ImageMsg_Type_PB_BYTE);
    pbImg->set_height( cvImg.rows );
    pbImg->set_width( cvImg.cols );
    pbImg->set_data( (const char*)cvImg.data );

    vImages.set_devicetime( systemTime );

    return success;
}


///////////////////////////////////////////////////////////////////////////////
void WebcamDriver::PrintInfo() {

    std::cout <<
    "FILEREADER\n"
    "Reads images from the disk."
    "\n"
    "Options:\n"
    "   -sdir           <source directory for images and camera model files> [default '.']\n"
    "   -lfile          <regular expression for left image channel>\n"
    "   -rfile          <regular expression for right image channel>\n"
    "   -lcmod          <left camera model xml file>\n"
    "   -rcmod          <right camera model xml file>\n"
    "   -sf             <start frame> [default 0]\n"
    "   -numchan        <number of channels> [default 2]\n"
    "   -buffsize       <size of buffer for image pre-read> [default 35]\n"
    "   -timekeeper     <name of variable holding image timestamps> [default 'SystemTime]\n"
    "\n"
    "Flags:\n"
    "   -greyscale      If the driver should return images in greyscale.\n"
    "   -loop           If the driver should restart once images are consumed.\n"
    "\n"
    "Example:\n"
    "./Exec  -idev FileReader  -lcmod lcmod.xml  -rcmod rcmod.xml  -lfile \"left.*pgm\"  -rfile \"right.*pgm\"\n\n";
}


///////////////////////////////////////////////////////////////////////////////
bool WebcamDriver::Init()
{
    assert(m_pPropertyMap);
    m_bForceGreyscale = m_pPropertyMap->GetProperty<bool>( "ForceGreyscale",  false );
    const int camId = m_pPropertyMap->GetProperty<bool>( "CamId",  false );
    return m_pCam.open(camId);
}
