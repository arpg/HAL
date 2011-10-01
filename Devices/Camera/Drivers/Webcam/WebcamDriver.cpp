
#include "WebcamDriver.h"
#include "opencv/cv.h"	// for Mat structure
#include "opencv2/highgui/highgui.hpp"	// for cap 

using namespace cv;

///////////////////////////////////////////////////////////////////////////////
WebcamDriver::WebcamDriver()
{

}

///////////////////////////////////////////////////////////////////////////////
WebcamDriver::~WebcamDriver()
{

}

///////////////////////////////////////////////////////////////////////////////
bool WebcamDriver::Capture( std::vector<cv::Mat>& vImages )
{

    // allocate images if neccessary
    if( vImages.size() != 1 ){
        vImages.resize( 1 ); 
    }

    m_pCam >> vImages[0];
    if( !vImages[0].data )
	return false;

    return true;
}


///////////////////////////////////////////////////////////////////////////////
bool WebcamDriver::Init()
{
    return m_pCam.open(0);
//    if( !m_pCam->isOpened() )  // check if we succeeded
//	    return false;
//    return true;
}
