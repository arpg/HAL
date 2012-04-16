
#include "WebcamDriver.h"
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
bool WebcamDriver::Capture( std::vector<rpg::ImageWrapper>& vImages )
{

    // allocate images if necessary
    if( vImages.size() != 1 ){
        vImages.resize( 1 );
    }

    m_pCam >> vImages[0].Image;
    if( !vImages[0].Image.data )
	return false;

    return true;
}


///////////////////////////////////////////////////////////////////////////////
bool WebcamDriver::Init()
{
    return m_pCam.open(0);
}
