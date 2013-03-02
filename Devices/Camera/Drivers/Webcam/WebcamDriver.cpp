
#include "WebcamDriver.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
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

    bool success = false;
    
    if(m_bForceGreyscale) {
        static Mat temp;
        success = m_pCam.retrieve(temp);
        cvtColor(temp,vImages[0].Image, CV_RGB2GRAY);
    }else{
        success = m_pCam.retrieve(vImages[0].Image);
    }
    
    return success;
}


///////////////////////////////////////////////////////////////////////////////
bool WebcamDriver::Init()
{
    assert(m_pPropertyMap);    
    m_bForceGreyscale = m_pPropertyMap->GetProperty<bool>( "ForceGreyscale",  false );
    bool success = m_pCam.open(0);
    if(success) {
        m_pCam.set(CV_CAP_PROP_CONVERT_RGB,1);
    }
    return success;
}
