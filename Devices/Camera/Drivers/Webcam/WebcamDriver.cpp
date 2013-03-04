
#include "WebcamDriver.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include "opencv2/highgui/highgui.hpp"	// for cap

#include <RPG/Utils/TicToc.h>

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
    double systemTime = 0;
    
    if(m_bForceGreyscale) {
        static Mat temp;
        success = m_pCam.read(temp);
        systemTime = Tic();
        if(success) {
            cvtColor(temp,vImages[0].Image, CV_RGB2GRAY);
        }
    }else{
        success = m_pCam.read(vImages[0].Image);
        systemTime = Tic();
    }

    vImages[0].Map.SetProperty("SystemTime", systemTime );
    
    return success;
}


///////////////////////////////////////////////////////////////////////////////
bool WebcamDriver::Init()
{
    assert(m_pPropertyMap);    
    m_bForceGreyscale = m_pPropertyMap->GetProperty<bool>( "ForceGreyscale",  false );
    
    return m_pCam.open(0);
}
