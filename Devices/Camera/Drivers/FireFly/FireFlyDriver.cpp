/*
   \file FireFlyDriver.cpp
 */

#include <inttypes.h>
#include <stdio.h>
#include <stdint.h>

#include <dc1394/conversions.h>
#include <mvl/image/image.h> // to rectify
#include <mvl/stereo/stereo.h>

#include "FireFlyDriver.h"


///////////////////////////////////////////////////////////////////////////////
void SetImageMetaDataFromCamera2(rpg::ImageWrapper& img, dc1394camera_t* pCam)
{
    // obtain meta data from image
    dc1394error_t e;
    float feature;
    e = dc1394_feature_get_absolute_value( pCam, DC1394_FEATURE_SHUTTER, &feature );
    if( e == DC1394_SUCCESS ) {
        img.Map.SetProperty("Shutter", feature );
    }

    e = dc1394_feature_get_absolute_value( pCam, DC1394_FEATURE_GAIN, &feature );
    if( e == DC1394_SUCCESS ) {
        img.Map.SetProperty("Gain", feature );
    }

    e = dc1394_feature_get_absolute_value( pCam, DC1394_FEATURE_GAMMA, &feature );
    if( e == DC1394_SUCCESS ) {
        img.Map.SetProperty("Gamma", feature );
    }
}


///////////////////////////////////////////////////////////////////////////////
//  Releases the cameras and exits
inline void FireFlyDriver::_cleanup_and_exit( dc1394camera_t *pCam )
{
    dc1394_video_set_transmission( pCam, DC1394_OFF );
    dc1394_capture_stop( pCam );
    dc1394_camera_free( pCam );
    exit(-1);
}


///////////////////////////////////////////////////////////////////////////////
//  Releases the cameras and exits
FireFlyDriver::FireFlyDriver()
{
}


///////////////////////////////////////////////////////////////////////////////
//  Releases the cameras and exits
FireFlyDriver::~FireFlyDriver()
{
}

///////////////////////////////////////////////////////////////////////////////
//  Less-than function for ordering dx1394 cameras on the bus.
//  This allows cameras to be returned in a consistent order.
bool dc1394CameraCompare(dc1394camera_t* c1, dc1394camera_t* c2) {
    return c1->guid < c2->guid;
}

///////////////////////////////////////////////////////////////////////////////
bool FireFlyDriver::Init()
{
    assert(m_pPropertyMap);
    m_pPropertyMap->PrintPropertyMap();

//    std::string sPath =  m_pPropertyMap->GetProperty("DataSourceDir","");
//    std::string sCamModel = sPath + "/" + m_pPropertyMap->GetProperty("CamModel","");

    // here we connect to the firefly and see if it's alive
    m_pBus = dc1394_new();
    dc1394error_t e;

    dc1394camera_list_t*  pCameraList = NULL;
    e = dc1394_camera_enumerate( m_pBus, &pCameraList );

    if( pCameraList->num == 0 ) {
        printf( "No cameras found!\n" );
        exit(-1);
    }

    m_nNumCams = 0;

    dc1394camera_t* pCam;
    for( int ii = 0; ii < (int)pCameraList->num; ii++) {
        pCam = dc1394_camera_new( m_pBus, pCameraList->ids[ii].guid );

        if( m_nNumCams == 5) {
            std::cerr << "warning: Maximum of 5 cameras can be initialized." << std::endl;
            break;
        }
        m_pCam[ m_nNumCams ] = pCam;
        m_nNumCams++;
    }
    
    // Sort cameras into canonical order (so they are consistent each time
    // they are loaded).
    std::sort(m_pCam, m_pCam + m_nNumCams, dc1394CameraCompare);

    // free the camera list
    dc1394_camera_free_list( pCameraList );

    for( unsigned int ii = 0; ii < m_nNumCams; ii++ ) {
        printf("Using camera with GUID %llu\n", m_pCam[ii]->guid );
    }

    // always this
    m_nVideoMode = DC1394_VIDEO_MODE_640x480_MONO8;

    dc1394color_coding_t coding;
    e = dc1394_get_color_coding_from_video_mode( m_pCam[0], m_nVideoMode, &coding );
    DC1394_ERR_CLN_RTN(e,_cleanup_and_exit(m_pCam[0]),"Could not get color coding");

    for( unsigned int ii = 0; ii < m_nNumCams; ii++ ) {
        e = dc1394_video_set_iso_speed( m_pCam[ii], DC1394_ISO_SPEED_400 );
        DC1394_ERR_CLN_RTN(e,_cleanup_and_exit(m_pCam[ii]),"Could not set iso speed");

        e = dc1394_video_set_mode( m_pCam[ii], m_nVideoMode );
        DC1394_ERR_CLN_RTN( e, _cleanup_and_exit(m_pCam[ii]),"Could not set video mode");
    }

    // get highest framerate
//    dc1394framerates_t vFramerates;
//    e = dc1394_video_get_supported_framerates( m_pCam[0], m_nVideoMode, &vFramerates);
//    DC1394_ERR_CLN_RTN(e,_cleanup_and_exit(m_pCam[0]),"Could not get framerates");
//    m_nFramerate = vFramerates.framerates[vFramerates.num-1];
    m_nFramerate = DC1394_FRAMERATE_30;

    for( unsigned int ii = 0; ii < m_nNumCams; ii++ ) {
        e = dc1394_video_set_framerate( m_pCam[ii], m_nFramerate );
        DC1394_ERR_CLN_RTN( e, _cleanup_and_exit(m_pCam[ii]),"Could not set framerate" );
    }

    int nNumDMAChannels = m_pPropertyMap->GetProperty( "DMA", 4 );

    std::cout << "NumCams: " << m_nNumCams << std::endl;

    for( unsigned int ii = 0; ii < m_nNumCams; ii++ ) {
        e = dc1394_capture_setup( m_pCam[ii], nNumDMAChannels, DC1394_CAPTURE_FLAGS_DEFAULT );
        DC1394_ERR_CLN_RTN(e, _cleanup_and_exit(m_pCam[ii]), "Could not setup camera. Make sure that the video mode and framerate are supported by your camera." );
    }

    // initiate transmission
    for( unsigned int ii = 0; ii < m_nNumCams; ii++ ) {
        e = dc1394_video_set_transmission( m_pCam[ii], DC1394_ON );
        DC1394_ERR_CLN_RTN( e, _cleanup_and_exit(m_pCam[ii]),"Could not start camera iso transmission");
    }

    //  capture one frame
    dc1394video_frame_t* pFrame;
    e = dc1394_capture_dequeue( m_pCam[0], DC1394_CAPTURE_POLICY_WAIT, &pFrame );
    DC1394_ERR_CLN_RTN(e,_cleanup_and_exit(m_pCam[0]),"Could not capture a frame");


    // capture an initial image to get the image sizes (SetProperty)

    // print capture image information. this is RAW
    m_nImageWidth = pFrame->size[0];
    m_nImageHeight = pFrame->size[1];

    // release the frame
    e = dc1394_capture_enqueue( m_pCam[0], pFrame );


    return true;
}


///////////////////////////////////////////////////////////////////////////////
bool FireFlyDriver::Capture( std::vector<rpg::ImageWrapper>& vImages )
{

    // allocate images if necessary
    if( vImages.size() != m_nNumCams ){
        vImages.resize( m_nNumCams );
        // and setup images
        for( unsigned int ii = 0; ii < m_nNumCams; ii++ ) {
            vImages[ii].Image = cv::Mat(m_nImageHeight, m_nImageWidth, CV_8UC1);
        }
    }

    //  capture
    dc1394video_frame_t* pFrame;
    dc1394error_t e;

    for( unsigned int ii = 0; ii < m_nNumCams; ii++ ) {
        e = dc1394_capture_dequeue( m_pCam[ii], DC1394_CAPTURE_POLICY_WAIT, &pFrame );
        DC1394_ERR_CLN_RTN(e, _cleanup_and_exit(m_pCam[ii]),"Could not capture a frame");
        memcpy( vImages[ii].Image.data, pFrame->image, m_nImageWidth * m_nImageHeight );
        SetImageMetaDataFromCamera2( vImages[ii], m_pCam[ii] );
        e = dc1394_capture_enqueue( m_pCam[ii], pFrame );
    }

    return true;
}
