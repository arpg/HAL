/*
   \file FireFlyDriver.cpp
 */

#include <inttypes.h>
#include <stdio.h>
#include <stdint.h>
#include "FireFlyDriver.h"
#include <dc1394/conversions.h>
#include <mvl/image/image.h> // to rectify
#include <mvl/stereo/stereo.h>

///////////////////////////////////////////////////////////////////////////////
//  Releases the cameras and exits
void FireFlyDriver::_cleanup_and_exit( dc1394camera_t *pCam )
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
bool FireFlyDriver::Capture( std::vector<rpg::ImageWrapper>& vImages )
{

    // allocate images if necessary
    if( vImages.size() != 1 ){
        vImages.resize( 1 );
        // and setup images
        vImages[0].Image = cv::Mat(m_nImageHeight, m_nImageWidth, CV_8UC1);
    }

    //  capture one frame
    dc1394video_frame_t* pFrame;
    dc1394error_t e;
    e = dc1394_capture_dequeue( m_pCam, DC1394_CAPTURE_POLICY_WAIT, &pFrame );
    DC1394_ERR_CLN_RTN(e, _cleanup_and_exit(m_pCam),"Could not capture a frame");

    // TODO: the whole rectification process can be speeded up by not doing so many memcopies
//    mvl_image_t *img, *img_rect;

	// OpenCV image to MVL image
//    img = mvl_image_alloc( vImages[0].cols, vImages[0].rows, GL_UNSIGNED_BYTE, GL_LUMINANCE, vImages[0].data );

	// allocate space to hold rectified images
//    img_rect = mvl_image_alloc( vImages[0].cols, vImages[0].rows, GL_UNSIGNED_BYTE, GL_LUMINANCE,NULL );

    // rectify
//    mvl_rectify( m_pCamMod, img, img_rect );

	// MVL image to OpenCV image
    memcpy( vImages[0].Image.data, pFrame->image, vImages[0].Image.cols*vImages[0].Image.rows );


    // release the frame
    e = dc1394_capture_enqueue( m_pCam, pFrame );

    return true;
}


///////////////////////////////////////////////////////////////////////////////
bool FireFlyDriver::Init()
{
    assert(m_pPropertyMap);
    m_pPropertyMap->PrintPropertyMap();

    // get camera model from files
//    double pose[16];
//    std::string sPath =  m_pPropertyMap->GetProperty("DataSourceDir","");
//    std::string sCamModel = sPath + "/" + m_pPropertyMap->GetProperty("CamModel","");

//	m_pCamMod  = mvl_read_camera(sCamModel.c_str(), pose );

//	if( m_pCamMod == NULL ) {
//        std::cout << "Error reading camera model!\n" << std::endl;
//        return -1;
//    }

    // here we connect to the firefly and see if it's alive
    m_pBus = dc1394_new();
    dc1394error_t e;

    dc1394camera_list_t*  pCameraList = NULL;
    e = dc1394_camera_enumerate( m_pBus, &pCameraList );

    if( pCameraList->num == 0 ) {
        printf( "No cameras found!\n" );
        exit(-1);
    }

    for( int ii = 0; ii < (int)pCameraList->num; ii++) {
        m_pCam = dc1394_camera_new( m_pBus, pCameraList->ids[ii].guid );
        printf("Model %s\n", m_pCam->model );

        // the model
        if( m_pCam->model == std::string("FireFly")){
            // good
        }
        else{
            // should close cam?
        }
    }

    // free the camera list
    dc1394_camera_free_list( pCameraList );
    printf("Using camera with GUID %llu\n", m_pCam->guid );

    // always this
    m_nVideoMode = DC1394_VIDEO_MODE_640x480_MONO8;

    dc1394color_coding_t coding;
    e = dc1394_get_color_coding_from_video_mode( m_pCam, m_nVideoMode, &coding );
    DC1394_ERR_CLN_RTN(e,_cleanup_and_exit(m_pCam),"Could not get color coding");

    e = dc1394_video_set_iso_speed( m_pCam, DC1394_ISO_SPEED_400 );
    DC1394_ERR_CLN_RTN(e,_cleanup_and_exit(m_pCam),"Could not set iso speed");

    e = dc1394_video_set_mode( m_pCam, m_nVideoMode );
    DC1394_ERR_CLN_RTN( e, _cleanup_and_exit(m_pCam),"Could not set video mode");

    // get highest framerate
    dc1394framerates_t vFramerates;
    e = dc1394_video_get_supported_framerates( m_pCam, m_nVideoMode, &vFramerates);
    DC1394_ERR_CLN_RTN(e,_cleanup_and_exit(m_pCam),"Could not get framerates");
    m_nFramerate = vFramerates.framerates[vFramerates.num-1];

	e = dc1394_video_set_framerate( m_pCam, m_nFramerate );
    DC1394_ERR_CLN_RTN( e, _cleanup_and_exit(m_pCam),"Could not set framerate" );

    int nNumDMAChannels = 4;
    e = dc1394_capture_setup( m_pCam, nNumDMAChannels, DC1394_CAPTURE_FLAGS_DEFAULT );
    DC1394_ERR_CLN_RTN(e, _cleanup_and_exit(m_pCam), "Could not setup camera. Make sure that the video mode and framerate are supported by your camera." );

    // print camera features
    e = dc1394_feature_get_all( m_pCam, &m_vFeatures );
    if( e != DC1394_SUCCESS ) {
        dc1394_log_warning("Could not get feature set");
    }
    else {
//        dc1394_feature_print_all( &m_vFeatures, stdout );
    }

    // initiate transmission
    e = dc1394_video_set_transmission( m_pCam, DC1394_ON );
    DC1394_ERR_CLN_RTN( e, _cleanup_and_exit(m_pCam),"Could not start camera iso transmission");

    //  capture one frame
    dc1394video_frame_t* pFrame;
    e = dc1394_capture_dequeue( m_pCam, DC1394_CAPTURE_POLICY_WAIT, &pFrame );
    DC1394_ERR_CLN_RTN(e,_cleanup_and_exit(m_pCam),"Could not capture a frame");


    // capture an initial image to get the image sizes (SetProperty)

    // print capture image information. this is RAW
    m_nImageWidth = pFrame->size[0];
    m_nImageHeight = pFrame->size[1];
    /*
    printf("\nIMAGE INFORMATION:\n");
    printf("------------------------\n");
    printf("Image Size: %d x %d\n", m_nImageWidth, m_nImageHeight );
    printf("Data Depth: %d\n", pFrame->data_depth );
    printf("Stride: %d\n", pFrame->stride );
    printf("Total Bytes: %llu\n", pFrame->total_bytes );
    printf("------------------------\n");
    */

    // release the frame
    e = dc1394_capture_enqueue( m_pCam, pFrame );


    return true;
}
