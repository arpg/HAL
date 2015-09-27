/*
   \file FireFlyDriver.cpp
 */

#include <inttypes.h>
#include <stdio.h>
#include <stdint.h>

#include <dc1394/conversions.h>

#include <opencv.hpp>

#include "FireFlyDriver.h"

using namespace hal;

///////////////////////////////////////////////////////////////////////////////
void SetImageMetaDataFromCamera2( hal::ImageMsg* img, dc1394camera_t* pCam )
{
    hal::ImageInfoMsg* info = img->mutable_info();

    // obtain meta data from image
    dc1394error_t e;
    float feature;
    e = dc1394_feature_get_absolute_value( pCam, DC1394_FEATURE_SHUTTER, &feature );
    if( e == DC1394_SUCCESS ) {
        info->set_shutter( feature );
    }

    e = dc1394_feature_get_absolute_value( pCam, DC1394_FEATURE_GAIN, &feature );
    if( e == DC1394_SUCCESS ) {
        info->set_gain( feature );
    }

//    e = dc1394_feature_get_absolute_value( pCam, DC1394_FEATURE_GAMMA, &feature );
    if( e == DC1394_SUCCESS ) {
        info->set_gamma( feature );
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
void _cleanup_and_exit(dc1394camera_t *pCam)
{
    dc1394_video_set_transmission(pCam, DC1394_OFF);
    dc1394_capture_stop(pCam);
    dc1394_camera_free(pCam);
    exit(-1);
}

///////////////////////////////////////////////////////////////////////////////
void ShowCameraProperties(
        dc1394camera_t* camera
    )
{
    dc1394featureset_t features;
    dc1394_feature_get_all(camera, &features);
    dc1394_feature_print_all( &features, stdout );
}

///////////////////////////////////////////////////////////////////////////////
dc1394error_t SetCameraProperties(
        dc1394camera_t* camera,
        dc1394video_mode_t mode,
        dc1394framerate_t framerate = DC1394_FRAMERATE_30,
        dc1394speed_t   iso_speed = DC1394_ISO_SPEED_400,
        unsigned dma_channels = 4
    )
{
    dc1394error_t e;

    e = dc1394_video_set_operation_mode(camera, DC1394_OPERATION_MODE_1394B);
    DC1394_ERR_CLN_RTN(e, _cleanup_and_exit(camera), "Could not set operation mode");

    e = dc1394_video_set_iso_speed(camera, iso_speed );
    DC1394_ERR_CLN_RTN(e, _cleanup_and_exit(camera), "Could not set iso speed");

    e = dc1394_video_set_mode(camera, mode);
    DC1394_ERR_CLN_RTN(e, _cleanup_and_exit(camera), "Could not set video mode");

    e = dc1394_video_set_framerate(camera, framerate);
    DC1394_ERR_CLN_RTN(e, _cleanup_and_exit(camera), "Could not set framerate");

    e = dc1394_capture_setup(camera, dma_channels, DC1394_CAPTURE_FLAGS_DEFAULT);
    DC1394_ERR_CLN_RTN(e, _cleanup_and_exit(camera), "Could not setup camera. Make sure that the video mode and framerate are supported by your camera.");

    dc1394color_coding_t coding;
    e = dc1394_get_color_coding_from_video_mode( camera, mode, &coding );
    DC1394_ERR_CLN_RTN(e,_cleanup_and_exit(camera),"Could not get color coding");
//    std::cout << "Color Coding is: " << coding << std::endl;

    return DC1394_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
dc1394error_t SetCameraProperties_Format7(
        dc1394camera_t* camera,
        dc1394video_mode_t mode,
        dc1394color_coding_t coding,
        int packet_size = 4095,
        dc1394speed_t iso_speed = DC1394_ISO_SPEED_400,
        unsigned dma_channels = 4,
        unsigned int img_width = 640,
        unsigned int img_height = 480,
        unsigned int left = 0,
        unsigned int top = 0
    )
{
    dc1394error_t e;

    e = dc1394_video_set_operation_mode(camera, DC1394_OPERATION_MODE_1394B);
    DC1394_ERR_CLN_RTN(e, _cleanup_and_exit(camera), "Could not set operation mode");

    // set ISO speed
    e = dc1394_video_set_iso_speed(camera, iso_speed);
    DC1394_ERR_CLN_RTN(e, _cleanup_and_exit(camera), "Could not set iso speed");

    // set video mode
    e = dc1394_video_set_mode(camera, mode);
    DC1394_ERR_CLN_RTN(e, _cleanup_and_exit(camera), "Could not set video mode");

    // set image position
    e = dc1394_format7_set_image_position( camera, mode, left, top );
    DC1394_ERR_CLN_RTN(e, _cleanup_and_exit(camera), "Could not set image position.");

    // set image size
    e = dc1394_format7_set_image_size( camera, mode, img_width, img_height );
    DC1394_ERR_CLN_RTN(e, _cleanup_and_exit(camera), "Could not set image size.");

    // set color coding
    e = dc1394_format7_set_color_coding( camera, mode, coding );
    DC1394_ERR_CLN_RTN(e,_cleanup_and_exit(camera),"Could not set color coding.");

    // set ROI
    // FPS = 1 / ( ( img_width * img_height * bytes_per_pixel / packet_size ) * 0.000125 )
    e = dc1394_format7_set_roi( camera, mode, coding, packet_size, left, top, img_width, img_height );
    DC1394_ERR_CLN_RTN(e, _cleanup_and_exit(camera), "Could not set ROI.");

    // set some stuff
//    e = dc1394_feature_set_mode(camera, DC1394_FEATURE_SHUTTER, DC1394_FEATURE_MODE_MANUAL);
    DC1394_ERR_CLN_RTN(e,_cleanup_and_exit(camera),"Could not set manual framerate");

//    e = dc1394_feature_set_absolute_control(camera, DC1394_FEATURE_FRAME_RATE, DC1394_ON);
    DC1394_ERR_CLN_RTN(e,_cleanup_and_exit(camera),"Could not set absolute control for framerate");

//    e = dc1394_feature_set_absolute_value(camera, DC1394_FEATURE_SHUTTER, framerate);
    DC1394_ERR_CLN_RTN(e,_cleanup_and_exit(camera),"Could not set framerate value");


    // set DMA channel
    e = dc1394_capture_setup(camera, dma_channels, DC1394_CAPTURE_FLAGS_DEFAULT);
    DC1394_ERR_CLN_RTN(e, _cleanup_and_exit(camera), "Could not setup camera. Make sure that the video mode and framerate are supported by your camera.");

    return DC1394_SUCCESS;
}


///////////////////////////////////////////////////////////////////////////////
void FireFlyDriver::PrintInfo() {

    std::cout <<
    "\nFIREFLY\n"
    "--------------------------------\n"
    "Opens all available cameras (up to 5) using DC1394."
    "\n\n"
    "Options:\n"
    "   -sdir           <source directory for images and camera model files> [default '.']\n"
    "   -cmodN          <camera model file for camera N>\n"
    "\n"
    "Flags:\n"
    "   -rectify        If images should be rectified.\n"
    "\n"
    "Example:\n"
    "./Exec  -idev FireFly  -cmod0 cmod.xml\n\n";
}


///////////////////////////////////////////////////////////////////////////////
bool FireFlyDriver::Init()
{
    m_bOutputRectified = m_pPropertyMap->GetProperty( "Rectify", true );

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

        if(m_bOutputRectified) {
            std::string sPath =  m_pPropertyMap->GetProperty("DataSourceDir","");
            char            Index[10];
            sprintf( Index, "%d", m_nNumCams );
            std::string sCModelPrefix = "CamModel-";
            std::string sCModelName = sCModelPrefix + Index;
            std::string sCModel = sPath + "/" + m_pPropertyMap->GetProperty( sCModelName, "" );

            double pose[16];
            m_pCMod[ m_nNumCams ] = mvl_read_camera(sCModel.c_str(), pose );

            if( m_pCMod[ m_nNumCams ] == NULL ) {
                std::cerr << "Error reading camera model! Not rectifying." << std::endl;
                m_bOutputRectified = false;
            }
        }


        m_nNumCams++;
    }

    // Sort cameras into canonical order (so they are consistent each time
    // they are loaded).
    std::sort(m_pCam, m_pCam + m_nNumCams, dc1394CameraCompare);

    // free the camera list
    dc1394_camera_free_list( pCameraList );

    for( unsigned int ii = 0; ii < m_nNumCams; ii++ ) {
        printf("Configuring camera with GUID %llu ... ", m_pCam[ii]->guid );
        fflush( stdout );
        dc1394_camera_reset(m_pCam[ii]);
        // TODO: allow people to modify these parameters through property map!!!
//        if( SetCameraProperties( m_pCam[ii], DC1394_VIDEO_MODE_640x480_MONO8, DC1394_FRAMERATE_30 ) == DC1394_SUCCESS ) {
        if( SetCameraProperties_Format7( m_pCam[ii], DC1394_VIDEO_MODE_FORMAT7_0, DC1394_COLOR_CODING_MONO8,
                                         9200, DC1394_ISO_SPEED_400, 2, 640, 480, 56, 0 ) == DC1394_SUCCESS ) {
//            ShowCameraProperties(m_pCam[ii]);
            printf("OK.\n");
        }
    }

    // initiate transmission
    for( unsigned int ii = 0; ii < m_nNumCams; ii++ ) {
        e = dc1394_video_set_transmission( m_pCam[ii], DC1394_ON );
        DC1394_ERR_CLN_RTN( e, _cleanup_and_exit(m_pCam[ii]),"Could not start camera iso transmission");
    }

    //  capture one frame
    // note: If you are getting no captures, check that the ISO speed is OK!
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
bool FireFlyDriver::Capture( hal::CameraMsg& vImages )
{
    //  capture
    dc1394video_frame_t* pFrame;
    dc1394error_t e;


    for( unsigned int ii = 0; ii < m_nNumCams; ii++ ) {

        hal::ImageMsg* pbImg = vImages.add_image();

        e = dc1394_capture_dequeue( m_pCam[ii], DC1394_CAPTURE_POLICY_WAIT, &pFrame );
        DC1394_ERR_CLN_RTN( e, _cleanup_and_exit(m_pCam[ii]), "Could not capture a frame" );

        // Get capture time at ring buffer dequeue
        vImages.set_devicetime( (double)pFrame->timestamp * 1E-6 );

        // TODO: this has to be modified if the parameters are changed in the Init (multiply by num channels)
        cv::Mat cvImg( m_nImageHeight, m_nImageWidth, CV_8UC1 );
        memcpy( cvImg.data, pFrame->image, m_nImageWidth * m_nImageHeight );

        if(m_bOutputRectified) {
            cv::Mat Tmp;
            cv::Mat Intrinsics = (cv::Mat_<float>(3,3) <<    m_pCMod[ii]->warped.fx, 0, m_pCMod[ii]->warped.cx,
                                                              0, m_pCMod[ii]->warped.fy, m_pCMod[ii]->warped.cy,
                                                              0, 0, 1);
            cv::Mat Distortion = (cv::Mat_<float>(1,5) <<   m_pCMod[ii]->warped.kappa1, m_pCMod[ii]->warped.kappa2, m_pCMod[ii]->warped.tau1,
                                  m_pCMod[ii]->warped.tau2, m_pCMod[ii]->warped.kappa3);

            cv::undistort( cvImg, Tmp, Intrinsics, Distortion );
            cvImg = Tmp;
        }

        SetImageMetaDataFromCamera2( pbImg, m_pCam[ii] );
        pbImg->set_data( cvImg.data, m_nImageHeight * m_nImageWidth );

        e = dc1394_capture_enqueue( m_pCam[ii], pFrame );
    }

    return true;
}
