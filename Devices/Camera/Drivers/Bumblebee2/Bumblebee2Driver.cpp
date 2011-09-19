/*
   \file Bumblebee2Driver.cpp
 */

#include <inttypes.h>
#include <stdio.h>
#include <stdint.h>
#include "Bumblebee2Driver.h"
#include <Mvlpp/Utils.h>  // for FindFiles and PrintError
#include <dc1394/conversions.h>
#include "opencv/cv.h"	// for Mat structure

///////////////////////////////////////////////////////////////////////////////
//  Releases the cameras and exits
void cleanup_and_exit( dc1394camera_t *pCam )
{
    dc1394_video_set_transmission( pCam, DC1394_OFF );
    dc1394_capture_stop( pCam );
    dc1394_camera_free( pCam );
    exit(-1);
}


///////////////////////////////////////////////////////////////////////////////
//  Releases the cameras and exits
Bumblebee2Driver::Bumblebee2Driver() 
{
}

///////////////////////////////////////////////////////////////////////////////
//  Releases the cameras and exits
Bumblebee2Driver::~Bumblebee2Driver()
{
}


///////////////////////////////////////////////////////////////////////////////
bool Bumblebee2Driver::Capture( std::vector<cv::Mat>& vImages )
{

    // allocate images if neccessary
    if( vImages.size() != 2 ){
        vImages.resize( 2 ); 
        // and setup images
//        vImages[0] = cv::Mat(m_nImageHeight*2, m_nImageWidth, CV_8UC1);
        vImages[0] = cv::Mat(m_nImageHeight/2, m_nImageWidth/2, CV_8UC1);
        vImages[1] = cv::Mat(m_nImageHeight/2, m_nImageWidth/2, CV_8UC1);
    }
    // should also check images are the right size, type etc

    //  capture one frame
    dc1394video_frame_t* pFrame;
    dc1394error_t e;
    e = dc1394_capture_dequeue( m_pCam, DC1394_CAPTURE_POLICY_WAIT, &pFrame );
    DC1394_ERR_CLN_RTN(e, cleanup_and_exit(m_pCam),"Could not capture a frame");

    uint8_t buff[m_nImageWidth*m_nImageHeight*2];
    // trying stuff
    dc1394_deinterlace_stereo( pFrame->image, buff, m_nImageWidth, m_nImageHeight*2 );

    // copy to our buffer, split, decimate, convert, etc.
    int a,b,c,d;
    for( int ii = 0; ii < m_nImageHeight; ii+=2 ) {
    	for( int jj = 0; jj < m_nImageWidth; jj+=2 ) {
    		a = buff[ (ii * m_nImageWidth) + jj   ];
    		b = buff[ (ii * m_nImageWidth) + jj + 1 ];
    		c = buff[((ii + 1) * m_nImageWidth) + jj ];
    		d = buff[((ii + 1) * m_nImageWidth) + jj + 1 ];
    		vImages[0].data[ ((ii/2)*(m_nImageWidth/2))+(jj/2) ] = ( a + b + c + d) / 4;
    		a = buff[ ((ii + m_nImageHeight) * m_nImageWidth) + jj   ];
    		b = buff[ ((ii + m_nImageHeight) * m_nImageWidth) + jj + 1 ];
    		c = buff[(((ii + m_nImageHeight) + 1) * m_nImageWidth) + jj ];
    		d = buff[(((ii + m_nImageHeight) + 1) * m_nImageWidth) + jj + 1 ];
    		vImages[1].data[ ((ii/2)*(m_nImageWidth/2))+(jj/2) ] = ( a + b + c + d) / 4;
    	}
    }

    // release the frame
    e = dc1394_capture_enqueue( m_pCam, pFrame );

    return true;
}


///////////////////////////////////////////////////////////////////////////////
bool Bumblebee2Driver::Init()
{
    assert(m_pPropertyMap);
    m_pPropertyMap->PrintPropertyMap();

    // here we connect to the bb2 and see if it's alive
    m_pBus = dc1394_new();
    dc1394error_t e;

    dc1394camera_list_t*  pCameraList = NULL;
    e = dc1394_camera_enumerate( m_pBus, &pCameraList );
    for( int ii = 0; ii < pCameraList->num; ii++) {
        m_pCam = dc1394_camera_new( m_pBus, pCameraList->ids[ii].guid );
        printf("Model %s\n", m_pCam->model );
        if( m_pCam->model == std::string("Bumblebee2")){
            // good
        }
        else{
            // should close cam 
        }
    }

    // free the camera lsit
    dc1394_camera_free_list( pCameraList );
    printf("Using camera with GUID %llu\n", m_pCam->guid );

    // always this
    m_nVideoMode = DC1394_VIDEO_MODE_FORMAT7_3;

    dc1394color_coding_t coding;
    e = dc1394_get_color_coding_from_video_mode( m_pCam, m_nVideoMode, &coding );
    DC1394_ERR_CLN_RTN(e,cleanup_and_exit(m_pCam),"Could not get color coding");

    // capure an initial image to get the image sizes (SetProperty)

    // get highest framerate
//    dc1394framerates_t vFramerates;
//    e = dc1394_video_get_supported_framerates( m_pCam, m_nVideoMode,&vFramerates);
//    DC1394_ERR_CLN_RTN(e,cleanup_and_exit(m_pCam),"Could not get framrates");
//    m_nFramerate = vFramerates.framerates[vFramerates.num-1];

    e = dc1394_video_set_iso_speed( m_pCam, DC1394_ISO_SPEED_400 );
    DC1394_ERR_CLN_RTN(e,cleanup_and_exit(m_pCam),"Could not set iso speed");

    e = dc1394_video_set_mode( m_pCam, m_nVideoMode );
    DC1394_ERR_CLN_RTN( e, cleanup_and_exit(m_pCam),"Could not set video mode");

//    e = dc1394_video_set_framerate( m_pCam, m_nFramerate );
//    DC1394_ERR_CLN_RTN( e, cleanup_and_exit(m_pCam),"Could not set framerate" );

    int nNumDMAChannels = 4;
    e = dc1394_capture_setup( m_pCam, nNumDMAChannels,
            DC1394_CAPTURE_FLAGS_DEFAULT );
    DC1394_ERR_CLN_RTN(e,cleanup_and_exit(m_pCam),"Could not setup camera. make sure that the video mode and framerate are supported by your camera.");

    // print camera features
    e = dc1394_feature_get_all( m_pCam, &m_vFeatures );
    if( e != DC1394_SUCCESS ) {
        dc1394_log_warning("Could not get feature set");
    }
    else {
        dc1394_feature_print_all( &m_vFeatures, stdout );
    }

    // initiate transmission
    e = dc1394_video_set_transmission( m_pCam, DC1394_ON );
    DC1394_ERR_CLN_RTN( e, cleanup_and_exit(m_pCam),"Could not start camera iso transmission");

    //  capture one frame
    dc1394video_frame_t* pFrame;
    e = dc1394_capture_dequeue( m_pCam, DC1394_CAPTURE_POLICY_WAIT, &pFrame );
    DC1394_ERR_CLN_RTN(e,cleanup_and_exit(m_pCam),"Could not capture a frame");


    // copy to our buffer, decimate, convert, etc.
    // print image information
    m_nImageWidth = pFrame->size[0];
    m_nImageHeight = pFrame->size[1];
    printf("\nIMAGE INFORMATION:\n");
    printf("------------------------\n");
    printf("Image Size: %d x %d\n", m_nImageWidth, m_nImageHeight );
    printf("Data Depth: %d\n", pFrame->data_depth );
    printf("Stride: %d\n", pFrame->stride );
    printf("Total Bytes: %llu\n", pFrame->total_bytes );
    printf("------------------------\n");

    // release the frame
    e = dc1394_capture_enqueue( m_pCam, pFrame );


    return false; 
}
