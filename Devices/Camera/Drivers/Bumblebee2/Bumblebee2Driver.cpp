/*
   \file Bumblebee2Driver.cpp
 */

#include <inttypes.h>
#include <stdio.h>
#include <stdint.h>
#include "Bumblebee2Driver.h"
#include <dc1394/conversions.h>
#include <mvl/image/image.h> // to rectify
#include <mvl/stereo/stereo.h>

///////////////////////////////////////////////////////////////////////////////
//  Releases the cameras and exits
void Bumblebee2Driver::_cleanup_and_exit( dc1394camera_t *pCam )
{
    dc1394_video_set_transmission( pCam, DC1394_OFF );
    dc1394_capture_stop( pCam );
    dc1394_camera_free( pCam );
    exit(-1);
}

///////////////////////////////////////////////////////////////////////////////
//  Releases the cameras and exits
Bumblebee2Driver::Bumblebee2Driver()
    : m_pDeinterlaceBuffer(0), m_pDebayerBuffer(0),
      m_pLeftCMod(0), m_pRightCMod(0)
{
}

///////////////////////////////////////////////////////////////////////////////
//  Releases the cameras and exits
Bumblebee2Driver::~Bumblebee2Driver()
{
    dc1394_video_set_transmission( m_pCam, DC1394_OFF );
    dc1394_capture_stop( m_pCam );
    dc1394_camera_free( m_pCam );
	
	if(m_pDeinterlaceBuffer) delete m_pDeinterlaceBuffer;
    if(m_pDebayerBuffer) delete m_pDebayerBuffer;
}

///////////////////////////////////////////////////////////////////////////////
void SetImageMetaDataFromCamera(rpg::ImageWrapper& img, dc1394camera_t* pCam)
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

// TODO: refactor into MVL?
void bayer8_to_grey8_half(unsigned char* src, unsigned char* dst, unsigned int srcWidth, unsigned int srcHeight )
{
    for( int ii = 0; ii < (int)srcHeight; ii+=2 ) {
        for( int jj = 0; jj < (int)srcWidth; jj+=2 ) {
            const unsigned int a = src[ (ii * srcWidth) + jj   ];
            const unsigned int b = src[ (ii * srcWidth) + jj + 1 ];
            const unsigned int c = src[((ii + 1) * srcWidth) + jj ];
            const unsigned int d = src[((ii + 1) * srcWidth) + jj + 1 ];                
            dst[ ((ii/2)*(srcWidth/2))+(jj/2) ] = ( a + b + c + d) / 4;
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
bool Bumblebee2Driver::Capture( std::vector<rpg::ImageWrapper>& vImages )
{
	
    // allocate images if necessary
    if( vImages.size() != 2 ){
        vImages.resize( 2 );
    }
    
    // should also check images are the right size, type etc
    if(vImages[0].Image.rows != (int)m_uImageHeight/2  || vImages[0].Image.cols != (int)m_uImageWidth/2 ) {
        // and setup images
        vImages[0].Image = cv::Mat(m_uImageHeight/2, m_uImageWidth/2, m_nCvOutputType);
        vImages[1].Image = cv::Mat(m_uImageHeight/2, m_uImageWidth/2, m_nCvOutputType);
    }    
    
    SetImageMetaDataFromCamera(vImages[0], m_pCam);
	
    //  capture one frame
    dc1394video_frame_t* pFrame;
    dc1394error_t e = dc1394_capture_dequeue( m_pCam, DC1394_CAPTURE_POLICY_WAIT, &pFrame );
    if(e != DC1394_SUCCESS) {
        std::cerr << "Could not capture frame" << std::endl;
        return false;
    }
        
    // Deinterlace into buffer
    dc1394_deinterlace_stereo( pFrame->image, m_pDeinterlaceBuffer, m_uImageWidth, m_uImageHeight*2 );    
    
    if( m_nCvOutputType == CV_8UC3 ) {
        // Debayer and downsample into RGB image
        dc1394_bayer_decoding_8bit(m_pDeinterlaceBuffer,vImages[0].Image.data, m_uImageWidth, m_uImageHeight, DC1394_COLOR_FILTER_GRBG, DC1394_BAYER_METHOD_DOWNSAMPLE );        
        dc1394_bayer_decoding_8bit(m_pDeinterlaceBuffer+m_uImageHeight*m_uImageWidth,vImages[1].Image.data, m_uImageWidth, m_uImageHeight, DC1394_COLOR_FILTER_GRBG, DC1394_BAYER_METHOD_DOWNSAMPLE );        

        // TODO: Allow rectification
        assert(m_bOutputRectified == false);
    } else {
        // Debayer and (optionally) rectify each camera
        for(unsigned int cam=0; cam < 2; ++cam) {
            unsigned char* imgBayer = m_pDeinterlaceBuffer + cam*m_uImageWidth*m_uImageHeight;
            unsigned char* imgGrey  = m_bOutputRectified ? m_pDebayerBuffer : vImages[cam].Image.data;
            
            // Debayer into Greyscale, half-sampled.
            bayer8_to_grey8_half(imgBayer, imgGrey, m_uImageWidth, m_uImageHeight);
            
            if(m_bOutputRectified) {
                // MVL image wrapper around OpenCV Rectified / Unrectified data
                mvl_image_t* img = mvl_image_alloc( vImages[cam].Image.cols, vImages[cam].Image.rows,  GL_UNSIGNED_BYTE, GL_LUMINANCE, imgGrey );
                mvl_image_t* img_rect = mvl_image_alloc( vImages[cam].Image.cols, vImages[cam].Image.rows,  GL_UNSIGNED_BYTE, GL_LUMINANCE, vImages[cam].Image.data );
                mvl_rectify( m_pLeftCMod, img, img_rect );
            }
            
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

    // get camera model from files
    double pose[16];
    
    m_bOutputRectified = m_pPropertyMap->GetProperty("Rectify", true);
    m_nCvOutputType = m_pPropertyMap->GetProperty("ForceGreyscale", true) ? CV_8UC1 : CV_8UC3;    
            
    if(m_bOutputRectified) {
        std::string sPath =  m_pPropertyMap->GetProperty("DataSourceDir","");
        std::string sLeftCModel = sPath + "/" + m_pPropertyMap->GetProperty("CamModel-L","lcmod.xml");
        std::string sRightCModel = sPath + "/" + m_pPropertyMap->GetProperty("CamModel-R","rcmod.xml");        
        
        m_pLeftCMod  = mvl_read_camera(sLeftCModel.c_str(), pose );
        m_pRightCMod = mvl_read_camera(sRightCModel.c_str(), pose );
     
        if( m_pLeftCMod == NULL || m_pRightCMod == NULL ) {
            std::cerr << "Error reading camera model! Not rectifying.\n" << std::endl;
            m_bOutputRectified = false;
        }
    }

    // here we connect to the bb2 and see if it's alive
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

        // the model actually returns more info than Bumblebee2
        if( m_pCam->model == std::string("Bumblebee2")){
            // good
        }
        else{
            // should close cam?
        }
    }

    // free the camera lsit
    dc1394_camera_free_list( pCameraList );
    printf("Using camera with GUID %llu\n", m_pCam->guid );

    // always this
    m_nVideoMode = DC1394_VIDEO_MODE_FORMAT7_3;

    dc1394color_coding_t coding;
    e = dc1394_get_color_coding_from_video_mode( m_pCam, m_nVideoMode, &coding );
    DC1394_ERR_CLN_RTN(e,_cleanup_and_exit(m_pCam),"Could not get color coding");

    // capure an initial image to get the image sizes (SetProperty)

    // get highest framerate (not supported on BB apparently)
//    dc1394framerates_t vFramerates;
//    e = dc1394_video_get_supported_framerates( m_pCam, m_nVideoMode,&vFramerates);
//    DC1394_ERR_CLN_RTN(e,cleanup_and_exit(m_pCam),"Could not get framrates");
//    m_nFramerate = vFramerates.framerates[vFramerates.num-1];

    e = dc1394_video_set_iso_speed( m_pCam, DC1394_ISO_SPEED_400 );
    DC1394_ERR_CLN_RTN(e,_cleanup_and_exit(m_pCam),"Could not set iso speed");

    e = dc1394_video_set_mode( m_pCam, m_nVideoMode );
    DC1394_ERR_CLN_RTN( e, _cleanup_and_exit(m_pCam),"Could not set video mode");

//    e = dc1394_video_set_framerate( m_pCam, m_nFramerate );
//    DC1394_ERR_CLN_RTN( e, cleanup_and_exit(m_pCam),"Could not set framerate" );

    int nNumDMAChannels = 4;
    e = dc1394_capture_setup( m_pCam, nNumDMAChannels,
    DC1394_CAPTURE_FLAGS_DEFAULT );
    DC1394_ERR_CLN_RTN(e,_cleanup_and_exit(m_pCam),"Could not setup camera. make sure that the video mode and framerate are supported by your camera.");

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

    // copy to our buffer, decimate, convert, etc.

    // print capture image information. this is RAW, interlaced and in Format7
    m_uImageWidth = pFrame->size[0];
    m_uImageHeight = pFrame->size[1];
    
    // alloc memory for deinterlacing buffer
    m_pDeinterlaceBuffer = new unsigned char[pFrame->total_bytes];
    m_pDebayerBuffer = new unsigned char[m_uImageWidth*m_uImageHeight / 4];
	
    /*
    printf("\nIMAGE INFORMATION:\n");
    printf("------------------------\n");
    printf("Image Size: %d x %d\n", m_uImageWidth, m_uImageHeight );
    printf("Data Depth: %d\n", pFrame->data_depth );
    printf("Stride: %d\n", pFrame->stride );
    printf("Total Bytes: %llu\n", pFrame->total_bytes );
    printf("------------------------\n");
    */

    // release the frame
    e = dc1394_capture_enqueue( m_pCam, pFrame );
    
    return true;
}
