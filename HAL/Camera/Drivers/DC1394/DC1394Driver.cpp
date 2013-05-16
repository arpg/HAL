//#include <inttypes.h>
//#include <stdio.h>
//#include <stdint.h>
/*
 * Format7 code extracted from Pangolin library.
 *
 */


#include <iostream>
#include <dc1394/conversions.h>

#include <HAL/Devices/DeviceException.h>

#include "DC1394Driver.h"


using namespace hal;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DC1394Driver::_SetImageMetaDataFromCamera( pb::ImageMsg* img, dc1394camera_t* pCam )
{
    pb::ImageInfoMsg* info = img->mutable_info();

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



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline int DC1394Driver::_NearestValue(int value, int step, int min, int max)
{
    int low, high;

    low=value-(value%step);
    high=value-(value%step)+step;
    if (low<min)
        low=min;
    if (high>max)
        high=max;

    if (abs(low-value)<abs(high-value))
        return low;
    else
        return high;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
DC1394Driver::DC1394Driver(
        unsigned int            nCamId,
        dc1394video_mode_t      Mode,
        unsigned int            nTop,
        unsigned int            nLeft,
        unsigned int            nWidth,
        unsigned int            nHeight,
        float                   fFPS,
        dc1394speed_t           ISO,
        unsigned int            nDMA
    )
{
    m_pBus = dc1394_new();
    dc1394error_t e;

    dc1394camera_list_t*  pCameraList = NULL;
    e = dc1394_camera_enumerate( m_pBus, &pCameraList );

    if( pCameraList->num == 0 ) {
        throw DeviceException("No cameras found!");
    }

    m_pCam = dc1394_camera_new( m_pBus, pCameraList->ids[nCamId].guid );

    // free the camera list
    dc1394_camera_free_list( pCameraList );

    printf("Configuring camera with GUID %llu ... ", m_pCam->guid );
    fflush( stdout );
    dc1394_camera_reset(m_pCam);


    //----- set ISO speed
    if(ISO >= DC1394_ISO_SPEED_800)
    {
        e = dc1394_video_set_operation_mode( m_pCam, DC1394_OPERATION_MODE_1394B );
        if( e != DC1394_SUCCESS )
            throw DeviceException("Could not set DC1394_OPERATION_MODE_1394B");
    }

    e = dc1394_video_set_iso_speed( m_pCam, ISO);
    if( e != DC1394_SUCCESS )
        throw DeviceException("Could not set iso speed");


    //----- set framerate and mode
    if( Mode < DC1394_VIDEO_MODE_FORMAT7_MIN ) {
        // "regular" mode

        e = dc1394_video_set_mode( m_pCam, Mode);
        if( e != DC1394_SUCCESS )
            throw DeviceException("Could not set video mode");

        // get framerate
        dc1394framerate_t FPS;
        if( fFPS == 1.875 ) {
            FPS = DC1394_FRAMERATE_1_875;
        } else if( fFPS == 3.75 ) {
            FPS = DC1394_FRAMERATE_3_75;
        } else if( fFPS == 7.5 ) {
            FPS = DC1394_FRAMERATE_7_5;
        } else if( fFPS == 15 ) {
            FPS = DC1394_FRAMERATE_15;
        } else if( fFPS == 30 ) {
            FPS = DC1394_FRAMERATE_30;
        } else if( fFPS == 60 ) {
            FPS = DC1394_FRAMERATE_60;
        } else if( fFPS == 120 ) {
            FPS = DC1394_FRAMERATE_120;
        } else {
            FPS = DC1394_FRAMERATE_240;
        }

        e = dc1394_video_set_framerate(m_pCam, FPS);
        if( e != DC1394_SUCCESS )
            throw DeviceException("Could not set frame rate");

    } else {
        // format 7 mode

        // check that the required mode is actually supported
        dc1394format7mode_t Info;

        e = dc1394_format7_get_mode_info( m_pCam, Mode, &Info);
        if( e != DC1394_SUCCESS )
            throw DeviceException("Could not get format7 mode info");

        if( Info.present == false )
            throw DeviceException("Format7 mode info not present");

        // safely set the video mode
        e = dc1394_video_set_mode( m_pCam, Mode );
        if( e != DC1394_SUCCESS )
            throw DeviceException("Could not set format7 video mode");

        // set position to 0,0 so that setting any size within min and max is a valid command
        e = dc1394_format7_set_image_position(m_pCam, Mode, 0, 0);
        if( e != DC1394_SUCCESS )
            throw DeviceException("Could not set format7 image position");

        // work out the desired image size
        nWidth = _NearestValue(nWidth, Info.unit_pos_x, 0, Info.max_size_x - nLeft);
        nHeight = _NearestValue(nHeight, Info.unit_pos_y, 0, Info.max_size_y - nTop);

        // set size
        e = dc1394_format7_set_image_size(m_pCam, Mode, nWidth, nHeight);
        if( e != DC1394_SUCCESS )
            throw DeviceException("Could not set format7 size");

        // get the info again since many parameters depend on image size
        e = dc1394_format7_get_mode_info(m_pCam, Mode, &Info);
        if( e != DC1394_SUCCESS )
            throw DeviceException("Could not get format7 mode info");

        // work out position of roi
        nLeft = _NearestValue(nLeft, Info.unit_size_x, Info.unit_size_x, Info.max_size_x - nWidth);
        nTop = _NearestValue(nTop, Info.unit_size_y, Info.unit_size_y, Info.max_size_y - nHeight);

        // set roi position
        e = dc1394_format7_set_image_position(m_pCam, Mode, nLeft, nTop);
        if( e != DC1394_SUCCESS )
            throw DeviceException("Could not set format7 size");

        std::cout<<"roi: "<<nLeft<<" "<<nTop<<" "<<nWidth<<" "<<nHeight<<"  ";


        e = dc1394_format7_set_packet_size(m_pCam, Mode, Info.max_packet_size);
        if( e != DC1394_SUCCESS )
            throw DeviceException("Could not set format7 packet size");

        if((fFPS != MAX_FR) && (fFPS != EXT_TRIG)){
            //set the framerate by using the absolute feature as suggested by the
            //folks at PointGrey
            e = dc1394_feature_set_absolute_control(m_pCam,DC1394_FEATURE_FRAME_RATE,DC1394_ON);
            if( e != DC1394_SUCCESS )
                throw DeviceException("Could not turn on absolute frame rate control");

            e = dc1394_feature_set_mode(m_pCam,DC1394_FEATURE_FRAME_RATE,DC1394_FEATURE_MODE_MANUAL);
            if( e != DC1394_SUCCESS )
                throw DeviceException("Could not make frame rate manual ");

            e = dc1394_feature_set_absolute_value(m_pCam,DC1394_FEATURE_FRAME_RATE,fFPS);
            if( e != DC1394_SUCCESS )
                throw DeviceException("Could not set format7 framerate ");
        }

        // ask the camera what is the resulting framerate (this assume that such a rate is actually
        // allowed by the shutter time)
        float value;
        e = dc1394_feature_get_absolute_value(m_pCam,DC1394_FEATURE_FRAME_RATE,&value);
        if( e != DC1394_SUCCESS )
            throw DeviceException("Could not get framerate");

        std::cout << " framerate(shutter permitting): " << value << std::endl;
    }


    //----- set DMA channels
    e = dc1394_capture_setup(m_pCam, nDMA, DC1394_CAPTURE_FLAGS_DEFAULT);
    if( e != DC1394_SUCCESS )
        throw DeviceException("Could not setup camera - check settings");

    printf("OK.\n");


    // initiate transmission
    e = dc1394_video_set_transmission( m_pCam, DC1394_ON );
    if( e != DC1394_SUCCESS )
        throw DeviceException("Could not start camera iso transmission");

    // capture one frame
    // note: If you are getting no captures, check that the ISO speed is OK!
    dc1394video_frame_t* pFrame;
    e = dc1394_capture_dequeue( m_pCam, DC1394_CAPTURE_POLICY_WAIT, &pFrame );
    if( e != DC1394_SUCCESS )
        throw DeviceException("Could not capture a frame");


    // image information. this is RAW
    m_nImageWidth = pFrame->size[0];
    m_nImageHeight = pFrame->size[1];

    if( pFrame->color_coding == DC1394_COLOR_CODING_MONO8  ) {
        m_VideoFormat = pb::PB_LUMINANCE;
        m_VideoType = pb::PB_UNSIGNED_BYTE;
    } else if( pFrame->color_coding == DC1394_COLOR_CODING_MONO16  ) {
        m_VideoFormat = pb::PB_LUMINANCE;
        m_VideoType = pb::PB_UNSIGNED_SHORT;
    } else if( pFrame->color_coding == DC1394_COLOR_CODING_RGB8  ) {
        m_VideoFormat = pb::PB_RGB;
        m_VideoType = pb::PB_UNSIGNED_BYTE;
    } else {
        m_VideoFormat = pb::PB_RAW;
        m_VideoType = pb::PB_UNSIGNED_BYTE;
    }

    // release the frame
    e = dc1394_capture_enqueue( m_pCam, pFrame );
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool DC1394Driver::Capture( pb::CameraMsg& vImages )
{
    // clear protobuf
    vImages.Clear();

    //  capture
    dc1394video_frame_t* pFrame;
    dc1394error_t e;

    pb::ImageMsg* pbImg = vImages.add_image();

    e = dc1394_capture_dequeue( m_pCam, DC1394_CAPTURE_POLICY_WAIT, &pFrame );
    if( e != DC1394_SUCCESS )
        return false;

    vImages.set_devicetime( (double)pFrame->timestamp * 1E-6 );

    pbImg->set_width( m_nImageWidth );
    pbImg->set_height( m_nImageHeight );
    pbImg->set_data( pFrame->image, pFrame->image_bytes );
    pbImg->set_type( m_VideoType );
    pbImg->set_format( m_VideoFormat );

    // add image properties: gain, gamma, etc
    _SetImageMetaDataFromCamera( pbImg, m_pCam );

    e = dc1394_capture_enqueue( m_pCam, pFrame );

    return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::string DC1394Driver::GetDeviceProperty(const std::string& /*sProperty*/)
{
    return std::string();
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned int DC1394Driver::Width( unsigned int /*idx*/ )
{
    return m_nImageWidth;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned int DC1394Driver::Height( unsigned int /*idx*/ )
{
    return m_nImageHeight;
}
