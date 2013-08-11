#include <iostream>

#include "FlycapDriver.h"

using namespace hal;
using namespace FlyCapture2;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline void FlycapDriver::_CheckError( Error err )
{
    if( err != PGRERROR_OK ) {
       err.PrintErrorTrace();
       exit(EXIT_FAILURE);
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///  Releases the cameras and exits
FlycapDriver::FlycapDriver()
{
    /*
     * Camera Properties:
     * Format 7 Mode 1
     * Image Width: 640
     * Image Height: 510
     * Packet Size: 5280
     *
     * GPIO2 is strobing (red cable - pin3)
     * GPIO3 is external trigger (green cable - pin4)
     * Grounds have to be connected (blue cables - pin6)
     *
     */

    // initialize
    m_nImgWidth  = 640;
    m_nImgHeight = 480;
//    m_nImgWidth  = 1280;
//    m_nImgHeight = 960;

    Error error;

    BusManager          BusMgr;
    unsigned int        nNumCams;

    error = BusMgr.GetNumOfCameras( &nNumCams );
    _CheckError(error);

    if( nNumCams == 0 ) {
       std::cerr << "HAL: No cameras found!" << std::endl;
       return;
    }

    // prepare Format 7 config
    Format7ImageSettings F7Config;
    F7Config.mode = MODE_1;
    F7Config.height = m_nImgHeight;
    F7Config.width = m_nImgWidth;
    F7Config.offsetX = 0;
    F7Config.offsetY = 0;
    F7Config.pixelFormat = PIXEL_FORMAT_RAW8;
    unsigned int PacketSize = 5280;
    unsigned int MaxPacketSize = 20416;

    // image properties
    Property P_Shutter;
    P_Shutter.type = SHUTTER;
    P_Shutter.onOff = true;
    P_Shutter.autoManualMode = false;
    P_Shutter.absControl = true;
    P_Shutter.absValue = 8;

    Property P_Exposure;
    P_Exposure.type = AUTO_EXPOSURE;
    P_Exposure.onOff = false;
    P_Exposure.autoManualMode = false;
    P_Exposure.absControl = true;
    P_Exposure.absValue = -3.5;

    Property P_Gain;
    P_Gain.type = GAIN;
    P_Gain.onOff = true;
    P_Gain.autoManualMode = false;
    P_Gain.absControl = true;
    P_Gain.absValue = 3;

    Property P_Brightness;
    P_Brightness.type = BRIGHTNESS;
    P_Brightness.onOff = true;
    P_Brightness.autoManualMode = false;
    P_Brightness.absControl = false;
    P_Brightness.absValue = 50;

    Property P_Sharpness;
    P_Sharpness.type = SHARPNESS;
    P_Sharpness.onOff = true;
    P_Sharpness.autoManualMode = false;
    P_Sharpness.absControl = false;
    P_Sharpness.absValue = 0;

    PGRGuid         GUID;

    // look for camera 1
    error = BusMgr.GetCameraFromIndex(0, &GUID);
    _CheckError(error);

    // connect to camera 1
    error = m_Cam1.Connect(&GUID);
    _CheckError(error);

//    m_Cam1.SetVideoModeAndFrameRate(VIDEOMODE_1280x960Y8, FRAMERATE_60);
    m_Cam1.SetVideoModeAndFrameRate(VIDEOMODE_640x480Y8, FRAMERATE_60);

    /*
    // set video mode and framerate
    error = m_Cam1.SetFormat7Configuration( &F7Config, PacketSize );
    CheckError(error);
    error = m_Cam1.SetProperty( &P_Shutter );
    CheckError(error);
    error = m_Cam1.SetProperty( &P_Exposure );
    CheckError(error);
    error = m_Cam1.SetProperty( &P_Gain );
    CheckError(error);
    error = m_Cam1.SetProperty( &P_Brightness );
    CheckError(error);
    error = m_Cam1.SetProperty( &P_Sharpness );
    CheckError(error);

    // prepare trigger
    TriggerMode Trigger;

    // external trigger is disabled
    Trigger.onOff = false;
    Trigger.mode = 0;
    Trigger.parameter = 0;
    Trigger.source = 0;

    // set trigger
    error = m_Cam1.SetTriggerMode( &Trigger );

    // prepare strobe
    StrobeControl Strobe;

    // set GPIO2 direction to out
    m_Cam1.SetGPIOPinDirection( 2, 1 );

    // set GPIO2 as strobe
    Strobe.onOff = true;
    Strobe.source = 2;
    Strobe.delay = 0;
    Strobe.duration = 0;
    Strobe.polarity = 0;

    error = m_Cam1.SetStrobe( &Strobe );
    CheckError(error);
    */

    /*
    // look for camera 2
    error = BusMgr.GetCameraFromIndex(1, &GUID);
    CheckError(error);

    // connect to camera 2
    error = m_Cam2.Connect(&GUID);
    CheckError(error);

    // set video mode and framerate
    error = m_Cam2.SetFormat7Configuration( &F7Config, MaxPacketSize );
    CheckError(error);
    error = m_Cam2.SetProperty( &P_Shutter );
    CheckError(error);
    error = m_Cam2.SetProperty( &P_Exposure );
    CheckError(error);
    error = m_Cam2.SetProperty( &P_Gain );
    CheckError(error);
    error = m_Cam2.SetProperty( &P_Brightness );
    CheckError(error);
    error = m_Cam2.SetProperty( &P_Sharpness );
    CheckError(error);

    // set GPIO3 direction to in
    m_Cam2.SetGPIOPinDirection( 3, 0 );

    // set camera to trigger mode 0
    // use GPIO3 as external trigger
    Trigger.onOff = true;
    Trigger.mode = 0;
    Trigger.parameter = 0;
    Trigger.source = 3;

    error = m_Cam2.SetTriggerMode( &Trigger );
    CheckError(error);
    */

    // initiate transmission
    error = m_Cam1.StartCapture();
    _CheckError(error);

//    error = m_Cam2.StartCapture();
//    CheckError(error);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///  Releases the cameras and exits
FlycapDriver::~FlycapDriver()
{
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool FlycapDriver::Capture(  pb::CameraMsg& vImages )
{
    Error error;

    Image Image1, Image2;

    error = m_Cam1.RetrieveBuffer( &Image1 );

    if( error != PGRERROR_OK ) {
        std::cerr << "HAL: Error grabbing camera 1 image." << std::endl;
        std::cerr << "Error was: " << error.GetDescription() << std::endl;
        return false;
    } else {

        // set timestamp
        vImages.set_device_time( Image1.GetMetadata().embeddedTimeStamp );

        pb::ImageMsg* pbImg = vImages.add_image();
        pbImg->set_width( Image1.GetCols() );
        pbImg->set_height( Image1.GetRows() );
        pbImg->set_data( Image1.GetData(), Image1.GetDataSize() );
        pbImg->set_type( pb::PB_UNSIGNED_BYTE );
        pbImg->set_format( pb::PB_LUMINANCE );
    }

    /*
    error = m_Cam2.RetrieveBuffer( &Image2 );

    if( error != PGRERROR_OK ) {
        std::cerr << "HAL: Error grabbing camera 2 image." << std::endl;
        return false;
    } else {
        pb::ImageMsg* pbImg = vImages.add_image();
        pbImg->set_width( Image2.GetCols() );
        pbImg->set_height( Image2.GetRows() );
        pbImg->set_data( Image2.GetData(), Image2.GetDataSize() );
        pbImg->set_type( pb::PB_UNSIGNED_BYTE );
        pbImg->set_format( pb::PB_LUMINANCE );
    }
    */

    return true;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::string FlycapDriver::GetDeviceProperty(const std::string& /*sProperty*/)
{
    return std::string();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
size_t FlycapDriver::NumChannels() const
{
    return 1;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
size_t FlycapDriver::Width( size_t /*idx*/ ) const
{
    return m_nImgWidth;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
size_t FlycapDriver::Height( size_t /*idx*/ ) const
{
    return m_nImgHeight;
}
