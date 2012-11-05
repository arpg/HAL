/*
   \file FlycapDriver.cpp
 */

#include <inttypes.h>
#include <stdio.h>
#include <stdint.h>
#include "FlycapDriver.h"

using namespace std;
using namespace FlyCapture2;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void FlycapDriver::PrintError( Error error )
{
    error.PrintErrorTrace();
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline void FlycapDriver::CheckError( Error err )
{
    if( err != PGRERROR_OK ) {
       PrintError(err);
       exit(-1);
    }
}


///////////////////////////////////////////////////////////////////////////////
//  Releases the cameras and exits
FlycapDriver::FlycapDriver()
{
}

///////////////////////////////////////////////////////////////////////////////
//  Releases the cameras and exits
FlycapDriver::~FlycapDriver()
{
}


///////////////////////////////////////////////////////////////////////////////
bool FlycapDriver::Capture( std::vector<rpg::ImageWrapper>& vImages )
{
    Error error;

    // allocate images if necessary
    if( vImages.size() != 2 ){
        vImages.resize( 2 );
        // and setup images
        vImages[0].Image = cv::Mat(m_nImgHeight, m_nImgWidth, CV_8UC1);
        vImages[1].Image = cv::Mat(m_nImgHeight, m_nImgWidth, CV_8UC1);
    }

    Image Image1, Image2;

    error = m_Cam1.RetrieveBuffer( &Image1 );

    if( error != PGRERROR_OK ) {
        cerr << "Error grabbing camera 1 image." << endl;
    } else {
        memcpy( vImages[0].Image.data, Image1.GetData(), m_nImgWidth * m_nImgHeight );
    }

    error = m_Cam2.RetrieveBuffer( &Image2 );

    if( error != PGRERROR_OK ) {
        cerr << "Error grabbing camera 2 image." << endl;
    } else {
        memcpy( vImages[1].Image.data, Image2.GetData(), m_nImgWidth * m_nImgHeight );
    }

    return true;
}


///////////////////////////////////////////////////////////////////////////////
bool FlycapDriver::Init()
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

    assert(m_pPropertyMap);
    m_pPropertyMap->PrintPropertyMap();

    // initialize
    m_nImgWidth  = 640;
    m_nImgHeight = 510;

    Error error;

    BusManager          BusMgr;
    unsigned int        nNumCams;

    error = BusMgr.GetNumOfCameras( &nNumCams );
    CheckError(error);

    if( nNumCams != 2 ) {
       printf( "Two cameras are required to initialize this driver.\n" );
       return false;
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
    CheckError(error);

    // connect to camera 1
    error = m_Cam1.Connect(&GUID);
    CheckError(error);

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

    // initiate transmission
    error = m_Cam1.StartCapture();
    CheckError(error);

    error = m_Cam2.StartCapture();
    CheckError(error);

    return true;
}
