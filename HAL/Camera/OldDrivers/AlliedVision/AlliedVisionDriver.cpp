/*
   \file AlliedVisionDriver.cpp
 */

#include <inttypes.h>
#include <stdio.h>
#include <stdint.h>
#include <algorithm>
#include "AlliedVisionDriver.h"
#include <mvl/image/image.h> // to rectify
#include <mvl/stereo/stereo.h>

const int NUM_BUFFER_FRAMES = 15;
const int MAX_RETRIES = 10;
const int MAX_CAMERA_LIST_SIZE = 20;
const int RETRY_WAIT_MS = 200;
const std::string PROPERTY_CAM_UUID  = "CamUUID";
const std::string PROPERTY_NUM_CAMS  = "NumChannels";
const std::string PROPERTY_WIDTH     = "ImageWidth";
const std::string PROPERTY_HEIGHT    = "ImageHeight";
const std::string PROPERTY_BINX      = "ImageBinningX";
const std::string PROPERTY_BINY      = "ImageBinningY";

// TODO: Set these properly!
#define _LINUX 1
#define _x64 1

#include <PvApi.h>

#if defined(_LINUX) || defined(_QNX) || defined(_OSX)
//Define function equivalent to Windows Sleep
void Sleep(unsigned int time)
{
    struct timespec t,r;
    t.tv_sec    = time / 1000;
    t.tv_nsec   = (time % 1000) * 1000000;
    while(nanosleep(&t,&r)==-1)
        t = r;
}
#endif

struct AlliedVisionCamera
{
    AlliedVisionCamera(unsigned long UID)
        : m_uid(UID), m_handle(0), m_nextFrame(0), m_frameSizeBytes(0), m_width(0), m_height(0)
    {
        memset(m_frames,0, sizeof(tPvFrame) * NUM_BUFFER_FRAMES);
    }

    ~AlliedVisionCamera()
    {
        for(int i=0; i<NUM_BUFFER_FRAMES; ++i ) {
            if(m_frames[i].ImageBuffer)
                delete [] (char*)m_frames[i].ImageBuffer;
        }
    }

    tPvUint32   m_uid;
    tPvHandle   m_handle;

    tPvFrame    m_frames[NUM_BUFFER_FRAMES];
    tPvUint32   m_nextFrame;

    tPvUint32   m_frameSizeBytes;
    tPvUint32   m_width;
    tPvUint32   m_height;
};

AlliedVisionCamera* GetCameraById(unsigned long uuid)
{
    tPvErr err = ePvErrUnplugged;

    tPvCameraInfoEx camInfo;
    camInfo.UniqueId = 0;

    for(unsigned retries = MAX_RETRIES; err != ePvErrSuccess && retries > 0; retries-- ) {
        Sleep(RETRY_WAIT_MS);
        err = PvCameraInfoEx(uuid, &camInfo, sizeof(tPvCameraInfoEx) );
    }

    if( err == ePvErrSuccess ) {
        return new AlliedVisionCamera(uuid);
    }else{
        std::cerr << "Camera with UUID " << uuid << " not found." << std::endl;
    }

    return 0;
}

AlliedVisionCamera* AlliedVisionDriver::GetFirstCamera()
{
    tPvUint32 numCameras = 0;
    tPvCameraInfoEx list[MAX_CAMERA_LIST_SIZE];

    // Find Connected cameras
    for(unsigned retries = MAX_RETRIES; retries > 0; retries-- ) {
        Sleep(RETRY_WAIT_MS);
        numCameras = PvCameraListEx(list,MAX_CAMERA_LIST_SIZE,0, sizeof(tPvCameraInfoEx));
        if(numCameras > 0) break;
    }

    for(unsigned i=0; i<numCameras; ++i ) {
        bool alreadyLoaded = false;
        for(std::vector<AlliedVisionCamera*>::iterator ic = m_cam.begin(); ic != m_cam.end(); ic++) {
            if( (*ic)->m_uid == list[i].UniqueId ) {
                alreadyLoaded = true;
                break;
            }
        }
        if(!alreadyLoaded) {
            return new AlliedVisionCamera(list[i].UniqueId);
        }
    }

    return 0;
}

///////////////////////////////////////////////////////////////////////////////
AlliedVisionDriver::AlliedVisionDriver()
{
}

///////////////////////////////////////////////////////////////////////////////
//  Releases the cameras and exits
AlliedVisionDriver::~AlliedVisionDriver()
{
    Deinit();
}

///////////////////////////////////////////////////////////////////////////////
void AlliedVisionDriver::PrintInfo() {

    std::cout <<
    "FILEREADER\n"
    "Reads images from the disk."
    "\n"
    "Options:\n"
    "   -sdir           <source directory for images and camera model files> [default '.']\n"
    "   -lfile          <regular expression for left image channel>\n"
    "   -rfile          <regular expression for right image channel>\n"
    "   -lcmod          <left camera model xml file>\n"
    "   -rcmod          <right camera model xml file>\n"
    "   -sf             <start frame> [default 0]\n"
    "   -numchan        <number of channels> [default 2]\n"
    "   -buffsize       <size of buffer for image pre-read> [default 35]\n"
    "   -timekeeper     <name of variable holding image timestamps> [default 'SystemTime]\n"
    "\n"
    "Flags:\n"
    "   -greyscale      If the driver should return images in greyscale.\n"
    "   -loop           If the driver should restart once images are consumed.\n"
    "\n"
    "Example:\n"
    "./Exec  -idev FileReader  -lcmod lcmod.xml  -rcmod rcmod.xml  -lfile \"left.*pgm\"  -rfile \"right.*pgm\"\n\n";
}


bool AlliedVisionDriver::InitCamera(AlliedVisionCamera* cam, unsigned int width, unsigned int height, unsigned int binningX, unsigned int binningY)
{
    tPvErr errCode;

    // Open Camera
    if ((errCode = PvCameraOpen(cam->m_uid,ePvAccessMaster,&(cam->m_handle))) != ePvErrSuccess) {
        if (errCode == ePvErrAccessDenied) {
            std::cerr << "PvCameraOpen returned ePvErrAccessDenied:\nCamera already open as Master, or camera wasn't properly closed and still waiting to HeartbeatTimeout." << std::endl;
        }else {
            std::cerr << "PvCameraOpen err: " <<  errCode << std::endl;
        }
        return false;
    }

    // Write binning attribute
    if(binningX > 0 && binningY > 0 ) {
        if( PvAttrUint32Set(cam->m_handle,"BinningX", binningX) != ePvErrSuccess ||
            PvAttrUint32Set(cam->m_handle,"BinningY", binningY) != ePvErrSuccess ) {
            std::cerr << "Unable to set image binning in X and Y" << std::endl;
            return false;
        }
    }

    // Attempt to set user specified size
    if( width > 0 && height > 0 ) {
        if( PvAttrUint32Set(cam->m_handle,"Width", width) != ePvErrSuccess ||
            PvAttrUint32Set(cam->m_handle,"Height", height) != ePvErrSuccess ) {
            std::cerr << "Unable to set image width and height" << std::endl;
//            return false;
        }
    }

    // Read Set size
    if( PvAttrUint32Get(cam->m_handle,"Width",&cam->m_width) != ePvErrSuccess ||
        PvAttrUint32Get(cam->m_handle,"Height",&cam->m_height) != ePvErrSuccess ) {
        std::cerr << "Unable to read image width or height" << std::endl;
        return false;
    }

    // Calculate frame buffer size
    if((errCode = PvAttrUint32Get(cam->m_handle,"TotalBytesPerFrame",&cam->m_frameSizeBytes)) != ePvErrSuccess) {
        printf("CameraSetup: Get TotalBytesPerFrame err: %u\n", errCode);
        return false;
    }

    // Allocate the frame buffers
    for(int i=0;i<NUM_BUFFER_FRAMES;i++) {
        cam->m_frames[i].ImageBuffer = new char[cam->m_frameSizeBytes];

        if(cam->m_frames[i].ImageBuffer) {
            cam->m_frames[i].ImageBufferSize = cam->m_frameSizeBytes;
        } else {
            printf("CameraSetup: Failed to allocate buffers.\n");
            return false;
        }
    }

    // Increase packet size
    if((errCode = PvCaptureAdjustPacketSize(cam->m_handle,8228)) != ePvErrSuccess) {
        printf("CameraStart: PvCaptureAdjustPacketSize err: %u\n", errCode);
        return false;
    }

    std::cout << "Successfully initialised AlliedVision Camera with UUID: " << cam->m_uid << std::endl;

    return true;
}

///////////////////////////////////////////////////////////////////////////////
// Allocate resources, start camera
bool AlliedVisionDriver::StartCamera(AlliedVisionCamera* cam)
{
    tPvErr errCode;

    // Start driver capture stream
    if((errCode = PvCaptureStart(cam->m_handle)) != ePvErrSuccess) {
        printf("CameraStart: PvCaptureStart err: %u\n", errCode);
        return false;
    }

    // Add buffer frames to capture queue
    for(int i=0;i<NUM_BUFFER_FRAMES;i++) {
        if((errCode = PvCaptureQueueFrame(cam->m_handle,&(cam->m_frames[i]), NULL /*FrameDoneCB*/)) != ePvErrSuccess)
        {
            printf("CameraStart: PvCaptureQueueFrame err: %u\n", errCode);
            Deinit();
            return false;
        }
    }

    // Set the camera in freerun trigger, continuous mode, and start camera receiving triggers
    if((PvAttrEnumSet(cam->m_handle,"FrameStartTriggerMode","Freerun") != ePvErrSuccess) ||
        (PvAttrEnumSet(cam->m_handle,"AcquisitionMode","Continuous") != ePvErrSuccess) ||
        (PvCommandRun(cam->m_handle,"AcquisitionStart") != ePvErrSuccess))
    {
        printf("CameraStart: failed to set camera attributes\n");
        Deinit();
        return false;
    }

    return true;
}

///////////////////////////////////////////////////////////////////////////////
// Allocate resources for all cameras
bool AlliedVisionDriver::Init()
{
    if(PvInitialize() != ePvErrSuccess) {
        std::cerr << "Failed to initialize PvAPI" << std::endl;
    }

    Sleep(RETRY_WAIT_MS);

    m_numCams = m_pPropertyMap->GetProperty<tPvUint32>(PROPERTY_NUM_CAMS, 1);
    unsigned int userWidth = m_pPropertyMap->GetProperty<tPvUint32>(PROPERTY_WIDTH, 0);
    unsigned int userHeight = m_pPropertyMap->GetProperty<tPvUint32>(PROPERTY_HEIGHT, 0);
    unsigned int userBinX = m_pPropertyMap->GetProperty<tPvUint32>(PROPERTY_BINX, 0);
    unsigned int userBinY = m_pPropertyMap->GetProperty<tPvUint32>(PROPERTY_BINY, 0);

    for(unsigned int i=0; i <m_numCams; ++i ) {
        std::stringstream ss;
        ss << PROPERTY_CAM_UUID;
        ss << i;
        const std::string guid_property = ss.str();
        AlliedVisionCamera* cam = 0;
        unsigned long camUuid = m_pPropertyMap->GetProperty<tPvUint32>(guid_property, 0);
        cam = (camUuid == 0) ? GetFirstCamera() : GetCameraById(camUuid);
        if(cam == 0) {
            return false;
        }
        m_cam.push_back(cam);
    }

    bool success = true;
    for(unsigned int i=0; i <m_numCams; ++i ) {
        success &= InitCamera(m_cam[i], userWidth, userHeight, userBinX, userBinY);
    }

    for(unsigned int i=0; i <m_numCams; ++i ) {
        success &= StartCamera(m_cam[i]);
    }

    return success;
}

///////////////////////////////////////////////////////////////////////////////
// Stop Camera, free resources
void AlliedVisionDriver::Deinit()
{
    for(unsigned int i=0; i < m_numCams; ++i ) {
        DeinitCamera(m_cam[i]);
    }
}

///////////////////////////////////////////////////////////////////////////////
// Stop Camera, free resources
void AlliedVisionDriver::DeinitCamera(AlliedVisionCamera* cam)
{
    tPvErr errCode;

    // Stop camera receiving triggers
    if ((errCode = PvCommandRun(cam->m_handle,"AcquisitionStop")) != ePvErrSuccess) {
        printf("AcquisitionStop command err: %u\n", errCode);
    }

    // PvCaptureQueueClear aborts any actively written frame with Frame.Status = ePvErrDataMissing
    // Further queued frames returned with Frame.Status = ePvErrCancelled
    if ((errCode = PvCaptureQueueClear(cam->m_handle)) != ePvErrSuccess) {
        printf("PvCaptureQueueClear err: %u\n", errCode);
    }

    // Stop driver stream
    if ((errCode = PvCaptureEnd(cam->m_handle)) != ePvErrSuccess) {
        printf("PvCaptureEnd err: %u\n", errCode);
    }

    // Close Camera
    if((errCode = PvCameraClose(cam->m_handle)) != ePvErrSuccess) {
        printf("CameraUnSetup: PvCameraClose err: %u\n", errCode);
    }

    delete cam;

    PvUnInitialize();
}

///////////////////////////////////////////////////////////////////////////////
bool AlliedVisionDriver::Capture(AlliedVisionCamera* cam, rpg::ImageWrapper& img )
{
    // Wait for next image in sequence
    if( PvCaptureWaitForFrameDone(cam->m_handle, &(cam->m_frames[cam->m_nextFrame]), PVINFINITE) != ePvErrSuccess )
    {
        std::cerr << "Unable to grab frame " << std::endl;
        return false;
    }

    if( cam->m_frames[cam->m_nextFrame].Status == ePvErrSuccess ) {
        memcpy(img.Image.data, cam->m_frames[cam->m_nextFrame].ImageBuffer, cam->m_frameSizeBytes );
    }else{
        std::cerr << "Bad frame status" << std::endl;
    }

    // Enqueue frame
    PvCaptureQueueFrame(cam->m_handle, &(cam->m_frames[cam->m_nextFrame]), NULL);

    // Compute next frame in sequence
    cam->m_nextFrame = (cam->m_nextFrame+1) % NUM_BUFFER_FRAMES;
    return true;
}

///////////////////////////////////////////////////////////////////////////////
bool AlliedVisionDriver::Capture( std::vector<rpg::ImageWrapper>& vImages )
{
    // allocate images if necessary
    if( vImages.size() != m_numCams ){
        vImages.resize( m_numCams );
        for(unsigned i=0; i< m_numCams; ++i ) {
            vImages[i].Image = cv::Mat(m_cam[i]->m_height, m_cam[i]->m_width, CV_8UC1);
        }
    }
    // should also check images are the right size, type etc

    bool success = true;
    for(unsigned i=0; i< m_numCams; ++i ) {
        success &= Capture(m_cam[i], vImages[i]);
    }

    return success;
}
