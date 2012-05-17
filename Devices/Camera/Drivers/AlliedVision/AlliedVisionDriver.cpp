/*
   \file AlliedVisionDriver.cpp
 */

#include <inttypes.h>
#include <stdio.h>
#include <stdint.h>
#include "AlliedVisionDriver.h"
#include <mvl/image/image.h> // to rectify
#include <mvl/stereo/stereo.h>

const int NUM_BUFFER_FRAMES = 15;
const int MAX_RETRIES = 10;
const int RETRY_WAIT_MS = 200;
const std::string PROPERTY_WIDTH  = "ImageWidth";
const std::string PROPERTY_HEIGHT = "ImageHeight";

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

AlliedVisionCamera* GetCamera()
{
    tPvUint32 connected;
    tPvCameraInfoEx list;

    tPvUint32 numCameras = 0;

    for(unsigned retries = MAX_RETRIES; retries > 0; retries-- ) {
        Sleep(RETRY_WAIT_MS);
        numCameras = PvCameraListEx(&list,1,&connected, sizeof(tPvCameraInfoEx));
        if(numCameras > 0 && connected ) break;
    }

    if(numCameras == 1) {
        return new AlliedVisionCamera(list.UniqueId);
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
// Allocate resources, start camera
bool AlliedVisionDriver::Init()
{
//    assert(m_pPropertyMap);
//    m_pPropertyMap->PrintPropertyMap();

    if(PvInitialize() != ePvErrSuccess) {
        std::cerr << "Failed to initialize PvAPI" << std::endl;
    }

    m_cam = GetCamera();
    if( !m_cam ) {
        std::cerr << "No cameras available" << std::endl;
        return false;
    }

    tPvErr errCode;

    // Open Camera
    if ((errCode = PvCameraOpen(m_cam->m_uid,ePvAccessMaster,&(m_cam->m_handle))) != ePvErrSuccess) {
        if (errCode == ePvErrAccessDenied) {
            std::cerr << "PvCameraOpen returned ePvErrAccessDenied:\nCamera already open as Master, or camera wasn't properly closed and still waiting to HeartbeatTimeout." << std::endl;
        }else {
            std::cerr << "PvCameraOpen err: " <<  errCode << std::endl;
        }
        return false;
    }

    // Read default size
    if( PvAttrUint32Get(m_cam->m_handle,"Width",&m_cam->m_width) != ePvErrSuccess ||
        PvAttrUint32Get(m_cam->m_handle,"Height",&m_cam->m_height) != ePvErrSuccess ) {
        std::cerr << "Unable to read image width or height" << std::endl;
        return false;
    }

    // Override size with user settings if supplied
    m_cam->m_width  = m_pPropertyMap->GetProperty<tPvUint32>(PROPERTY_WIDTH, m_cam->m_width);
    m_cam->m_height = m_pPropertyMap->GetProperty<tPvUint32>(PROPERTY_HEIGHT, m_cam->m_height);

    // Write desired image size
    if( PvAttrUint32Set(m_cam->m_handle,"Width", m_cam->m_width) != ePvErrSuccess ||
        PvAttrUint32Set(m_cam->m_handle,"Height", m_cam->m_height) != ePvErrSuccess ) {
        std::cerr << "Unable to set image width and height" << std::endl;
        return false;
    }

    // Calculate frame buffer size
    if((errCode = PvAttrUint32Get(m_cam->m_handle,"TotalBytesPerFrame",&m_cam->m_frameSizeBytes)) != ePvErrSuccess) {
        printf("CameraSetup: Get TotalBytesPerFrame err: %u\n", errCode);
        return false;
    }

    // Allocate the frame buffers
    for(int i=0;i<NUM_BUFFER_FRAMES;i++) {
        m_cam->m_frames[i].ImageBuffer = new char[m_cam->m_frameSizeBytes];

        if(m_cam->m_frames[i].ImageBuffer) {
            m_cam->m_frames[i].ImageBufferSize = m_cam->m_frameSizeBytes;
        } else {
            printf("CameraSetup: Failed to allocate buffers.\n");
            return false;
        }
    }

    // Increase packet size
    if((errCode = PvCaptureAdjustPacketSize(m_cam->m_handle,8228)) != ePvErrSuccess) {
        printf("CameraStart: PvCaptureAdjustPacketSize err: %u\n", errCode);
        return false;
    }

    // Start driver capture stream
    if((errCode = PvCaptureStart(m_cam->m_handle)) != ePvErrSuccess) {
        printf("CameraStart: PvCaptureStart err: %u\n", errCode);
        return false;
    }

    // Add buffer frames to capture queue
    for(int i=0;i<NUM_BUFFER_FRAMES;i++) {
        if((errCode = PvCaptureQueueFrame(m_cam->m_handle,&(m_cam->m_frames[i]), NULL /*FrameDoneCB*/)) != ePvErrSuccess)
        {
            printf("CameraStart: PvCaptureQueueFrame err: %u\n", errCode);
            Deinit();
            return false;
        }
    }

    // Set the camera in freerun trigger, continuous mode, and start camera receiving triggers
    if((PvAttrEnumSet(m_cam->m_handle,"FrameStartTriggerMode","Freerun") != ePvErrSuccess) ||
        (PvAttrEnumSet(m_cam->m_handle,"AcquisitionMode","Continuous") != ePvErrSuccess) ||
        (PvCommandRun(m_cam->m_handle,"AcquisitionStart") != ePvErrSuccess))
    {
        printf("CameraStart: failed to set camera attributes\n");
        Deinit();
        return false;
    }

    return true;
}

///////////////////////////////////////////////////////////////////////////////
// Stop Camera, free resources
void AlliedVisionDriver::Deinit()
{
    tPvErr errCode;

    // Stop camera receiving triggers
    if ((errCode = PvCommandRun(m_cam->m_handle,"AcquisitionStop")) != ePvErrSuccess) {
        printf("AcquisitionStop command err: %u\n", errCode);
    }

    // PvCaptureQueueClear aborts any actively written frame with Frame.Status = ePvErrDataMissing
    // Further queued frames returned with Frame.Status = ePvErrCancelled
    if ((errCode = PvCaptureQueueClear(m_cam->m_handle)) != ePvErrSuccess) {
        printf("PvCaptureQueueClear err: %u\n", errCode);
    }

    // Stop driver stream
    if ((errCode = PvCaptureEnd(m_cam->m_handle)) != ePvErrSuccess) {
        printf("PvCaptureEnd err: %u\n", errCode);
    }

    // Close Camera
    if((errCode = PvCameraClose(m_cam->m_handle)) != ePvErrSuccess) {
        printf("CameraUnSetup: PvCameraClose err: %u\n", errCode);
    }

    delete m_cam;

    PvUnInitialize();
}

///////////////////////////////////////////////////////////////////////////////
bool AlliedVisionDriver::Capture( std::vector<rpg::ImageWrapper>& vImages )
{
    // allocate images if necessary
    if( vImages.size() != 1 ){
        vImages.resize( 1 );
        // and setup images
        vImages[0].Image = cv::Mat(m_cam->m_height, m_cam->m_width, CV_8UC1);
//        vImages[1].Image = cv::Mat(m_cam->m_height, m_cam->m_width, CV_8UC1);
    }
    // should also check images are the right size, type etc

    // Wait for next image in sequence
    PvCaptureWaitForFrameDone(m_cam->m_handle, &(m_cam->m_frames[m_cam->m_nextFrame]), PVINFINITE);

    if( m_cam->m_frames[m_cam->m_nextFrame].Status == ePvErrSuccess ) {
        // Copy image
        memcpy(vImages[0].Image.data, m_cam->m_frames[m_cam->m_nextFrame].ImageBuffer, m_cam->m_frameSizeBytes );
//        memcpy(vImages[1].Image.data, m_cam->m_frames[m_cam->m_nextFrame].ImageBuffer, m_cam->m_frameSizeBytes );
    }

    // Enqueue frame
    PvCaptureQueueFrame(m_cam->m_handle, &(m_cam->m_frames[m_cam->m_nextFrame]), NULL);

    // Compute next frame in sequence
    m_cam->m_nextFrame = (m_cam->m_nextFrame+1) % NUM_BUFFER_FRAMES;

    return true;
}
