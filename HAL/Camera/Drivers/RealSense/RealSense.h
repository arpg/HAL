#pragma once

#include <memory>
#include <HAL/Camera/CameraDriverInterface.h>

#include "libuvc_ex/include/libuvc_ex/libuvc.h"

namespace hal
{

class RealSenseDriver : public CameraDriverInterface
{
public:
    RealSenseDriver(bool useIR, bool useSync);
    ~RealSenseDriver();
    
    bool Capture( pb::CameraMsg& vImages );
    std::shared_ptr<CameraDriverInterface> GetInputDevice() { return std::shared_ptr<CameraDriverInterface>(); }
    bool SetExposure(int nExposure);
    
    void Start(int vid, int pid, char* sn);
    void Stop();
    
    size_t NumChannels() const;
    size_t Width( size_t /*idx*/ = 0 ) const;
    size_t Height( size_t /*idx*/ = 0 ) const;
    uvc_error_t getFrame(uvc_stream_handle_t *streamh, uvc_frame_t **frame);
    const int64_t frameSyncLimitPos = 40000000000;
    const int64_t frameSyncLimitNeg = -20000000000;
    const int64_t wrapLimit = 8800000000000;
    int64_t diffSCR(uint64_t depth, uint64_t rgb);
    static void ImageCallbackAdapter(uvc_frame_t *frame, void *ptr);
    void ImageCallback(uvc_frame_t *frame);
    
    uvc_context* ctx_;
    uvc_device*  dev_;
    uvc_device_handle*  devh_rgb; //one for each camera
    uvc_device_handle*  devh_d;  
    uvc_stream_ctrl_t ctrl_rgb;
    uvc_stream_ctrl_t ctrl_d;
    uvc_stream_handle_t *streamh_rgb;
    uvc_stream_handle_t *streamh_d;

    uvc_frame_t* frame_rgb;
    uvc_frame_t* frame_d;
 
    
    int pbtype_rgb;
    int pbtype_d;
 
    int pbformat_rgb;
    int pbformat_d;

    int width_;
    int height_;
    int fps_;

    bool useIR;
    bool useSync;

};

}
