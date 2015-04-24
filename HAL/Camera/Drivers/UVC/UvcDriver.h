#pragma once

#include <memory>
#include <HAL/Camera/CameraDriverInterface.h>

#include <libuvc/libuvc.h>

namespace hal
{

class UvcDriver : public CameraDriverInterface
{
public:
    UvcDriver();
    ~UvcDriver();
    
    bool Capture( hal::CameraMsg& vImages );
    std::shared_ptr<CameraDriverInterface> GetInputDevice() { return std::shared_ptr<CameraDriverInterface>(); }
    bool SetExposure(int nExposure);
    
    void Start(int vid, int pid, char* sn);
    void Stop();
    
    size_t NumChannels() const;
    size_t Width( size_t /*idx*/ = 0 ) const;
    size_t Height( size_t /*idx*/ = 0 ) const;
    
    static void ImageCallbackAdapter(uvc_frame_t *frame, void *ptr);
    void ImageCallback(uvc_frame_t *frame);
    
    uvc_context* ctx_;
    uvc_device*  dev_;
    uvc_device_handle*  devh_;
    uvc_frame_t* frame_;
    
    int pbtype;
    int pbformat;
    int width_;
    int height_;
    int fps_;


};

}
