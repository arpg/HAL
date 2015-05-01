#include "UvcDriver.h"

#include <HAL/Devices/DeviceException.h>

#include <iostream>

namespace hal {

UvcDriver::UvcDriver()
    : ctx_(NULL),
      dev_(NULL),
      devh_(NULL),
      frame_(NULL)
{
//    Start(0,0,NULL);
//    Start(0x0c45,0x62f1,NULL); // Sonix
  //Start(0x03e7,0x1811,NULL); // Twizzler
    Start(0x8086, 0x0a66, NULL);
}

UvcDriver::~UvcDriver()
{
    Stop();
}

size_t UvcDriver::NumChannels() const
{
    return 1;
}

size_t UvcDriver::Width( size_t /*idx*/) const
{
    return width_;
}

size_t UvcDriver::Height( size_t /*idx*/) const
{
    return height_;
}

bool UvcDriver::SetExposure(int nExposure)
{
    uvc_error_t err = uvc_set_exposure_abs(devh_,nExposure);
    if (err != UVC_SUCCESS) {
        return false;
    }else{
        return true;
    }
}


void UvcDriver::Start(int vid, int pid, char* sn)
{
    width_ = 640;
    height_ = 480;
    fps_ = 30;
    
    if(ctx_) {
        Stop();
    }
    
    uvc_init(&ctx_, NULL);
    if(!ctx_) {
        throw DeviceException("Unable to open UVC Context");
    }
    
    uvc_error_t find_err = uvc_find_device(ctx_, &dev_, vid, pid, sn );
    if (find_err != UVC_SUCCESS) {
        uvc_perror(find_err, "uvc_find_device");
        throw DeviceException("Unable to open UVC Device");
    }
    if(!dev_) {
        throw DeviceException("Unable to open UVC Device - no pointer returned.");
    }
    
    uvc_error_t open_err = uvc_open(dev_, &devh_);
    if (open_err != UVC_SUCCESS) {
        uvc_perror(open_err, "uvc_open");
        uvc_unref_device(dev_);
        throw DeviceException("Unable to open device");
    }



    
//    uvc_set_status_callback(devh_, &CameraDriver::AutoControlsCallbackAdapter, this);
    
    uvc_stream_ctrl_t ctrl;
    uvc_error_t mode_err = uvc_get_stream_ctrl_format_size(
                devh_, &ctrl,
                UVC_COLOR_FORMAT_YUYV,
                width_, height_,
                fps_);
    
    pbtype = hal::PB_UNSIGNED_BYTE;
    pbformat = hal::PB_LUMINANCE;
        
    if (mode_err != UVC_SUCCESS) {
        uvc_perror(mode_err, "uvc_get_stream_ctrl_format_size");
        uvc_close(devh_);
        uvc_unref_device(dev_);
        throw DeviceException("Unable to device mode.");
    }
    
//    uvc_error_t stream_err = uvc_start_iso_streaming(devh_, &ctrl, &UvcDriver::ImageCallbackAdapter, this);
    //uvc_error_t stream_err = uvc_start_iso_streaming(devh_, &ctrl, NULL, this);
    uvc_error_t stream_err = uvc_start_streaming(devh_, &ctrl, NULL, this, 0);
    
    if (stream_err != UVC_SUCCESS) {
        uvc_perror(stream_err, "uvc_start_iso_streaming");
        uvc_close(devh_);
        uvc_unref_device(dev_);
        throw DeviceException("Unable to start iso streaming.");
    }
    
    if (frame_)
        uvc_free_frame(frame_);
    
    frame_ = uvc_allocate_frame(ctrl.dwMaxVideoFrameSize);
    if(!frame_) {
        throw DeviceException("Unable to allocate frame.");
    }
}

void UvcDriver::Stop()
{
    if(devh_) {
        uvc_stop_streaming(devh_);
        devh_ = 0;
        dev_ = 0;
    }
    
    if (frame_) {
        uvc_free_frame(frame_);
        frame_ = 0;
    }    

//    if (ctx_) {
//        // Work out how to kill this properly
//        uvc_exit(ctx_);
//        ctx_ = 0;
//    }   
}

//void UvcDriver::ImageCallbackAdapter(uvc_frame_t *frame, void *ptr) {
//  UvcDriver *driver = static_cast<UvcDriver*>(ptr);
//  driver->ImageCallback(frame);
//}

//void UvcDriver::ImageCallback(uvc_frame_t* /*frame*/) {
//}

bool UvcDriver::Capture( hal::CameraMsg& vImages )
{
    vImages.Clear();

    uvc_frame_t* frame = NULL;
    uvc_error_t err = uvc_get_frame(devh_, &frame, 0);
    if(err!= UVC_SUCCESS) {
        uvc_perror(err, "uvc_get_frame");
    }else{
        if(frame) {
            hal::ImageMsg* pimg = vImages.add_image();
            pimg->set_type( (hal::Type) pbtype );
            pimg->set_format( (hal::Format) pbformat );            
            pimg->set_width(frame->width);
            pimg->set_height(frame->height);
            pimg->set_data(frame->data, width_ * height_);
            return true;
        }else{
            std::cout << "No data..." << std::endl;
        }
        
    }
                     
    return true;
}

}
