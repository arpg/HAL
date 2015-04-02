#include "RealSense.h"

#include <HAL/Devices/DeviceException.h>

#include <iostream>

#include <unistd.h>

namespace hal {

  RealSenseDriver::RealSenseDriver(bool m_useIR, bool m_useSync)
    : ctx_(NULL),
      dev_(NULL),
      devh_rgb(NULL),
      devh_d(NULL),
      frame_rgb(NULL),
      frame_d(NULL),
      useIR(m_useIR),
      useSync(m_useSync)
{

  Start(0x8086, 0x0a66, NULL); //hardcoded for the RealSense camera's vid/pid
}

RealSenseDriver::~RealSenseDriver()
{
    Stop();
}

size_t RealSenseDriver::NumChannels() const
{
  return 2; //one channel for RGB, one for depth
}

size_t RealSenseDriver::Width( size_t /*idx*/) const
{
    return width_;
}

size_t RealSenseDriver::Height( size_t /*idx*/) const
{
    return height_;
}

bool RealSenseDriver::SetExposure(int nExposure)
{
    uvc_error_t err = uvc_set_exposure_abs(devh_rgb,nExposure);
    if (err != UVC_SUCCESS) {
        return false;
    }else{
        return true;
    }
}


void RealSenseDriver::Start(int vid, int pid, char* sn)
{
  //The RealSense camera is capable of delivering separate streams for rgb and depth at separate resolutions
  //For now, use 640x480@30fps for each one
  //There are separate streams defined for depth using non-standard UVC GUIDs - use the YUYV-advertised GUID, which is really just 
  //gray16 data


    width_ = 640;
    height_ = 480;
    fps_ = 30;
    
    if(ctx_) {
        Stop();
    }
    
    uvc_init(&ctx_, NULL);
    if(!ctx_) {
        throw DeviceException("Unable to open UVC library context");
    }

    uvc_error_t find_err;
    if (sn)
      find_err = uvc_find_device(ctx_, &dev_, vid, pid, sn );
    else
      find_err = uvc_find_device(ctx_, &dev_, vid, pid, NULL);

    if (find_err != UVC_SUCCESS) {
        uvc_perror(find_err, "uvc_find_device");
        throw DeviceException("Unable to open RealSense device");
    }
    if(!dev_) {
        throw DeviceException("Unable to open RealSense Device - no pointer returned.");
    }
    
    uvc_error_t open_err = uvc_open2(dev_, &devh_rgb, 0); //Camera 0 is RGB
    if (open_err != UVC_SUCCESS) {
        uvc_perror(open_err, "uvc_open");
        uvc_unref_device(dev_);
        throw DeviceException("Unable to open RealSense RGB device");
    }

    open_err = uvc_open2(dev_, &devh_d, 1); //Camera 1 is depth
    if (open_err != UVC_SUCCESS) {
        uvc_perror(open_err, "uvc_open");
        uvc_unref_device(dev_);
        throw DeviceException("Unable to open RealSense Depth device");
    }
    
  
//    uvc_set_status_callback(devh_, &CameraDriver::AutoControlsCallbackAdapter, this);
    

    
    uvc_error_t mode_err = uvc_get_stream_ctrl_format_size(
                devh_rgb, &ctrl_rgb,
                UVC_COLOR_FORMAT_YUYV,
                width_, height_,
                60);
    
    pbtype_rgb = pb::PB_UNSIGNED_BYTE;
    pbformat_rgb = pb::PB_RGB;
        
    if (mode_err != UVC_SUCCESS) {
        uvc_perror(mode_err, "uvc_get_stream_ctrl_format_size");
        uvc_close(devh_rgb);
	uvc_close(devh_d);
        uvc_unref_device(dev_);
        throw DeviceException("Unable to get RGB device mode.");
    }

    //Use the same frame parameters for both RGB and depth (for now)
    if (useIR)
      {
	//Depth as mono8
	mode_err = uvc_get_stream_ctrl_format_size(
						   devh_d, &ctrl_d,
						   UVC_FRAME_FORMAT_INVI,
						   width_, height_,
						   fps_);
	
	pbtype_d = pb::PB_UNSIGNED_BYTE;
      }
    else
      {
	//Grayscale depth as yuyv, mono16
	mode_err = uvc_get_stream_ctrl_format_size(
						   devh_d, &ctrl_d,
						   //UVC_FRAME_FORMAT_INVI,
						   UVC_FRAME_FORMAT_YUYV,
						   width_, height_,
						   fps_);
    
	pbtype_d = pb::PB_UNSIGNED_SHORT;
      }
    
    pbformat_d = pb::PB_LUMINANCE;
        
    if (mode_err != UVC_SUCCESS) {
        uvc_perror(mode_err, "uvc_get_stream_ctrl_format_size");
        uvc_close(devh_rgb);
	uvc_close(devh_d);
        uvc_unref_device(dev_);
        throw DeviceException("Unable to get depth device mode.");
    }

    uvc_error_t stream_ctl_err = uvc_stream_open_ctrl(devh_rgb, &streamh_rgb, &ctrl_rgb);
   
    if (stream_ctl_err != UVC_SUCCESS) {
        uvc_perror(stream_ctl_err, "uvc_stream_open_ctrl");
        uvc_close(devh_rgb);
	uvc_close(devh_d);
        uvc_unref_device(dev_);
        throw DeviceException("Unable to open stream on RGB.");
    }

    stream_ctl_err = uvc_stream_open_ctrl(devh_d, &streamh_d, &ctrl_d);
   
    if (stream_ctl_err != UVC_SUCCESS) {
        uvc_perror(stream_ctl_err, "uvc_stream_open_ctrl");
        uvc_close(devh_rgb);
	uvc_close(devh_d);
        uvc_unref_device(dev_);
        throw DeviceException("Unable to open stream on depth.");
    }

    uvc_error_t stream_start_err = uvc_stream_start(streamh_rgb, NULL, this, 0);
    
    if (stream_start_err != UVC_SUCCESS) {
        uvc_perror(stream_start_err, "uvc_stream_start");
	uvc_close(devh_rgb);
	uvc_close(devh_d);
        uvc_unref_device(dev_);
        throw DeviceException("Unable to start streaming on rgb.");
    }

    stream_start_err = uvc_stream_start(streamh_d, NULL, this, 0);

    if (stream_start_err != UVC_SUCCESS) {
        uvc_perror(stream_start_err, "uvc_stream_start");
	uvc_close(devh_rgb);
	uvc_close(devh_d);
        uvc_unref_device(dev_);
        throw DeviceException("Unable to start streaming on depth.");
    }
    
    if (frame_rgb)
        uvc_free_frame(frame_rgb);
     if (frame_d)
        uvc_free_frame(frame_d);
    
    
    frame_rgb = uvc_allocate_frame(ctrl_rgb.dwMaxVideoFrameSize);
    if(!frame_rgb) {
        throw DeviceException("Unable to allocate RGB frame.");
    }

    frame_d = uvc_allocate_frame(ctrl_d.dwMaxVideoFrameSize);
    if(!frame_d) {
        throw DeviceException("Unable to allocate depth frame.");
    }

    if (useSync)
      {
	std::cout << "RealSense: Using SCR to sync" << std::endl;
      }
}

void RealSenseDriver::Stop()
{
  std::cout << "Stopping RealSense driver" << std::endl;

  /*
    if(streamh_rgb) {
        uvc_stream_close(streamh_rgb);
    }

    if(streamh_d) {
        uvc_stream_close(streamh_d);
    }
  */
    std::cout << "Closed streams" << std::endl;

    if (frame_rgb) {
        uvc_free_frame(frame_rgb);
        frame_rgb = 0;
    }    

    if (frame_d) {
        uvc_free_frame(frame_d);
        frame_d = 0;
    }
  
    std::cout << "Released frames" << std::endl;

    /* Release the device descriptor, shutdown UVC */
    uvc_close(devh_rgb);
    uvc_close(devh_d);

    std::cout << "Closed devices" << std::endl;

    
    uvc_unref_device(dev_);
    std::cout << "Unreffed device" << std::endl;


    uvc_exit(ctx_);
    dev_ = 0;

    std::cout << "Stop of RealSense driver complete" << std::endl;
 
}

  
  uvc_error_t RealSenseDriver::getFrame(uvc_stream_handle_t *streamh, uvc_frame_t **frame)
  {
    uvc_error_t err;
    err = uvc_stream_get_frame(streamh, frame, -1); //use the latest acquired frame
    if(err != UVC_SUCCESS)
      {
	uvc_perror(err, "uvc_get_frame");
      }
    return err;
  }

int64_t RealSenseDriver::diffSCR(uint64_t depth, uint64_t rgb)
{
  //return the difference as depth - rgb, accounting for rollovers of each one as necessary
  int64_t newDepth, newRGB;
  newDepth = depth;
  newRGB = rgb;

  if (wrapLimit - depth + rgb < frameSyncLimitPos)
    return (wrapLimit - depth + rgb);
  
  if (depth + frameSyncLimitPos > wrapLimit)
    {
      //Rollover the depth into negative territory
      newDepth = depth - wrapLimit;
    }

  if (rgb + frameSyncLimitPos > wrapLimit)
    {
      //Rollover the RGB into negative territory
      newRGB = rgb - wrapLimit;
    }

  return (newDepth - newRGB);
  
}

bool RealSenseDriver::Capture( pb::CameraMsg& vImages )
{
  /*use a two-vector, where the first image is RGB, second is gray16 for depth or gray8 for IR */
    vImages.Clear();

    uvc_frame_t* frameRGB = NULL;
    uvc_frame_t* frameDepth = NULL;
    uvc_frame_t *rgb; //to hold the YUV->RGB conversion
    uvc_error_t err;
    pb::ImageMsg* pimg;
    uint64_t scrDepth, scrRGB;

    scrDepth = 0;
    scrRGB = 0;
    
    /* Try to sync up the streams
       Based on the SCR value that is shared between the two cameras, it looks like 
       RGB leads (comes before) IR by about 21475451171 units (roughly) between frames
    
       Strategy: Make sure the difference between SCR values for each frame is less than
       frameSyncLimit units.

       Also, depth takes some time to actually start publishing - keep reading RGB until we get
       a good pair

    */

    //Two functions: advanceDepth, advanceRGB
    //Pick up an RGB, wait for a depth, see if diff > synclimit
    //yes? Repeat with new RGB
    //no? Stash both into protobuf and continue

    
    /*Pick up the depth image */
    int gotPair = 0;
    int needRGB = 1;
    int needDepth = 1;
    int advRGB = 0;
    
    int64_t frameSync;
    
    while (!gotPair)
      {
	while (needRGB)
	  {
	    err = getFrame(streamh_rgb, &frameRGB);
	   
	    if ((err == UVC_SUCCESS) && (frameRGB != 0))
	      {
		//Calc the sync
		memcpy(&scrRGB, &frameRGB->capture_time, sizeof(scrRGB));
		needRGB = 0;
	      }
	    usleep(1);
	  }

	//Pick up a Depth
	while (needDepth)
	  {
	    err = getFrame(streamh_d, &frameDepth);
	    if ((err == UVC_SUCCESS) && (frameDepth != 0))
	      {
		//Calc the sync
		memcpy(&scrDepth, &frameDepth->capture_time, sizeof(scrDepth));
		needDepth = 0;
	      }
	    usleep(1);
	  }
	
	frameSync = diffSCR(scrDepth, scrRGB);


	if (!useSync)
	  {
	    //Don't care about the sync, just return the images
	    
	    gotPair = 1;
	    break;
	  }
	
	std::cout << "Sync?: R:" << scrRGB << " D: " << scrDepth << " diffSCR: " << frameSync;
		
	if (frameSync > frameSyncLimitPos)
	  {
	    //Depth is ahead of RGB, advance the RGB stream
	    advRGB++;
	    if (advRGB > 3)
	      {
		//restart, the rgb is out of whack
		needDepth = 1;
		std::cout << " Restarting acq sequence" << std::endl;
	      }
	    else
	      {
		std::cout << " Advancing RGB stream" << std::endl;
	      }
	    needRGB = 1;
	    continue;
	  }
	else if (frameSync < frameSyncLimitNeg)
	  {
	    // depth after RGB by too much, start over
	    needRGB = 1;
	    needDepth = 1;
	    std::cout << " Bad sync, starting over" << std::endl;
	    advRGB = 0;
	    continue;
	  }
	else
	  {
	    //Within limits, exit the loop
	    std::cout << " Good sync" << std::endl;
	    gotPair = 1;
	  }
      }
     
    if(frameRGB)
      {
	pimg = vImages.add_image();
	pimg->set_type( (pb::Type) pbtype_rgb );
	pimg->set_format( (pb::Format) pbformat_rgb );            
	pimg->set_width(frameRGB->width);
	pimg->set_height(frameRGB->height);
	rgb = uvc_allocate_frame(frameRGB->width * frameRGB->height * 3);
	uvc_any2rgb(frameRGB, rgb);
	//Convert the YUYV to RGB
	pimg->set_data(rgb->data, width_ * height_ * 3); //RGB uint8_t
	    
	uvc_free_frame(rgb);
      }
    
    if(frameDepth)
      {
	pimg = vImages.add_image();
	pimg->set_type( (pb::Type) pbtype_d );
	pimg->set_format( (pb::Format) pbformat_d );            
	pimg->set_width(frameDepth->width);
	pimg->set_height(frameDepth->height);
	    

	if (useIR)
	  {
	    pimg->set_data(frameDepth->data, width_ * height_); //gray uint8_t
	  }
	else
	  {
	    pimg->set_data(frameDepth->data, width_ * height_ * 2); //gray uint16_t
	  }
      }

    
    return true;
}

} //namespace
