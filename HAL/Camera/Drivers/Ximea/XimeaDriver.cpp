#include <iostream>

#include <HAL/Devices/DeviceException.h>
#include <HAL/Utils/TicToc.h>

#include "XimeaDriver.h"
//Limit the bandwidth available to each camera out of this bucket of data
//Assumes that all cameras share the same host controller
//From the Ximea support page, https://www.ximea.com/support/wiki/usb3/Multiple_Cameras_Setup

#define USB3_HUB_DATA_RATE 2400

using namespace hal;

///////////////////////////////////////////////////////////////////////////
inline void XimeaDriver::_CheckError(
    XI_RETURN     err,
    std::string   place
    )
{
  if (err != XI_OK) {
    std::cerr << "Error after '" << place << "' " << err << std::endl;
    throw DeviceException("Ximea SDK exception!");
  }
}

///////////////////////////////////////////////////////////////////////////
XimeaDriver::XimeaDriver(
       std::vector<std::string>&  vector_ids,
    float                       fps,
    int                         exp,
    float                       gain,
    XI_IMG_FORMAT               mode,
    ImageRoi                    roi,
    int                         sync,
    int                         binning,
    int                         bus_cams
    )
{
 XI_RETURN error = XI_OK;

  // Sync flag.
  sync_ = sync;

  // Initialize image size.
  image_format_ = mode;
  image_width_  = roi.w;
  image_height_ = roi.h;
  binning_ = binning;
  bus_cams_ = bus_cams;
  std::cout << "Bus cameras @ init:" << bus_cams_ << std::endl;

  // Get number of camera devices.
  DWORD num_devices;
  uint16_t camera_data_rate;

  error = xiGetNumberDevices(&num_devices);
  _CheckError(error, "xiGetNumberDevices");

  if (num_devices == 0) {
    throw DeviceException("No cameras found!");
  }

  // If no ids are provided, all cameras will be opened.
  if (vector_ids.empty()) {
    num_channels_ = num_devices;
  } else {
    num_channels_ = vector_ids.size();
  }

  if (num_channels_ < vector_ids.size()) {

    throw DeviceException("Less cameras detected than those requested!");
  }

  // Resize camera handle vector to accomodate requested cameras.
  cam_handles_.resize(num_channels_);

  // disable auto bandwidth calculation (before camera open)
  xiSetParamInt(0, XI_PRM_AUTO_BANDWIDTH_CALCULATION, XI_OFF);
  std::cerr << "Ximea auto-bandwidth calc disabled" << std::endl;

  for (size_t ii = 0; ii < num_channels_; ++ii) {
    // Retrieving a handle to the camera device.
    // Actually open the correct one if serial numbers are specified
    if (vector_ids.size() > 0)
      {
  error = xiOpenDeviceBy(XI_OPEN_BY_SN, vector_ids[ii].c_str(), &cam_handles_[ii]);
      }
    else
      {
  std::cerr << "Opening Ximea index: " << ii << std::endl;
  error = xiOpenDevice(ii, &cam_handles_[ii]);
      }
    _CheckError(error, "xiOpenDevice");
  }
  //Calculate the data rate available per camera

  if (bus_cams_ == 0)
    {
      //if not set explicitly, assume that whatever is detected / requested is on the same bus
      bus_cams_ = num_channels_;
    }


  camera_data_rate =  USB3_HUB_DATA_RATE / bus_cams_;

  // Set parameters.
  bool first_cam = true; // Flag to check if we are dealing with first camera.
  for (HANDLE handle: cam_handles_) {
    //Set data rate limit
    int curr_camera_data_rate;
    error = xiGetParamInt(handle, XI_PRM_LIMIT_BANDWIDTH, &curr_camera_data_rate);
    std::cout << "Current data rate:" << curr_camera_data_rate << std::endl;
    _CheckError(error, "xiGetParamInt (limit_bandwidth)");

    std::cout << "Desired data rate:" << camera_data_rate << std::endl;
    if (camera_data_rate < curr_camera_data_rate) {
      error = xiSetParamInt(handle, XI_PRM_LIMIT_BANDWIDTH , camera_data_rate);
      _CheckError(error, "xiSetParam (limit_bandwidth)");
      std::cerr << "Set bandwidth limit on camera" << std::endl;
    }

    // Setting "format" parameter. Has to be set first before image size!
    error = xiSetParamInt(handle, XI_PRM_IMAGE_DATA_FORMAT, image_format_);
    _CheckError(error, "xiSetParam (format)");

    int max_width, max_height;
    xiGetParamInt(handle, XI_PRM_WIDTH XI_PRM_INFO_MAX, &max_width);
    xiGetParamInt(handle, XI_PRM_HEIGHT XI_PRM_INFO_MAX, &max_height);

    // Setting "width" parameter.
    if (image_width_ <= max_width - roi.x) {
      error = xiSetParamInt(handle, XI_PRM_WIDTH, image_width_);
      _CheckError(error, "xiSetParam (width)");

      int left = roi.x;
      if (left == 0) {
        left = (max_width / 2) - image_width_ / 2;
      }

      error = xiSetParamInt(handle, XI_PRM_OFFSET_X, left);
      _CheckError(error, "xiSetParam (x-offset)");
    } else {
      std::cerr << "Image width out of bounds. Maximum value: " <<
                   max_width << std::endl;
      throw DeviceException("Image out of bounds!");
    }

    // Setting "height" parameter.
    if (image_height_ <= max_height - roi.y) {
      error = xiSetParamInt(handle, XI_PRM_HEIGHT, image_height_);
      _CheckError(error, "xiSetParam (height)");

      int top = roi.y;
      if (top == 0) {
        top = (max_height / 2) - image_height_ / 2;
      }

      error = xiSetParamInt(handle, XI_PRM_OFFSET_Y, top);
      _CheckError(error, "xiSetParam (y-offset)");
    } else {
      std::cerr << "Image height out of bounds. Maximum value: " <<
                   max_height << std::endl;
      throw DeviceException("Image out of bounds!");
    }

    //Set binning if applicable
    if (binning_ > 0) {
      error = xiSetParamInt(handle, XI_PRM_DOWNSAMPLING, binning_);
      _CheckError(error, "xiSetParam (binning)");
    }

    // Setting "exposure" parameter (10ms=10000us). Default 0 is AUTO exposure.
    if (exp == 0) {
      error = xiSetParamInt(handle, XI_PRM_AEAG, 1);
      _CheckError(error, "xiSetParam (AUTO exposure)");
    } else {
      int min_exp, max_exp;
      xiGetParamInt(handle, XI_PRM_EXPOSURE XI_PRM_INFO_MIN, &min_exp);
      xiGetParamInt(handle, XI_PRM_EXPOSURE XI_PRM_INFO_MAX, &max_exp);
      if (exp < min_exp || exp > max_exp) {
        std::cerr << "Exposure Values = Min: " << min_exp << " - Max: "
                  << max_exp << std::endl;
        throw DeviceException("Requested EXPOSURE not supported!");
      }
      error = xiSetParamInt(handle, XI_PRM_EXPOSURE, exp);
      _CheckError(error, "xiSetParam (exposure)");
    }

    // Setting "gain" parameter. If AUTO exposure, gain is calculated based
    // on exposure priority in algoirthm -- e.g. (0.5 - exposure 50%, gain 50%)
    if (exp == 0) {
      if (gain < 0.0 || gain > 1.0) {
        std::cerr << "Exposure is set to AUTO, so gain value must be [0.0-1.0]. "
                  << "This controls exposure priority: 0.5 = exposure 50%, gain 50%. "
                  << "Setting default value of 0.5." << std::endl;
        gain = 0.5;
      }
      error = xiSetParamFloat(handle, XI_PRM_EXP_PRIORITY, gain);
      _CheckError(error, "xiSetParam (gain)");
    } else {
      float min_gain, max_gain;
      xiGetParamFloat(handle, XI_PRM_GAIN XI_PRM_INFO_MIN, &min_gain);
      xiGetParamFloat(handle, XI_PRM_GAIN XI_PRM_INFO_MAX, &max_gain);
      if (gain < min_gain || gain > max_gain) {
        std::cerr << "Gain Values = Min: " << min_gain << " - Max: "
                  << max_gain << std::endl;
        throw DeviceException("Requested GAIN not supported!");
      }
      error = xiSetParamFloat(handle, XI_PRM_GAIN, gain);
      _CheckError(error, "xiSetParam (gain)");
    }

    // Check if cameras are synced.
    if (sync_ < 0 || sync_ > 2) {
      throw DeviceException("Sync type can only be: 0(None), 1(Software) or 2(Hardware).");
    }

    // No sync. So, setting "FPS" parameter.
    if (sync_ == 0) {
      if (fps != 0) {
        error = xiSetParamInt(handle, XI_PRM_ACQ_TIMING_MODE,
                              XI_ACQ_TIMING_MODE_FRAME_RATE);
        _CheckError(error, "xiSetParam (ACQ Timing)");

        float min_fps, max_fps;
        xiGetParamFloat(handle, XI_PRM_FRAMERATE XI_PRM_INFO_MIN, &min_fps);
        xiGetParamFloat(handle, XI_PRM_FRAMERATE XI_PRM_INFO_MAX, &max_fps);
        if (fps < min_fps || fps > max_fps) {
          std::cerr << "Framerate Values = Min: " << min_fps << " - Max: "
                    << max_fps << std::endl;
          throw DeviceException("Requested FPS not supported!");
        }

        error = xiSetParamFloat(handle, XI_PRM_FRAMERATE, fps);
        _CheckError(error, "xiSetParam (FPS)");
      }
    }

    // Software sync.
    if (sync_ == 1) {
      xiSetParamInt(handle, XI_PRM_TRG_SOURCE, XI_TRG_SOFTWARE);
      xiSetParamInt(handle, XI_PRM_GPO_SELECTOR, 1);
      xiSetParamInt(handle, XI_PRM_GPO_MODE,  XI_GPO_FRAME_ACTIVE);
    }

    // Hardware sync.
    if (sync_ == 2) {
      if (first_cam) {
        // First camera is master.
        xiSetParamInt(handle, XI_PRM_TRG_SOURCE, XI_TRG_SOFTWARE);
        xiSetParamInt(handle, XI_PRM_GPO_SELECTOR, 1);
        xiSetParamInt(handle, XI_PRM_GPO_MODE,  XI_GPO_FRAME_ACTIVE);

        if (fps != 0) {
          error = xiSetParamInt(handle, XI_PRM_ACQ_TIMING_MODE,
                                XI_ACQ_TIMING_MODE_FRAME_RATE);
          _CheckError(error, "xiSetParam (ACQ Timing)");

          float min_fps, max_fps;
          xiGetParamFloat(handle, XI_PRM_FRAMERATE XI_PRM_INFO_MIN, &min_fps);
          xiGetParamFloat(handle, XI_PRM_FRAMERATE XI_PRM_INFO_MAX, &max_fps);
          if (fps < min_fps || fps > max_fps) {
            std::cerr << "Framerate Values = Min: " << min_fps << " - Max: "
                      << max_fps << std::endl;
            throw DeviceException("Requested FPS not supported!");
          }

          error = xiSetParamFloat(handle, XI_PRM_FRAMERATE, fps);
          _CheckError(error, "xiSetParam (FPS)");
        }

        first_cam = false;
      } else {
        // Other cameras are slave.
        xiSetParamInt(handle, XI_PRM_TRG_SOURCE, XI_TRG_EDGE_RISING);
        xiSetParamInt(handle, XI_PRM_GPI_SELECTOR, 1);
        xiSetParamInt(handle, XI_PRM_GPI_MODE,  XI_GPI_TRIGGER);
      }
    }



  }

  // Start acquisition.
  for (HANDLE handle: cam_handles_) {
    error = xiStartAcquisition(handle);
    _CheckError(error, "xiStartAcquisition");

    //Set the LED to indicate BUSY status (frame active, XI_LED_FRAME_ACTIVE)
    xiSetParamInt(handle, XI_PRM_LED_SELECTOR,  XI_MQ_LED_STATUS2);
    xiSetParamInt(handle, XI_PRM_LED_MODE, XI_LED_FRAME_ACTIVE);
  }

  // Reset image buffer.
  memset(&image_, 0, sizeof(image_));
  image_.size = sizeof(XI_IMG);
}

///////////////////////////////////////////////////////////////////////////
XimeaDriver::~XimeaDriver()
{
  // Close devices.
  for (HANDLE handle: cam_handles_) {
    xiCloseDevice(handle);
  }
}

///////////////////////////////////////////////////////////////////////////
bool XimeaDriver::Capture(hal::CameraMsg& images)
{
  XI_RETURN error = XI_OK;

  // Software sync -- all cameras.
  if (sync_ == 1) {
    for (HANDLE handle: cam_handles_) {
      xiSetParamInt(handle, XI_PRM_TRG_SOFTWARE, 1);
    }
  }

  // Hardware sync -- only first camera.
  if (sync_ == 2) {
    // Trigger acquisition on Master camera
    xiSetParamInt(cam_handles_[0], XI_PRM_TRG_SOFTWARE, 1);
  }

  // Grab data.
  for (HANDLE handle: cam_handles_) {
    error = xiGetImage(handle, 100, &image_);

    // If timeout, return false;
    if (error == 10) {
      return false;
    }
    _CheckError(error, "xiGetImage");

    // Allocate memory.
    hal::ImageMsg* pb_img = images.add_image();

    // Set timestamp from camera.
    images.set_device_time(image_.tsSec + 1e-6*image_.tsUSec);
    images.set_system_time(hal::Tic());

    pb_img->set_width(image_.width);
    pb_img->set_height(image_.height);

    if (image_format_ == XI_RAW8) {
      pb_img->set_data(image_.bp, image_width_ * image_height_);
      pb_img->set_type(hal::PB_UNSIGNED_BYTE);
      pb_img->set_format(hal::PB_LUMINANCE);
    } else if (image_format_ == XI_RAW16) {
      pb_img->set_data(image_.bp, 2 * image_width_ * image_height_);
      pb_img->set_type(hal::PB_UNSIGNED_SHORT);
      pb_img->set_format(hal::PB_LUMINANCE);
    } else if (image_format_ == XI_MONO8) {
      pb_img->set_data(image_.bp, image_width_ * image_height_);
      pb_img->set_type(hal::PB_UNSIGNED_BYTE);
      pb_img->set_format(hal::PB_LUMINANCE);
    } else if (image_format_ == XI_MONO16) {
      pb_img->set_data(image_.bp, 2 * image_width_ * image_height_);
      pb_img->set_type(hal::PB_UNSIGNED_SHORT);
      pb_img->set_format(hal::PB_LUMINANCE);
    } else if (image_format_ == XI_RGB24) {
      pb_img->set_data(image_.bp, 3 * image_width_ * image_height_);
      pb_img->set_type(hal::PB_UNSIGNED_BYTE);
      pb_img->set_format(hal::PB_BGR);
    } else if (image_format_ == XI_RGB32) {
      pb_img->set_data(image_.bp, 4 * image_width_ * image_height_);
      pb_img->set_type(hal::PB_UNSIGNED_BYTE);
      pb_img->set_format(hal::PB_RGBA);
    } else {
      std::cerr << "Image format not supported." << std::endl;
    }

  }

  images.set_system_time(hal::Tic());

  return error == XI_OK ? true : false;

}


///////////////////////////////////////////////////////////////////////////
std::string XimeaDriver::GetDeviceProperty(const std::string& /*sProperty*/)
{
  return std::string();
}

///////////////////////////////////////////////////////////////////////////
size_t XimeaDriver::NumChannels() const
{
  return num_channels_;
}

///////////////////////////////////////////////////////////////////////////
size_t XimeaDriver::Width(size_t /*idx*/) const
{
  return image_width_;
}

///////////////////////////////////////////////////////////////////////////
size_t XimeaDriver::Height(size_t /*idx*/) const
{
  return image_height_;
}

///////////////////////////////////////////////////////////////////////////
double XimeaDriver::MaxExposure(int) const
{
  return 1000000;
}

///////////////////////////////////////////////////////////////////////////
double XimeaDriver::MinExposure(int) const
{
  return 16;
}

///////////////////////////////////////////////////////////////////////////
double XimeaDriver::MaxGain(int) const
{
  return 6;
}

///////////////////////////////////////////////////////////////////////////
double XimeaDriver::MinGain(int) const
{
  return 0;
}

///////////////////////////////////////////////////////////////////////////
double XimeaDriver::Exposure(int channel)
{
  int result;
  HANDLE handle = cam_handles_[channel];
  xiGetParamInt(handle, "exposure", &result);
  return result;
}

///////////////////////////////////////////////////////////////////////////
void XimeaDriver::SetExposure(double exposure, int channel)
{
  HANDLE handle = cam_handles_[channel];
  xiSetParamInt(handle, "exposure", exposure);
}

///////////////////////////////////////////////////////////////////////////
double XimeaDriver::Gain(int channel)
{
  int result;
  HANDLE handle = cam_handles_[channel];
  xiGetParamInt(handle, "gain", &result);
  return result;
}

///////////////////////////////////////////////////////////////////////////
void XimeaDriver::SetGain(double gain, int channel)
{
  HANDLE handle = cam_handles_[channel];
  xiSetParamInt(handle, "gain", gain);
}

///////////////////////////////////////////////////////////////////////////
double XimeaDriver::ProportionalGain(int) const
{
  return 55.0;
}

///////////////////////////////////////////////////////////////////////////
double XimeaDriver::IntegralGain(int) const
{
  return 0.1;
}

///////////////////////////////////////////////////////////////////////////
double XimeaDriver::DerivativeGain(int) const
{
  return 0.0;
}
