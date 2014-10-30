#include <iostream>

#include <HAL/Devices/DeviceException.h>
#include <HAL/Utils/TicToc.h>

#include "XimeaDriver.h"

using namespace hal;

///////////////////////////////////////////////////////////////////////////
inline void XimeaDriver::_CheckError(
    XI_RETURN     err,
    std::string   place
  )
{
  if (err != XI_OK) {
    std::cerr << "Error after" << place << " " << err << std::endl;
    throw DeviceException("Ximea SDK exception!");
  }
}

///////////////////////////////////////////////////////////////////////////
XimeaDriver::XimeaDriver(
    std::vector<unsigned int>&  vector_ids,
    float                       fps,
    XI_IMG_FORMAT               mode,
    ImageRoi                    roi
  )
{
  XI_RETURN error = XI_OK;

  // Initialize image size.
  image_format = mode;
  image_width  = roi.w;
  image_height = roi.h;

  // Get number of camera devices.
  DWORD num_devices;
  error = xiGetNumberDevices(&num_devices);
  _CheckError(error, "xiGetNumberDevices");

  if (num_devices == 0) {
    throw DeviceException("No cameras found!");
  }

  // If no ids are provided, all cameras will be opened.
  if (vector_ids.empty()) {
    num_channels = num_devices;
  } else {
    num_channels = vector_ids.size();
  }

  if (num_channels < vector_ids.size()) {
    throw DeviceException("Less cameras detected than those requested!");
  }

  // Resize camera handle vector to accomodate requested cameras.
  cam_handle.resize(num_channels);

  for (size_t ii = 0; ii < num_channels; ++ii) {
    // Retrieving a handle to the camera device.
    error = xiOpenDevice(ii, &cam_handle[ii]);
    _CheckError(error, "xiOpenDevice");
  }

  // Set parameters.
  for (HANDLE handle: cam_handle) {
    // Setting "format" parameter. Has to be set first before image size!
    error = xiSetParamInt(handle, XI_PRM_IMAGE_DATA_FORMAT, image_format);
    _CheckError(error, "xiSetParam (format)");

    int max_width, max_height;
    xiGetParamInt(handle, XI_PRM_WIDTH XI_PRM_INFO_MAX, &max_width);
    xiGetParamInt(handle, XI_PRM_HEIGHT XI_PRM_INFO_MAX, &max_height);

    // Setting "width" parameter.
    if (image_width <= max_width - roi.x) {
      error = xiSetParamInt(handle, XI_PRM_WIDTH, image_width);
      _CheckError(error, "xiSetParam (width)");

      int left = roi.x;
      if (left == 0) {
        left = (max_width / 2) - image_width / 2;
      }

      error = xiSetParamInt(handle, XI_PRM_OFFSET_X, left);
      _CheckError(error, "xiSetParam (x-offset)");
    } else {
      std::cerr << "Image width out of bounds. Maximum value: " <<
                   max_width << std::endl;
      throw DeviceException("Image out of bounds!");
    }

    // Setting "height" parameter.
    if (image_height <= max_height - roi.y) {
      error = xiSetParamInt(handle, XI_PRM_HEIGHT, image_height);
      _CheckError(error, "xiSetParam (height)");

      int top = roi.y;
      if (top == 0) {
        top = (max_height / 2) - image_height / 2;
      }

      error = xiSetParamInt(handle, XI_PRM_OFFSET_Y, top);
      _CheckError(error, "xiSetParam (y-offset)");
    } else {
      std::cerr << "Image height out of bounds. Maximum value: " <<
                   max_height << std::endl;
      throw DeviceException("Image out of bounds!");
    }

    // Setting "FPS" parameter.
    if (fps != 0) {
      float min_fps, max_fps;
      xiGetParamFloat(handle, XI_PRM_FRAMERATE XI_PRM_INFO_MIN, &min_fps);
      xiGetParamFloat(handle, XI_PRM_FRAMERATE XI_PRM_INFO_MAX, &max_fps);
      if (fps < min_fps || fps > max_fps) {
        std::cerr << "Framerate Values = Min: " << min_fps << " - Max: "
                  << max_fps << std::endl;
        throw DeviceException("Requested FPS not supported!");
      }
      error = xiSetParamInt(handle, XI_PRM_ACQ_TIMING_MODE,
                            XI_ACQ_TIMING_MODE_FRAME_RATE);
      _CheckError(error, "xiSetParam (ACQ Timing)");
      error = xiSetParamFloat(handle, XI_PRM_FRAMERATE, fps);
      _CheckError(error, "xiSetParam (FPS)");
    }

    // Setting "exposure" parameter (10ms=10000us).
    error = xiSetParamInt(handle, XI_PRM_EXPOSURE, 10000);
    _CheckError(error, "xiSetParam (exposure)");
  }

  // Start acquisition.
  for (HANDLE handle: cam_handle) {
    error = xiStartAcquisition(handle);
    _CheckError(error, "xiStartAcquisition");
  }

  // Reset image buffer.
  memset(&image, 0, sizeof(image));
  image.size = sizeof(XI_IMG);
}

///////////////////////////////////////////////////////////////////////////
XimeaDriver::~XimeaDriver()
{
  // Close devices.
  for (HANDLE handle: cam_handle) {
      xiCloseDevice(handle);
  }
}

///////////////////////////////////////////////////////////////////////////
bool XimeaDriver::Capture(pb::CameraMsg& images)
{
  XI_RETURN error = XI_OK;

  for (HANDLE handle: cam_handle) {
    pb::ImageMsg* pb_img = images.add_image();

    error = xiGetImage(handle, 5000, &image);
    _CheckError(error, "xiGetImage");

    // Set timestamp from camera.
    images.set_device_time(image.tsSec + 1e-9*image.tsUSec);

    pb_img->set_width(image.width);
    pb_img->set_height(image.height);

    if (image_format == XI_MONO8) {
      pb_img->set_data(image.bp, image_width * image_height);
      pb_img->set_type(pb::PB_UNSIGNED_BYTE);
      pb_img->set_format(pb::PB_LUMINANCE);
    } else if (image_format == XI_MONO16) {
      pb_img->set_data(image.bp, 2 *image_width * image_height);
      pb_img->set_type(pb::PB_UNSIGNED_SHORT);
      pb_img->set_format(pb::PB_LUMINANCE);
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
  return num_channels;
}

///////////////////////////////////////////////////////////////////////////
size_t XimeaDriver::Width(size_t /*idx*/) const
{
  return image_width;
}

///////////////////////////////////////////////////////////////////////////
size_t XimeaDriver::Height(size_t /*idx*/) const
{
  return image_height;
}
