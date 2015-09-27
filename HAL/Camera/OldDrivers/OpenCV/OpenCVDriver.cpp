
#include "OpenCVDriver.h"

#include <HAL/Utils/TicToc.h>

using namespace cv;
using namespace hal;

OpenCVDriver::OpenCVDriver( const Uri& uri )
{
  // sat sane default parameters
  CameraDriverInterface::SetDefaultProperties({
      {"id", "0", "Id of the camera to use." },
      {"grey", "false", "convert to greyscale"}
      });
  if( !CameraDriverInterface::ParseUriProperties( uri.properties ) ){
    std::cerr << "OpenCVDriver knows about the following properties:\n";
    CameraDriverInterface::PrintPropertyMap();
    return;
  }

  num_channels_ = 1;
  force_greyscale_    = GetProperty<bool>("grey", false);
  unsigned int cam_id = GetProperty<unsigned int>("id", 0);

  cam_ = cv::VideoCapture(cam_id);

  if (!cam_.isOpened()){
    abort();
  }

  img_width_ = cam_.get(CV_CAP_PROP_FRAME_WIDTH);
  img_height_ = cam_.get(CV_CAP_PROP_FRAME_HEIGHT);
}

OpenCVDriver::~OpenCVDriver() {}

bool OpenCVDriver::Capture(hal::CameraMsg& images_msg) {
  if(!cam_.isOpened()) {
    std::cerr << "HAL: Error reading from camera." << std::endl;
    return false;
  }
  images_msg.set_device_time(Tic());

  cv::Mat temp;
  bool success = cam_.read(temp);
  hal::ImageMsg* pbImg = images_msg.add_image();
  pbImg->set_type(hal::PB_UNSIGNED_BYTE);
  pbImg->set_height(img_height_);
  pbImg->set_width(img_width_);
  pbImg->set_format(force_greyscale_ ? hal::PB_LUMINANCE : hal::PB_BGR);

  if (!success) return false;

  // This may not store the image in contiguous memory which PbMsgs
  // requires, so we might need to copy it
  cv::Mat cv_image;
  if(force_greyscale_) {
    cvtColor(temp, cv_image, CV_RGB2GRAY);
  } else if (!cv_image.isContinuous()) {
    temp.copyTo(cv_image);
  } else {
    cv_image = temp;
  }

  pbImg->set_data(static_cast<const unsigned char*>(cv_image.data),
      cv_image.elemSize() * cv_image.total());
  return true;
}

size_t OpenCVDriver::NumChannels() const {
  return num_channels_;
}

size_t OpenCVDriver::Width(size_t /*idx*/) const {
  return img_width_;
}

size_t OpenCVDriver::Height(size_t /*idx*/) const {
  return img_height_;
}
