
#include "OpenCVDriver.h"

#include <HAL/Utils/TicToc.h>

using namespace cv;
using namespace hal;

OpenCVDriver::OpenCVDriver(unsigned int cam_id, bool force_grey)
    : num_channels_(1), force_greyscale_(force_grey), cam_(cam_id) {
  if (!cam_.isOpened()) abort();

  img_width_ = cam_.get(CV_CAP_PROP_FRAME_WIDTH);
  img_height_ = cam_.get(CV_CAP_PROP_FRAME_HEIGHT);
}

OpenCVDriver::~OpenCVDriver() {}

bool OpenCVDriver::Capture(pb::CameraMsg& images_msg) {
  if(!cam_.isOpened()) {
    std::cerr << "HAL: Error reading from camera." << std::endl;
    return false;
  }
  images_msg.set_device_time(Tic());

  cv::Mat temp;
  bool success = cam_.read(temp);
  pb::ImageMsg* pbImg = images_msg.add_image();
  pbImg->set_type(pb::PB_UNSIGNED_BYTE);
  pbImg->set_height(img_height_);
  pbImg->set_width(img_width_);

  if (!success) return false;

  if(force_greyscale_) {
    cvtColor(temp, temp, CV_RGB2GRAY);
    pbImg->set_format(pb::PB_LUMINANCE);
    num_channels_ = 1;
  } else {
    pbImg->set_format(pb::PB_RGB);
    num_channels_ = 3;
  }

  // This may not store the image in contiguous memory which PbMsgs
  // requires, so we might need to copy it
  cv::Mat cv_image;
  if (!cv_image.isContinuous()) {
    temp.copyTo(cv_image);
  } else {
    cv_image = temp;
  }

  pbImg->set_data((const char*)cv_image.data,
                  img_height_ * img_width_ * num_channels_);
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
