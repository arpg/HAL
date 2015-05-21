#include <HAL/Messages/Image.h>

#include <fstream>
#include <memory>

#include <HAL/Messages.pb.h>
#include <glog/logging.h>

namespace hal {

void ReadCvMat(const cv::Mat& cvImage, hal::ImageMsg* pbImage) {
  pbImage->set_data((const char*)cvImage.data,
                    cvImage.total() * cvImage.elemSize());
  pbImage->set_height(cvImage.rows);
  pbImage->set_width(cvImage.cols);

  if (cvImage.elemSize1() == 1) {
    pbImage->set_type(hal::PB_UNSIGNED_BYTE);
  } else if (cvImage.elemSize1() == 2) {
    pbImage->set_type(hal::PB_UNSIGNED_SHORT);
  } else if (cvImage.elemSize1() == 4) {
    pbImage->set_type(hal::PB_FLOAT);
  } else {
    LOG(FATAL) << "Unknown image type";
  }

  if (cvImage.channels() == 1) {
    pbImage->set_format(hal::PB_LUMINANCE);
  } else if (cvImage.channels() == 3) {
    pbImage->set_format(hal::PB_RGB);
  } else {
    LOG(FATAL) << "Unknown number of image channels";
  }
}

cv::Mat WriteCvMat(const hal::ImageMsg& pbImage) {
  int nCvType = 0;
  if (pbImage.type() == hal::PB_BYTE ||
      pbImage.type() == hal::PB_UNSIGNED_BYTE) {
    if (pbImage.format() == hal::PB_LUMINANCE) {
      nCvType = CV_8UC1;
    } else if (pbImage.format() == hal::PB_RGB) {
      nCvType = CV_8UC3;
    } else if (pbImage.format() == hal::PB_BGR) {
      nCvType = CV_8UC3;
    }

  } else if (pbImage.type() == hal::PB_UNSIGNED_SHORT ||
             pbImage.type() == hal::PB_SHORT) {
    if (pbImage.format() == hal::PB_LUMINANCE) {
      nCvType = CV_16UC1;
    } else if (pbImage.format() == hal::PB_RGB) {
      nCvType = CV_16UC3;
    }
  } else if (pbImage.type() == hal::PB_FLOAT) {
    if (pbImage.format() == hal::PB_LUMINANCE) {
      nCvType = CV_32FC1;
    } else if (pbImage.format() == hal::PB_RGB) {
      nCvType = CV_32FC3;
    }
  }

  return cv::Mat(pbImage.height(), pbImage.width(), nCvType,
                 (void*)pbImage.data().data());
}

void ReadFile(const std::string& sFileName, hal::ImageMsg* pbImage) {
  cv::Mat Image;

  std::string sExtension = sFileName.substr(sFileName.rfind(".") + 1);

  // check if it is our own "portable depth map" format
  if (sExtension == "pdm") {

    // magic number P7, portable depthmap, binary
    std::ifstream File(sFileName.c_str());

    if (File.is_open()) {
      std::string sType;
      File >> sType;

      unsigned int nImgWidth = 0;
      File >> nImgWidth;

      unsigned int nImgHeight = 0;
      File >> nImgHeight;

      long unsigned int nImgSize = 0;
      File >> nImgSize;

      // the actual PGM/PPM expects this as the next field:
      //		nImgSize++;
      //		nImgSize = (log(nImgSize) / log(2)) / 8.0;

      // but ours has the actual size (4 bytes of float * pixels):
      nImgSize = 4 * nImgWidth * nImgHeight;

      Image.create(nImgHeight, nImgWidth, CV_32FC1);

      File.seekg(File.tellg() + (std::ifstream::pos_type)1, std::ios::beg);
      File.read((char*)Image.data, nImgSize);
      File.close();
    }
  } else {
    // ... otherwise let OpenCV open it
    Image = cv::imread(sFileName, cv::IMREAD_UNCHANGED);
  }
  ReadCvMat(Image, pbImage);
}

/// Construct with only an ImageMsg reference. Caller is responsible
/// for ensuring the data outlasts this Image and its cv::Mat
Image::Image(const ImageMsg& img) : msg_(&img),
                                    mat_(WriteCvMat(*msg_)),
                                    owns_image_(false) {
}

/// Construct with a pointer to the parent ImageArray
Image::Image(const ImageMsg& img,
             const std::shared_ptr<const ImageArray>& source_array) :
    msg_(&img), source_array_(source_array),
    mat_(WriteCvMat(*msg_)),
    owns_image_(false)
{}

Image& Image::operator=(const Image& other) {
  if (this != &other) {
    // If we've already created our own image, free it before overwriting
    if (owns_image_) {
      delete msg_;
    }

    owns_image_ = true;
    msg_ = new hal::ImageMsg(*other.msg_);
    source_array_.reset();
    mat_ = WriteCvMat(*msg_);
  }
  return *this;
}

Image::Image(const Image& other) : msg_(new hal::ImageMsg(*other.msg_)),
                                   mat_(WriteCvMat(*msg_)),
                                   owns_image_(true) {
}

Image::~Image() {
  if (owns_image_) {
    delete msg_;
  }
}

unsigned int Image::Width() const {
  return msg_->width();
}

unsigned int Image::Height() const {
  return msg_->height();
}

int Image::Type() const {
  return msg_->type();
}

int Image::Format() const {
  return msg_->format();
}

long int Image::SerialNumber() const {
  return msg_->serial_number();
}

double Image::Timestamp() const {
  return msg_->timestamp();
}

const hal::ImageInfoMsg& Image::GetInfo() const {
  return msg_->info();
}

bool Image::HasInfo() const {
  return msg_->has_info();
}

const unsigned char* Image::data() const {
  return (const unsigned char*)(&msg_->data().front());
}

const unsigned char* Image::RowPtr(unsigned int row) const {
  return data() + (row * Width());
}

}  // end namespace hal
