#pragma once

#include <fstream>
#include <memory>

#include <Eigen/Eigen>
#include <PbMsgs/Messages.pb.h>

#define HAVE_OPENCV

#ifdef HAVE_OPENCV
#pragma GCC system_header
#include <opencv2/opencv.hpp>
#endif

namespace pb {

class ImageArray;

#ifdef HAVE_OPENCV

inline void ReadCvMat(const cv::Mat& cvImage, pb::ImageMsg* pbImage) {
  pbImage->set_data((const char*)cvImage.data,
                    cvImage.total() * cvImage.elemSize());
  pbImage->set_height(cvImage.rows);
  pbImage->set_width(cvImage.cols);

  if (cvImage.elemSize1() == 1) {
    pbImage->set_type(pb::PB_UNSIGNED_BYTE);
  } else if (cvImage.elemSize1() == 2) {
    pbImage->set_type(pb::PB_UNSIGNED_SHORT);
  } else if (cvImage.elemSize1() == 4) {
    pbImage->set_type(pb::PB_FLOAT);
  } else {
    abort();
  }

  if (cvImage.channels() == 1) {
    pbImage->set_format(pb::PB_LUMINANCE);
  } else if (cvImage.channels() == 3) {
    pbImage->set_format(pb::PB_RGB);
  } else {
    abort();
  }
}

/** This will make create a cv::Mat based on the data stored in the
 * given message.
 *
 * */
inline cv::Mat WriteCvMat(const pb::ImageMsg& pbImage) {
  int nCvType = 0;
  if (pbImage.type() == pb::PB_BYTE ||
      pbImage.type() == pb::PB_UNSIGNED_BYTE) {
    if (pbImage.format() == pb::PB_LUMINANCE) {
      nCvType = CV_8UC1;
    } else if (pbImage.format() == pb::PB_RGB) {
      nCvType = CV_8UC3;
    }
  } else if (pbImage.type() == pb::PB_UNSIGNED_SHORT ||
             pbImage.type() == pb::PB_SHORT) {
    if (pbImage.format() == pb::PB_LUMINANCE) {
      nCvType = CV_16UC1;
    } else if (pbImage.format() == pb::PB_RGB) {
      nCvType = CV_16UC3;
    }
  } else if (pbImage.type() == pb::PB_FLOAT) {
    if (pbImage.format() == pb::PB_LUMINANCE) {
      nCvType = CV_32FC1;
    } else if (pbImage.format() == pb::PB_RGB) {
      nCvType = CV_32FC3;
    }
  }

  return cv::Mat(pbImage.height(), pbImage.width(), nCvType,
                 (void*)pbImage.data().data());
}

inline void ReadFile(const std::string& sFileName, pb::ImageMsg* pbImage) {
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
#endif  // HAVE_OPENCV

class Image {
 public:
  /// Construct with only an ImageMsg reference. Caller is responsible
  /// for ensuring the data outlasts this Image and its cv::Mat
  explicit Image(const ImageMsg& img) : msg_(&img)
#ifdef HAVE_OPENCV
                                      , mat_(WriteCvMat(*msg_))
#endif  // HAVE_OPENCV
  {}

  /// Construct with a pointer to the parent ImageArray
  Image(const ImageMsg& img,
        const std::shared_ptr<const ImageArray>& source_array) :
      msg_(&img), source_array_(source_array)
#ifdef HAVE_OPENCV
      , mat_(WriteCvMat(*msg_))
#endif  // HAVE_OPENCV
  {}

  Image& operator=(const Image&) = default;
  Image(const Image&) = default;
  Image(Image&&) = default;

  unsigned int Width() const {
    return msg_->width();
  }

  unsigned int Height() const {
    return msg_->height();
  }

  int Type() const {
    return msg_->type();
  }

  int Format() const {
    return msg_->format();
  }

  double Timestamp() const {
    return msg_->timestamp();
  }

  const pb::ImageInfoMsg& GetInfo() const {
    msg_->info();
  }

  const bool HasInfo() const {
    return msg_->has_info();
  }

  const unsigned char* data() const {
    return (const unsigned char*)(&msg_->data().front());
  }

  const unsigned char* RowPtr(unsigned int row = 0) const {
    return data() + (row * Width());
  }

#ifdef HAVE_OPENCV
  operator cv::Mat() {
    return mat_;
  }

  cv::Mat& Mat() {
    return mat_;
  }
#endif  // HAVE_OPENCV

  template< typename T >
  T at(unsigned int row, unsigned int col) const {
    return *(T*)(RowPtr(row) + (col*sizeof(T)));
  }

  template< typename T >
  T& at(unsigned int row, unsigned int col) {
    return *(T*)(RowPtr(row) + (col*sizeof(T)));
  }

  unsigned char operator()(unsigned int row, unsigned int col) const {
    return *(RowPtr(row) + col);
  }

  const unsigned char& operator()(unsigned int row, unsigned int col) {
    return *(RowPtr(row) + col);
  }

 protected:
  const ImageMsg* msg_;

  /// Maintains a reference to the parent ImageArray to ensure its
  /// lifetime extends longer than this Image's
  std::shared_ptr<const ImageArray> source_array_;
#ifdef HAVE_OPENCV
  cv::Mat mat_;
#endif  // HAVE_OPENCV
};

}  // end namespace pb
