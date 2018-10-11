#pragma once

#include <memory>
#include <HAL/Messages.pb.h>

#pragma GCC system_header
#include <opencv2/opencv.hpp>

namespace hal {

class ImageArray;

/** This will make create a cv::Mat based on the data stored in the
 * given message.
 *
 * */
cv::Mat WriteCvMat(const hal::ImageMsg& pbImage);
void ReadCvMat(const cv::Mat& cvImage, hal::ImageMsg* pbImage);
void ReadFile(const std::string& sFileName, hal::ImageMsg* pbImage);

/**
 * Basic image class used as a wrapper around an ImageMsg.
 *
 * IMPORTANT: Copy-semantics of hal::Image:
 * =======================================
 *
 * None of the basic constructors copy the image they are given!
 *
 * When constructed from an image message in an ImageArray, will
 * maintain a shared_ptr to that ImageArray to keep the Image alive
 * for (at least) as long as the Image exists.
 *
 * When hal::Image is copied, it will perform a DEEP copy of the image,
 * and assume ownership of the copied Image. This follows normal C++
 * copy-semantics. Pass the hal::Image by reference if you don't want a
 * deep copy.
 *
 */
class Image {
 public:
  /// Construct with only an ImageMsg reference. Caller is responsible
  /// for ensuring the data outlasts this Image and its cv::Mat
  ///
  /// NO-COPY
  explicit Image(const ImageMsg& img);

  /// Construct with a pointer to the parent ImageArray.
  ///
  /// NO-COPY
  Image(const ImageMsg& img,
        const std::shared_ptr<const ImageArray>& source_array);

  /// Performs a DEEP copy of the Image and takes ownership of the image
  Image& operator=(const Image& other);

  /// Performs a DEEP copy of the Image and takes ownership of the image
  Image(const Image& other);

  /// Move constructor performs a SHALLOW copy
  Image(Image&&) = default;

  virtual ~Image();
  unsigned int Width() const;
  unsigned int Height() const;
  long int SerialNumber() const;
  int Type() const;
  int Format() const;
  double Timestamp() const;
  const hal::ImageInfoMsg& GetInfo() const;
  bool HasInfo() const;
  const unsigned char* data() const;
  const unsigned char* RowPtr(unsigned int row = 0) const;

  operator cv::Mat() {
    return mat_;
  }

  cv::Mat& Mat() {
    return mat_;
  }

  const cv::Mat& Mat() const {
    return mat_;
  }

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
  cv::Mat mat_;
  bool owns_image_;
};

}  // end namespace hal
