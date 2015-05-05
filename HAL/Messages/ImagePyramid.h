#pragma once

#include <miniglog/logging.h>
#include <HAL/Messages/Image.h>

namespace hal {
class ImagePyramid {
 public:
  /** Construct an image pyramid */
  ImagePyramid(size_t num_levels, double scale_factor)
      : num_levels_(num_levels), scale_factor_(scale_factor) {}

  /**
   * Performs a DEEP copy of the image pyramid
   */
  ImagePyramid(const ImagePyramid& other) {
    CopyFrom(other);
  }

  /**
   * Performs a DEEP copy of the image pyramid
   */
  ImagePyramid& operator=(const ImagePyramid& other) {
    CopyFrom(other);
    return *this;
  }

  virtual ~ImagePyramid() {}

  /**
   * Performs a DEEP copy of the image pyramid
   */
  void CopyFrom(const ImagePyramid& other) {
    if (this == &other) return;

    scale_factor_ = other.scale_factor_;
    num_levels_ = other.num_levels_;
    image_ = std::make_shared<Image>(*other.image_);

    levels_.reserve(other.levels_.size());
    levels_.emplace_back(image_->Mat());
    for (size_t level = 1; level < num_levels_; ++level) {
      levels_.emplace_back(other.levels_[level].clone());
    }
  }

  /**
   * (Re)Build the image pyramid from the given image with its own
   * intrinsic parameters (scale, # levels).
   *
   * This also allows reuse of an ImagePyramid structure, as long as
   * its parameters don't change.
   */
  void Build(const std::shared_ptr<Image>& image) {
    levels_.resize(num_levels_);
    image_ = image;
    levels_[0] = image->Mat();

    for (size_t level = 1; level < num_levels_; ++level) {
      cv::resize(levels_[level - 1], levels_[level],
                 cv::Size(), scale_factor_, scale_factor_);
    }
  }

  double ScaleFactor() const {
    return scale_factor_;
  }

  size_t NumLevels() const {
    return num_levels_;
  }

  cv::Mat& operator[](size_t i) {
    CHECK_LT(i, levels_.size());
    return levels_[i];
  }

  cv::Mat& at(size_t i) {
    CHECK_LT(i, levels_.size());
    return levels_[i];
  }

  const cv::Mat& at(size_t i) const {
    CHECK_LT(i, levels_.size());
    return levels_[i];
  }

  std::shared_ptr<Image> base() const {
    return image_;
  }

  bool Initialized() const {
    return levels_.size() == num_levels_;
  }

 private:
  // We hold onto the image that created us to ensure its lifetime
  std::shared_ptr<Image> image_;

  // The pyramid levels
  std::vector<cv::Mat> levels_;

  size_t num_levels_;
  double scale_factor_;
};
}  // end namespace hal
