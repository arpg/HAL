#pragma once

#include <iostream>
#include <memory>
#include <PbMsgs/Messages.pb.h>
#include <PbMsgs/Image.h>

namespace pb {

class ImageArray : public std::enable_shared_from_this<ImageArray> {
 public:
  static std::shared_ptr<ImageArray> Create() {
    return std::shared_ptr<ImageArray>(new ImageArray);
  }

  CameraMsg& Ref() {
    return message_;
  }

  int Size() const {
    return message_.image_size();
  }

  bool Empty() const {
    return message_.image_size() == 0;
  }

  double Timestamp() const {
    return message_.image(0).timestamp();
  }

  Image at(int idx) const {
    if (idx < Size()) {
      return Image(message_.image(idx), shared_from_this());
    }

    // TODO: define ensure macro or throw exception
    std::cerr << "error: Channel index '" << idx
              << "' out of bounds: did you split the image stream?"
              << std::endl;
    exit(1);
  }

  Image operator[](int idx) const {
    return at(idx);
  }

 private:
  ImageArray() {}
  CameraMsg message_;
};

}