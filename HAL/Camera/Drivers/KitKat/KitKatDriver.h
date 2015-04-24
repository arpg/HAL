// Copyright (c) George Washington University, all rights reserved. See the
// accompanying LICENSE file for more information.

#pragma once

#include <HAL/Camera/CameraDriverInterface.h>

struct camera_t;

namespace hal {

class KitKatDriver : public CameraDriverInterface {
 public:
  KitKatDriver();
  virtual ~KitKatDriver();

  bool Capture( hal::CameraMsg& vImages );

  size_t NumChannels() const;
  size_t Width( size_t /*idx*/ = 0 ) const;
  size_t Height( size_t /*idx*/ = 0 ) const;

  inline std::shared_ptr<CameraDriverInterface> GetInputDevice() {
    return std::shared_ptr<CameraDriverInterface>();
  }

 private:
  camera_t* camera_;
};
}  // namespace hal
