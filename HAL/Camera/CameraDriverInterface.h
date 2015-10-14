#pragma once

#include <memory>

#include <HAL/Devices/DeviceDriverInterface.h>

#include <HAL/Camera.pb.h>

namespace hal {

  // Generic camera driver interface
  class CameraDriverInterface : public DeviceDriverInterface
  {
    public:
      // Pure virtual functions driver writers must implement:
      virtual ~CameraDriverInterface() {}
      virtual bool Capture( hal::CameraMsg& vImages ) = 0;

			// These tell users how data is laid out in the image (these have a
			// one-to-one mapping with GL format and type).
			virtual hal::Type   PixelType() = 0;
			virtual hal::Format PixelFormat() = 0;

      // Allows cascading of drivers
      virtual std::shared_ptr<CameraDriverInterface> GetInputDriver() = 0;

      virtual size_t NumChannels() const = 0;
      virtual size_t Width( size_t /*idx*/ = 0 ) const = 0;
      virtual size_t Height( size_t /*idx*/ = 0 ) const = 0;
  };

}
