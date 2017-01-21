#pragma once

#include <functional>

#include <HAL/Devices/DriverInterface.h>

#include <HAL/Pose.pb.h>


namespace hal {

typedef std::function<void (hal::PoseMsg&)> PosysDriverDataCallback;
typedef std::function<void ()> PosysDriverFinishedCallback;

///
/// Generic Posys Driver Interface
class PosysDriverInterface : public DriverInterface {
 public:
  virtual ~PosysDriverInterface() {}
  virtual void RegisterPosysDataCallback(PosysDriverDataCallback callback) = 0;
  virtual void RegisterPosysFinishedCallback(PosysDriverFinishedCallback ) {}
  virtual bool IsRunning() const = 0;
};

} /* namespace hal */
