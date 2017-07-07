#pragma once

#include <functional>
#include <HAL/Devices/DriverInterface.h>
#include <HAL/Signal.pb.h>

namespace hal {

typedef std::function<void (hal::SignalMsg&)> SignalDriverDataCallback;
typedef std::function<void ()> SignalDriverFinishedCallback;

/// Generic Signal driver interface
///
class SignalDriverInterface : public DriverInterface {
 public:
  virtual ~SignalDriverInterface() {}
  virtual void RegisterSignalDataCallback(SignalDriverDataCallback callback) = 0;
  virtual void RegisterSignalFinishedCallback(SignalDriverFinishedCallback ) {}
  virtual bool IsRunning() const = 0;
};

} /* namespace */
