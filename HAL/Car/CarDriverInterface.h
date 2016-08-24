#pragma once
#include <functional>
#include <HAL/Devices/DriverInterface.h>
#include <HAL/Car.pb.h>

namespace hal {
typedef std::function<void (hal::CarStateMsg&)> CarStateDataCallback;

///////////////////////////////////////////////////////////////////////////////
// Generic Car Driver Interface
///////////////////////////////////////////////////////////////////////////////
class CarDriverInterface : public DriverInterface
{
public:
  virtual ~CarDriverInterface() {}

  virtual void UpdateCarCommand( CarCommandMsg& command_msg) = 0;

  virtual void RegisterCarStateDataCallback(CarStateDataCallback callback) = 0;
};

}
