/*
    \file Master interface for all car drivers
*/

#pragma once

// each car driver can have a generic set of "properties"
#include <HAL/Devices/DriverInterface.h>

namespace hal {

///////////////////////////////////////////////////////////////////////////////
// Generic Car Driver Interface
///////////////////////////////////////////////////////////////////////////////
class CarDriverInterface : public DriverInterface
{
public:
  virtual ~CarDriverInterface() {}

  virtual bool ApplyCommand( double flTorque, double flSteering,
                             double flCommand_time) = 0;
};

}
