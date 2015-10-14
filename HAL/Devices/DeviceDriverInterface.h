#pragma once

#include <string>
#include <iostream>
#include <sstream>
#include <string>
#include <map>
#include <limits>

#include <HAL/Utils/PropertyMap.h>
#include <HAL/Utils/Uri.h>

namespace hal 
{
  class DeviceDriverInterface 
  {
    public:
      DeviceDriverInterface() {}
      virtual void PrintInfo() = 0;
  };

}

