#pragma once

#include <HAL/Devices/DeviceRegistry.h>

#include <memory>
#include <vector>


namespace hal 
{

  template<typename BaseDevice>
    class DriverFactory
    {
      public:        

        DriverFactory(std::string name)
          : name_(name)
        {
          Registry().RegisterFactory(name_, this);
        }

        virtual ~DriverFactory() {}
        virtual std::shared_ptr<BaseDevice> CreateDriver(const Uri& uri) = 0;

        inline static hal::DeviceRegistry<BaseDevice>& Registry() 
        {
          return hal::DeviceRegistry<BaseDevice>::Instance();
        }

      protected:
        std::string name_;
    };

}
