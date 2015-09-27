#pragma once

#include <HAL/Devices/DeviceDriverRegistry.h>

#include <memory>
#include <vector>


namespace hal 
{

  template<typename BaseDevice>
    class DriverFactory
    {
      public:

        DriverFactory( const std::string& name="", const std::vector<param_t>& params={"","",""} )
          : name_(name) 
        {
          default_params_.SetProperties(params);
          hal::DeviceDriverRegistry<BaseDevice>::Registry().RegisterFactory(name_, this);
        }

        virtual ~DriverFactory() {}
        virtual std::shared_ptr<BaseDevice> CreateDriver(const Uri& uri) = 0;
       
        void PrintDefaultParams()
        {
           default_params_.PrintPropertyMap(); 
        }

      protected:
        std::string   name_;
        PropertyMap   default_params_;
    };
}

