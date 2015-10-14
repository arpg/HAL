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
        virtual std::shared_ptr<BaseDevice> CreateDriver( 
            PropertyMap& device_properties,
            const Uri& uri
            ) = 0;

        void PrintDefaultParams()
        {
           default_params_.PrintPropertyMap(); 
        }

        // helper function handles boilerplate common to all factory methods
        // that initialzes a devices property map.
        bool InitDevicePropertyMap( 
            PropertyMap& device_properties, // Output property map to init
            const Uri& uri // Input uri with user specified properties
            )
        {
          // Set sane default parameters.
          // Defaults come from each specific constructor.
          device_properties = default_params_;
          // See what user wants to set (MUST be a subset of known properties)
          for( auto& it : uri.properties.GetPropertyMap() ){
            if( !device_properties.Contains( it.first ) ){
              std::cerr << "Unkown property: '" << it.first << "'\n";
              std::cerr << name_ << "Driver knows about the following properties:\n";
              device_properties.PrintPropertyMap();
              return false;
            }
            param_t p = it.second;
            device_properties.SetProperty(p.key,p.value,p.desc);
          }
          return true;
        }


      protected:
        std::string   name_;
        PropertyMap   default_params_; // NOT the same as specific Driver's property map.
    };
}

