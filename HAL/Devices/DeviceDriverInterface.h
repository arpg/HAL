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
  // Top level device driver class s.t. all drivers have properties that can be set/get.
  class DeviceDriverInterface 
  {
    public:

      // all drivers are required to have an init function.
      virtual bool Init( 
          const PropertyMap& default_properties, // from factory
          const Uri& uri
          ) = 0;

      // property map accessors
      template<typename T>
        void SetProperty(const std::string& key, T value)
        {   
          driver_properties_.SetProperty(key,value);
        }

      template<typename T>
        T GetProperty(const std::string& key, T default_val = T() ) const
        {
          return driver_properties_.GetProperty(key,default_val);
        }


      void SetDefaultProperties( const std::vector<param_t>& params )
      {
        driver_properties_.SetProperties(params);
      }

      void SetDefaultProperties( const PropertyMap& props )
      {
        driver_properties_ = props;
      }

      bool ParseUriProperties( const PropertyMap& m )
      {
        for( auto& it : m.GetPropertyMap() ){
          if( !driver_properties_.Contains( it.first ) ){
            std::cerr << "Unkown property: '" << it.first << "'\n";
            return false;
          }
        }
        // OK all properties are known, so just copy them in.
        driver_properties_.SetProperties( m );
        return true;
      }

      void PrintPropertyMap()
      {
        driver_properties_.PrintPropertyMap();
      }

    protected:
      // This is used for passing values in and out of drivers...
      PropertyMap driver_properties_;

  };

}

