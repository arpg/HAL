#include "DeviceDriverRegistry.h"

#include <HAL/Camera/CameraDevice.h>
#include <HAL/IMU/IMUDevice.h>
//#include <HAL/Encoder/EncoderDevice.h>
//#include <HAL/LIDAR/LIDARDevice.h>
//#include <HAL/Posys/PosysDevice.h>

namespace hal
{

  template<typename BaseDevice>
    DeviceDriverRegistry<BaseDevice>::DeviceDriverRegistry() 
    {
      RegisterAlias( "bumblebee", "convert:[fmt=MONO8]//debayer://deinterlace://dc1394:[mode=FORMAT7_3]//" );
    }

  template<typename BaseDevice>
    DeviceDriverRegistry<BaseDevice>::~DeviceDriverRegistry()
    {
    }

  template<typename BaseDevice>
    DeviceDriverRegistry<BaseDevice>& DeviceDriverRegistry<BaseDevice>::Registry()
    {
      static DeviceDriverRegistry<BaseDevice> s_instance;
      return s_instance;
    }

  // Register factory pointer.
  template<typename BaseDevice>
    void DeviceDriverRegistry<BaseDevice>::RegisterFactory(
        const std::string& device_name,
        DriverFactory<BaseDevice>* factory
        )
    {
      m_factories[device_name] = factory;
    }

  template<typename BaseDevice>
    void DeviceDriverRegistry<BaseDevice>::RegisterAlias(
        const std::string& name,
        const std::string& alias
        )
    {
      m_aliases[name] = alias;
    }

  // called by Device constructors (e.g., CamreaDevice::CameraDevice)
  template<typename BaseDevice>
    DriverFactory<BaseDevice>& DeviceDriverRegistry<BaseDevice>::GetFactory(
        const Uri& uri )
    {
      // Check for aliases
      std::map<std::string,std::string>::const_iterator iAlias= m_aliases.find( uri.scheme );

      if( iAlias != m_aliases.end() ){
        std::string sAlias = iAlias->second;
        std::ostringstream oss;
        oss << sAlias << uri.url;
        return GetFactory( oss.str() );
      }
      auto pf = m_factories.find(uri.scheme);
      return *pf->second;
    }



  // list all drivers registered with this device driver registry.
  template<typename BaseDevice>
    void DeviceDriverRegistry<BaseDevice>::ListDrivers()
    {
      std::cout << "\nRegistered HAL device drivers:\n";
      for( auto& it : m_factories ){
        std::cout << "  " << it.first << std::endl;
      }
      std::cout << "\n\n";
    }

  // Explicitly instantiate desired registries.
  template class DeviceDriverRegistry<hal::CameraDriverInterface>;
  template class DeviceDriverRegistry<hal::IMUDriverInterface>;
//  template class DeviceDriverRegistry<hal::EncoderDriverInterface>;
//  template class DeviceDriverRegistry<hal::LIDARDriverInterface>;
//  template class DeviceDriverRegistry<hal::PosysDriverInterface>;

}

