#include "DeviceRegistry.h"

#include <HAL/Camera/CameraDevice.h>
#include <HAL/IMU/IMUDevice.h>
#include <HAL/LIDAR/LIDARDevice.h>
#include <HAL/Posys/PosysDevice.h>
#include <HAL/Car/CarDevice.h>
#include <HAL/Gamepad/GamepadDevice.h>

namespace hal
{

template<typename BaseDevice>
DeviceRegistry<BaseDevice>::DeviceRegistry() {
  RegisterAlias( "bumblebee", "convert:[fmt=MONO8]//debayer://deinterlace://dc1394:[mode=FORMAT7_3]//" );
  RegisterAlias( "twizzler",  "deinterlace://v4l://" );
}

template<typename BaseDevice>
DeviceRegistry<BaseDevice>::~DeviceRegistry()
{
}

template<typename BaseDevice>
DeviceRegistry<BaseDevice>& DeviceRegistry<BaseDevice>::Instance()
{
  static DeviceRegistry<BaseDevice> s_instance;
  return s_instance;
}

// Register factory pointer.
template<typename BaseDevice>
void DeviceRegistry<BaseDevice>::RegisterFactory(
    const std::string& device_name,
    DeviceFactory<BaseDevice>* factory
    )
{
  m_factories[device_name] = factory;
}

template<typename BaseDevice>
void DeviceRegistry<BaseDevice>::RegisterAlias(
    const std::string& name,
    const std::string& alias
    )
{
  m_aliases[name] = alias;
}

template<typename BaseDevice>
std::shared_ptr<BaseDevice> DeviceRegistry<BaseDevice>::Create(
    const Uri& uri )
{
  // Check for aliases
  std::map<std::string,std::string>::const_iterator iAlias= m_aliases.find( uri.scheme );

  if( iAlias != m_aliases.end() ){
    std::string sAlias = iAlias->second;
    std::ostringstream oss;
    oss << sAlias << uri.url;
    return Create(oss.str());
  }
  else{
    auto pf = m_factories.find(uri.scheme);
    if(pf != m_factories.end()) {
      std::shared_ptr<BaseDevice> dev = pf->second->GetDevice(uri);
      return dev;
    }
    else{
      throw DeviceException("Scheme '" + uri.scheme + "' not registered for factory");
    }
  }
}

template<typename BaseDevice>
void DeviceRegistry<BaseDevice>::PrintRegisteredDevices()
{
/*
  std::cout << "Registered device factories:\n";
  for( auto it = m_factories.begin; it != m_factories.end; it++ ){
     std::cout << it->first << std::endl;
  }
*/
}


template<typename BaseDevice>
void DeviceRegistry<BaseDevice>::Destroy(BaseDevice* /*dev*/)
{
}
// Explicitly instantiate desired registries.
template class DeviceRegistry<hal::CameraDriverInterface>;
template class DeviceRegistry<hal::IMUDriverInterface>;
template class DeviceRegistry<hal::GamepadDriverInterface>;
template class DeviceRegistry<hal::LIDARDriverInterface>;
template class DeviceRegistry<hal::PosysDriverInterface>;
template class DeviceRegistry<hal::CarDriverInterface>;
}
