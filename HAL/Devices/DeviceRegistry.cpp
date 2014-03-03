#include "DeviceRegistry.h"

#include <HAL/Camera/CameraDevice.h>
#include <HAL/Encoder/EncoderDevice.h>
#include <HAL/IMU/IMUDevice.h>
#include <HAL/LIDAR/LIDARDevice.h>
#include <HAL/Posys/PosysDevice.h>
#include <HAL/Car/CarDevice.h>

#ifdef HAVE_TINYXML2
  #include <HAL/Utils/SimLauncher.h>
#endif

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
DeviceRegistry<BaseDevice>& DeviceRegistry<BaseDevice>::I()
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
    const Uri& uri,
    const char *sDeviceType)
{
#ifdef HAVE_TINYXML2
  char *sEnv = getenv("SIM");
  if(sEnv!= NULL)
  {
    if(sDeviceType==NULL)
      throw DeviceException("DeviceType was not set when create was not"
                            " called, so i don't know what driver to"
                            " use for Simulating the Device");

    std::string sSimName;
    if( !LaunchSimulationIfNeeded(sSimName) )
        throw DeviceException("You set SIM Environment variable, but"
                              " Sim was not succesfully launched."
                              " Please Read messages above this one for"
                              " more detail.");


    //TODO: Create a Node<device> object and return
    //If we are here that means Simulator is launched and we have a device
    //type passed to us, but then you knew that, didn't you.
    std::string sDriverName = "Node"; sDriverName.append( sDeviceType);
    hal::Uri simUri = uri;
    simUri.SetProperties("sim=" + sSimName);

    auto pf = m_factories.find(sDriverName);
    if(pf != m_factories.end()) {

      std::shared_ptr<BaseDevice> dev = pf->second->GetDevice(simUri);
      return dev;
    }
    else{
      throw DeviceException("Simulation Device for " + std::string(sDeviceType)
                            + " was not found. Perhaps BUILD_" + sDriverName
                            + "was off while building HAL.");
    }
  }
#endif  // HAVE_TINYXML2

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
void DeviceRegistry<BaseDevice>::Destroy(BaseDevice* /*dev*/)
{
}

// Explicitly instantiate desired registries.
template class DeviceRegistry<hal::CameraDriverInterface>;
template class DeviceRegistry<hal::EncoderDriverInterface>;
template class DeviceRegistry<hal::IMUDriverInterface>;
template class DeviceRegistry<hal::LIDARDriverInterface>;
template class DeviceRegistry<hal::PosysDriverInterface>;
template class DeviceRegistry<hal::CarDriverInterface>;

}
