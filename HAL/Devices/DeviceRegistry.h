#pragma once

#include <HAL/config.h>
#include <HAL/Utils/Uri.h>

#include <memory>
#include <map>


#ifndef __GXX_EXPERIMENTAL_CXX0X__
    #error "C++11 is required to use HAL. Please enable -std=c++0x or -std=c++11 flag on compiler options."
#endif

namespace hal
{

// Forward declaration
template<typename BaseDevice> class DeviceFactory;

template<typename BaseDevice>
class DeviceRegistry
{
public:
    static DeviceRegistry<BaseDevice>& I();

    DeviceRegistry();
    ~DeviceRegistry();

    // Register factory pointer accessible by device_name
    void RegisterFactory(
            const std::string& device_name,
            DeviceFactory<BaseDevice>* factory
            );

    // Register device 'name' as alias for 'alias'
    void RegisterAlias(
            const std::string& name,
            const std::string& alias
            );

    //std::shared_ptr<BaseDevice> Create(const Uri& uri);
    std::shared_ptr<BaseDevice> Create(const Uri& uri, const char* sDeviceType=NULL);

    void Destroy(BaseDevice* dev);

protected:
    struct DevInstance
    {
        Uri uri;
        std::shared_ptr<BaseDevice> dev;
    };

    // Map of device names to aliases.
    std::map<std::string,std::string> m_aliases;

    // Map of device name -> factory instance
    std::map<std::string,DeviceFactory<BaseDevice>*> m_factories;

//    // Vector of running instances
//    std::vector<DevInstance> m_instances;
};

}
