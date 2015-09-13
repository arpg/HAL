#pragma once

#include <HAL/config.h>
#include <HAL/Utils/Uri.h>

#include <memory>
#include <map>

namespace hal
{

// Forward declaration
template<typename BaseDevice> class DeviceFactory;

template<typename BaseDevice>
class DeviceRegistry
{
public:
    /// global device registery singleton 
    static DeviceRegistry<BaseDevice>& Instance();

    DeviceRegistry();
    ~DeviceRegistry();

    /// Register factory pointer accessible by device_name
    void RegisterFactory(
            const std::string& device_name,
            DeviceFactory<BaseDevice>* factory
            );

    // Register device 'name' as alias for 'alias'
    void RegisterAlias(
            const std::string& name,
            const std::string& alias
            );

    // Get factory associated with uri
    std::shared_ptr<BaseDevice> Create(const Uri& uri);

    void Destroy(BaseDevice* dev);

    // print the map<string,Dec> table 
    void PrintRegisteredDevices();

protected:

    // Map of device names to aliases.
    std::map<std::string,std::string> m_aliases;

    // Map of device name -> factory instance
    std::map<std::string,DeviceFactory<BaseDevice>*> m_factories;

/*
    struct DevInstance
    {
        Uri uri;
        std::shared_ptr<BaseDevice> dev;
    };
*/
//    // Vector of running instances
//    std::vector<DevInstance> m_instances;
};

}
