#pragma once

#include <HAL/Devices/SharedLoad.h>
#include <HAL/Utils/Uri.h>

#include <memory>
#include <map>

#ifndef __GXX_EXPERIMENTAL_CXX0X__
    #error "C++11 is required to use HAL. Please enable -std=c++11 flag on compiler options."
#endif

namespace hal
{

// Forward declaration
template<typename BaseDevice> class DeviceFactory;

template<typename BaseDevice>
class DeviceRegistry
{
public:
    struct DevInstance
    {
        Uri uri;
        std::shared_ptr<BaseDevice> dev;
    };

    DeviceRegistry() {
        RegisterAlias( "bumblebee", "debayer://dc1394:[]://" );
        RegisterAlias( "twizzler",  "deinterlace://v4l://" );
    }

    ~DeviceRegistry() = default;

    inline static DeviceRegistry<BaseDevice>& I()
    {
        static DeviceRegistry<BaseDevice> s_instance;
        return s_instance;
    }

    // Register factory pointer.
    void RegisterFactory(
            const std::string& device_name,
            DeviceFactory<BaseDevice>* factory
            )
    {
        m_factories[device_name] = factory;
    }
    
    void RegisterAlias(
            const std::string& name,
            const std::string& alias
            )
    {
        m_aliases[name] = alias;
    }

    std::shared_ptr<BaseDevice> Create(const Uri& uri)
    {
        // Check for aliases
        std::map<std::string,std::string>::const_iterator iAlias=
                m_aliases.find(uri.scheme);
        
        if(iAlias != m_aliases.end()) {
            std::string sAlias = iAlias->second;
            std::ostringstream oss;
            oss << sAlias << uri.url;
            return Create(oss.str());
        }else{
            auto pf = m_factories.find(uri.scheme);
            if(pf != m_factories.end()) {
                std::shared_ptr<BaseDevice> dev = pf->second->GetDevice(uri);
//                m_instances[uri.scheme].insert( dev );
                return dev;
            }else{
                throw DeviceException("Scheme '" + uri.scheme + "' not registered for factory");
            }
        }
    }

    void Destroy(BaseDevice* dev)
    {
//        auto i = std::find(m_instances.begin(), m_instances.end(), dev);

//        if(i != m_instances.end()) {
//            m_instances.erase(i);
//        }
    }

protected:
    // Map of device names to aliases.
    std::map<std::string,std::string> m_aliases;
    
    // Map of device name -> factory instance
    std::map<std::string,DeviceFactory<BaseDevice>*> m_factories;

//    // Vector of running instances
//    std::vector<DevInstance> m_instances;
};

}
