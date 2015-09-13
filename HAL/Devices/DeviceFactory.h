#pragma once

#include <HAL/Devices/DeviceRegistry.h>

#include <memory>
#include <vector>

namespace hal
{

template<typename BaseDevice>
class DeviceFactory
{
public:        
    struct Param
    {
        std::string param;
        std::string defaultval;
        std::string help;
    };
    
    DeviceFactory(std::string name)
        : name_(name)
    {
        Registry().RegisterFactory(name_, this);
    }
    
    virtual ~DeviceFactory() {}
    virtual std::shared_ptr<BaseDevice> CreateDriver(const Uri& uri) = 0;
 
    virtual std::vector<std::string> AvailableDevices() {
        return std::vector<std::string>();
    }
 
    inline std::vector<Param>& Params() {
        return params_;
    }
 
    inline static hal::DeviceRegistry<BaseDevice>& Registry() {
        return hal::DeviceRegistry<BaseDevice>::Instance();
    }    
 
protected:
    std::string name_;
    std::vector<Param> params_;
};

}
