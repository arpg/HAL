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
        std::string help;
    };
    
    DeviceFactory(std::string name)
        : m_name(name)
    {
        hal::DeviceRegistry<BaseDevice>::I().RegisterFactory(m_name, this);
    }
    
    virtual ~DeviceFactory() {}
    virtual std::shared_ptr<BaseDevice> GetDevice(const Uri& uri) = 0;
    
    virtual std::vector<std::string> AvailableDevices() {
        return std::vector<std::string>();
    }
    
    std::vector<Param>& Params() {
        return m_params;
    }
    
protected:
    std::string m_name;
    std::vector<Param> m_params;
};

}
