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
    struct DevInstance
    {
        Uri uri;
        std::shared_ptr<BaseDevice> dev;
    };
    
    DeviceRegistry() = default;
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
        std::cout << "RegisterFactory " << device_name << ", " << factory << std::endl;
        m_factories[device_name] = factory;
    }
    
    std::shared_ptr<BaseDevice> Create(const Uri& uri)
    {
        auto pf = m_factories.find(uri.scheme);
        if(pf != m_factories.end()) {
            std::shared_ptr<BaseDevice> dev = pf->second->GetDevice(uri);
            m_instances[uri.scheme].insert( dev );
            return dev;
        }else{
            throw VideoException("Scheme '" + uri.scheme + "' not registered for factory");
        }
    }
    
    static void Destroy(BaseDevice* dev)
    {
        auto i = std::find(m_instances.begin(), m_instances.end(), dev);
        
        if(i != m_instances.end()) {
            m_instances.erase(i);
        }
    }
    
protected:
    // Map of device name -> factory instance
    std::map<std::string,DeviceFactory<BaseDevice>*> m_factories;
    
    // Vector of running instances
    std::vector<DevInstance> m_instances;
};

}
