#include <HAL/Devices/DeviceFactory.h>
#include "ProtoReaderDriver.h"

namespace hal
{

class ProtoReaderFactory : public DeviceFactory<CameraDriverInterface>
{
public:
    ProtoReaderFactory(const std::string& name)
        : DeviceFactory<CameraDriverInterface>(name)
    {
        Params() = {
        };
    }
        
    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
    {               
        ProtoReaderDriver* driver = new ProtoReaderDriver(uri.url);
        return std::shared_ptr<CameraDriverInterface>( driver );
    }
};

// Register this factory by creating static instance of factory
static ProtoReaderFactory g_ProtoReaderFactory1("proto");
static ProtoReaderFactory g_ProtoReaderFactory2("log");

}
