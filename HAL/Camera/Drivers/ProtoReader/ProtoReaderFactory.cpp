#include <HAL/Devices/DriverFactory.h>
#include "ProtoReaderDriver.h"

namespace hal
{

class ProtoReaderFactory : public DriverFactory<CameraDriverInterface>
{
public:
    ProtoReaderFactory(const std::string& name)
        : DriverFactory<CameraDriverInterface>(name) {}

    std::shared_ptr<CameraDriverInterface> CreateDriver(const Uri& uri)
    {
        ProtoReaderDriver* driver =
            new ProtoReaderDriver( uri );
        return std::shared_ptr<CameraDriverInterface>( driver );
    }
};

// Register this factory by creating static instance of factory
static ProtoReaderFactory g_ProtoReaderFactory1("proto");
static ProtoReaderFactory g_ProtoReaderFactory2("log");

}
