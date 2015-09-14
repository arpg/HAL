#include <HAL/Devices/DriverFactory.h>
#include "ProtoReaderLIDARDriver.h"

namespace hal
{

class ProtoReaderLIDARFactory : public DriverFactory<LIDARDriverInterface>
{
public:
    ProtoReaderLIDARFactory(const std::string& name)
        : DriverFactory<LIDARDriverInterface>(name) {}

    std::shared_ptr<LIDARDriverInterface> CreateDriver(const Uri& uri)
    {
        const std::string file = ExpandTildePath(uri.url);

        ProtoReaderLIDARDriver* pDriver = new ProtoReaderLIDARDriver(file);
        return std::shared_ptr<LIDARDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static ProtoReaderLIDARFactory g_ProtoReaderLIDARFactory1("proto");
static ProtoReaderLIDARFactory g_ProtoReaderLIDARFactory2("log");

}
