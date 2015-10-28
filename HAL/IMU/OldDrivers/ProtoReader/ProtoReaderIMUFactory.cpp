#include <HAL/Devices/DriverFactory.h>
#include "ProtoReaderIMUDriver.h"

namespace hal
{

class ProtoReaderIMUFactory : public DriverFactory<IMUDriverInterface>
{
public:
    ProtoReaderIMUFactory(const std::string& name)
        : DriverFactory<IMUDriverInterface>(name) {}

    std::shared_ptr<IMUDriverInterface> CreateDriver(const Uri& uri)
    {
      const std::string file = ExpandTildePath(uri.url);
      ProtoReaderIMUDriver* pDriver = new ProtoReaderIMUDriver(file);
      return std::shared_ptr<IMUDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static ProtoReaderIMUFactory g_ProtoReaderIMUFactory1("proto");
static ProtoReaderIMUFactory g_ProtoReaderIMUFactory2("log");

}
