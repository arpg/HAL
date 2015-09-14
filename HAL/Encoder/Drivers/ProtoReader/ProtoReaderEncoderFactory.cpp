#include <HAL/Devices/DriverFactory.h>
#include "ProtoReaderEncoderDriver.h"

namespace hal
{

class ProtoReaderEncoderFactory : public DriverFactory<EncoderDriverInterface>
{
public:
    ProtoReaderEncoderFactory(const std::string& name)
        : DriverFactory<EncoderDriverInterface>(name) {}

    std::shared_ptr<EncoderDriverInterface> CreateDriver(const Uri& uri)
    {
        const std::string file = ExpandTildePath(uri.url);

        ProtoReaderEncoderDriver* pDriver = new ProtoReaderEncoderDriver(file);
        return std::shared_ptr<EncoderDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static ProtoReaderEncoderFactory g_ProtoReaderEncoderFactory1("proto");
static ProtoReaderEncoderFactory g_ProtoReaderEncoderFactory2("log");

}
