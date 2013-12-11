#include <HAL/Devices/DeviceFactory.h>
#include "ProtoReaderEncoderDriver.h"

namespace hal
{

class ProtoReaderEncoderFactory : public DeviceFactory<EncoderDriverInterface>
{
public:
    ProtoReaderEncoderFactory(const std::string& name)
        : DeviceFactory<EncoderDriverInterface>(name)
    {
        Params() = {
        };
    }

    std::shared_ptr<EncoderDriverInterface> GetDevice(const Uri& uri)
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
