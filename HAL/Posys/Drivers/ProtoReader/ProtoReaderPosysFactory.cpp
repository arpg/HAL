#include <HAL/Devices/DeviceFactory.h>

#include "ProtoReaderPosysDriver.h"

namespace hal
{

class ProtoReaderPosysFactory : public DeviceFactory<PosysDriverInterface>
{
public:
    ProtoReaderPosysFactory(const std::string& name)
        : DeviceFactory<PosysDriverInterface>(name)
    {
        Params() = {
        };
    }

    std::shared_ptr<PosysDriverInterface> GetDevice(const Uri& uri)
    {
        ProtoReaderPosysDriver* pDriver = new ProtoReaderPosysDriver(uri.url);
        return std::shared_ptr<PosysDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static ProtoReaderPosysFactory g_ProtoReaderPosysFactory1("proto");
static ProtoReaderPosysFactory g_ProtoReaderPosysFactory2("log");

}
