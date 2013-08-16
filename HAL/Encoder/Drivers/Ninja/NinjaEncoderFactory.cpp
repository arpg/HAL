#include <HAL/Devices/DeviceFactory.h>

#include "NinjaEncoderDriver.h"

namespace hal
{

class NinjaEncoderFactory : public DeviceFactory<EncoderDriverInterface>
{
public:
    NinjaEncoderFactory(const std::string& name)
        : DeviceFactory<EncoderDriverInterface>(name)
    {
        Params() = {
        };
    }

    std::shared_ptr<EncoderDriverInterface> GetDevice(const Uri& uri)
    {
        NinjaEncoderDriver* pDriver = new NinjaEncoderDriver( uri.url );
        return std::shared_ptr<EncoderDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static NinjaEncoderFactory g_NinjaEncoderFactory("ninja");

}
