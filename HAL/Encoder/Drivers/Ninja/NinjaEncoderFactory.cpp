#include <HAL/Devices/DriverFactory.h>

#include "NinjaEncoderDriver.h"

namespace hal
{

class NinjaEncoderFactory : public DriverFactory<EncoderDriverInterface>
{
public:
    NinjaEncoderFactory(const std::string& name)
        : DriverFactory<EncoderDriverInterface>(name)
    {
        Params() = {
        };
    }

    std::shared_ptr<EncoderDriverInterface> CreateDriver(const Uri& uri)
    {
        NinjaEncoderDriver* pDriver = new NinjaEncoderDriver( uri.url );
        return std::shared_ptr<EncoderDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static NinjaEncoderFactory g_NinjaEncoderFactory("ninja");

}
