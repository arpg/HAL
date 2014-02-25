#include <HAL/Devices/DeviceFactory.h>

#include "PCANEncoderDriver.h"

namespace hal
{

class PCANEncoderFactory : public DeviceFactory<EncoderDriverInterface>
{
public:
    PCANEncoderFactory(const std::string& name)
        : DeviceFactory<EncoderDriverInterface>(name)
    {
        Params() = {
        };
    }

    std::shared_ptr<EncoderDriverInterface> GetDevice(const Uri& uri)
    {
        PCANEncoderDriver* pDriver = new PCANEncoderDriver( uri.url );
        return std::shared_ptr<EncoderDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static PCANEncoderFactory g_PCANEncoderFactory("PCAN");

}
