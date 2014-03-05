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
        {"portname", "/dev/pcan32", "Port name such as /dev/pcan32"},
        {"baudrate", "500000", "CAN Bus baudrate as 125000 or 500000 or1000000"}
        };
    }

    std::shared_ptr<EncoderDriverInterface> GetDevice(const Uri& uri)
    {
        std::string portname  = uri.properties.Get("portname", std::string("/dev/pcan32"));
        int baudrate = uri.properties.Get("baudrate", 500000);
        PCANEncoderDriver* pDriver = new PCANEncoderDriver( baudrate, portname );

        return std::shared_ptr<EncoderDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static PCANEncoderFactory g_PCANEncoderFactory("pcan");

}



