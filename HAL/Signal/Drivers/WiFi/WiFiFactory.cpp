#include <HAL/Devices/DeviceFactory.h>

#include "WiFiDriver.h"

namespace hal
{

class WiFiFactory : public DeviceFactory<SignalDriverInterface>
{
public:
    WiFiFactory(const std::string& name)
        : DeviceFactory<SignalDriverInterface>(name)
    {
        Params() = {
            {"scanhz", "1", "IMU capture rate in Hz."}
        };
    }

    std::shared_ptr<SignalDriverInterface> GetDevice(const Uri& uri)
    {
        float scan_hz = uri.properties.Get("scanhz", 1);

        WiFiDriver* pDriver =
            new WiFiDriver(uri.url, scan_hz);
        return std::shared_ptr<SignalDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static WiFiFactory g_WiFiFactory("wifi");

}
