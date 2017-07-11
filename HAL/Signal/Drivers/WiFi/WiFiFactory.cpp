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
            {"scanperiod", "1", "Seconds between scans."}
        };
    }

    std::shared_ptr<SignalDriverInterface> GetDevice(const Uri& uri)
    {
        int scan_period = uri.properties.Get("scanperiod", 1);

        WiFiDriver* pDriver =
            new WiFiDriver(uri.url, scan_period);
        return std::shared_ptr<SignalDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static WiFiFactory g_WiFiFactory("wifi");

}
