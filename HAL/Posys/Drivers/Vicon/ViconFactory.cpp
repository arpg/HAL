#include <HAL/Devices/DeviceFactory.h>

#include "ViconDriver.h"

namespace hal
{

class ViconFactory : public DeviceFactory<PosysDriverInterface>
{
public:
    ViconFactory(const std::string& name)
        : DeviceFactory<PosysDriverInterface>(name)
    {
        Params() = {
        };
    }

    PosysDriverInterface* GetDevice(const Uri& uri)
    {
        std::vector<std::string> Splitter = Split(uri.url, ':');

        if( Splitter.size() != 2 ) {
            return nullptr;
        }

        ViconDriver* pDriver = new ViconDriver( Splitter[0], Splitter[1] );
        return static_cast<PosysDriverInterface*>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static ViconFactory g_ViconFactory("vicon");


// format: vicon://<HOST>:[<Tracked Objects>]
// example: vicon://10.0.0.1:[Car,Boat]

}
