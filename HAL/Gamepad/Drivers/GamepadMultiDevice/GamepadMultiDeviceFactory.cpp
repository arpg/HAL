#include <HAL/Devices/DeviceFactory.h>

#include "GamepadMultiDeviceDriver.h"

namespace hal
{

class GamepadMultiDeviceFactory : public DeviceFactory<GamepadDriverInterface>
{
public:
    GamepadMultiDeviceFactory(const std::string& name)
        : DeviceFactory<GamepadDriverInterface>(name)
    {
        Params() = {
        };
    }

    std::shared_ptr<GamepadDriverInterface> GetDevice(const Uri& /*uri*/)
    {
        GamepadMultiDeviceDriver* pDriver =
            new GamepadMultiDeviceDriver();
        return std::shared_ptr<GamepadDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static GamepadMultiDeviceFactory g_GamepadMultiDeviceFactory("gamepad");

}
