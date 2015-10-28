#include <HAL/Devices/DriverFactory.h>
#include "NodeCamDriver.h"

namespace hal
{

CREATE_DRIVER_FACTORY_CLASS(NodeCam) // Create NodeCam class


// Register this factory by creating static instance of factory
static NodeCamFactory g_NodeCamFactory("node", { {} });

}
