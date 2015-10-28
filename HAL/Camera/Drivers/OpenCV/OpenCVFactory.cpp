#include <HAL/Devices/DriverFactory.h>
#include "OpenCVDriver.h"

namespace hal {

CREATE_DRIVER_FACTORY_CLASS(OpenCV) // Create FileReaderFactory class

// Register this factory by creating static instance of factory
static OpenCVFactory g_OpenCVFactory("opencv",
{{"","",""}}
 );
}  // namespace hal
