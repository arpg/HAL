#include <HAL/Devices/DriverFactory.h>
#include "OpenCVDriver.h"

namespace hal {

CREATE_DRIVER_FACTORY_CLASS(OpenCV) // Create FileReaderFactory class

// Register this factory by creating static instance of factory
static OpenCVFactory g_OpenCVFactory("opencv",
{
{"grey","false","Greyscale boolean control."},
{"idN", "0", "Camera serial number. Increment N from 0 to (num serial numbers -1)"}
} );

}  // namespace hal
