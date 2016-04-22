#include <HAL/Devices/DriverFactory.h>
#include "JoinCameraDriver.h"

namespace hal
{
   CREATE_DRIVER_FACTORY_CLASS(Join) // Create JoinFactory class

   // Register this factory by creating static instance of factory

   static JoinFactory g_JoinFactory("join",
   {
   {"id", "0", "Camera id (serial number or UUID)."}
   } );


}
