#include <HAL/Devices/DriverFactory.h>
#include "JoinCameraDriver.h"

namespace hal
{
   CREATE_DRIVER_FACTORY_CLASS(JoinCamera) // Create JoinFactory class

   // Register this factory by creating static instance of factory

   static JoinCameraFactory g_JoinCameraFactory("joincamera",
   {
   {"id", "0", "Camera id (serial number or UUID)."}
   } );


}
