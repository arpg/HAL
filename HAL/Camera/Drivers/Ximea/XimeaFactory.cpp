#include <HAL/Devices/DriverFactory.h>

#include "XimeaDriver.h"


namespace hal
{

	CREATE_DRIVER_FACTORY_CLASS(Ximea) // Create XimeaFactory class

		// Register this factory by creating static instance of factory.
		static XimeaFactory g_XimeaFactory("ximea",
				{ 
				{"idN", "0", "Camera serial number. Increment N from 0 to (num serial numbers -1)"},
				{"fps", "DEFAULT", "Capture framerate: [1, 150]"},
				{"exp", "0", "Exposure time (microseconds): [1,1000000] (0: AUTO)"},
				{"gain", "0.5", "Gain (dB): [-2.4, 12.0]"},
				{"mode", "MONO8", "Video mode: RAW8, RAW16, MONO8, MONO16, RGB24, RGB32"},
				{"size", "640x480", "Capture resolution."},
				{"roi", "0+0+0x0", "ROI resolution."},
				{"sync", "0", "Sync type. [0: none, 1: software, 2: hardware]"},
				{"binning", "0", "Binning: Divide frame by this integer in hardware"},
        {"bus_cams", "0", "Divide the total bus bandwidth by this number per camera"}
				} );

} // namespace hal

