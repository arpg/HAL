#include <HAL/Devices/DriverFactory.h>
#include "FlycapDriver.h"

namespace hal
{

	CREATE_DRIVER_FACTORY_CLASS(Flycap) // Create FlycapFactory class

	// Register this factory by creating static instance with all allowable options:
	static FlycapFactory g_FlycapFactory("flycap",
			{
			{"ids","0","ids=<id;id;...;id> List of camera serial numbers for cameras on bus."},
			{"mode","FORMAT7_0","Video mode: FORMAT7_0,FORMAT7_1,FORMAT7_2 etc"},
			{"format","RGB8","pixel format"},
			{"size", "1920x1200", "Capture resolution."},
			{"roi", "0+0+0x0", "ROI resolution for Format7."},
			{"shutter","auto","shutter setting: auto, man, abs"},
			{"gain","auto","gain setting: auto, man, abs"},
			{"exposure","auto","exposure setting: auto, man, abs"},
			{"brightness","auto","brightness setting: auto, man, abs"},
      {"debayer_method", "bilinear", "debayer method: none, nearest, simple"
			                                 ", downsample, bilinear,hqlinear, vng, ahd"},
      {"debayer_filter", "rggb", "debayer filter: rggb, gbrg, grbg"}
			}
			);
}

