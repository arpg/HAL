#pragma once

#include <string>

namespace hal {

const std::string DeviceDirectory           = "Directory";
const std::string DeviceDepthFocalLength    = "DepthFocalLength";
const std::string DeviceDepthBaseline       = "DepthBaseline";

///////////////////////////////////////////////////////////////////////////////
// Generic device driver interface
class DriverInterface
{
public:
    // Get device specific properties, such as base directory for a file reader
    inline virtual std::string GetDeviceProperty(const std::string& /*sProperty*/)
    {
        return std::string();
    }
};

}
