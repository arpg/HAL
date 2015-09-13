#pragma once

#include <string>

namespace hal {


// Generic device driver interface
class DriverInterface
{
public:
    // Get device driver specific properties, such as base directory for a file reader
    inline virtual std::string GetProperty(const std::string& /*sProperty*/)
    {
        return std::string();
    }
};

}
