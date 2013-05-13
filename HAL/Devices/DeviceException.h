#pragma once

#include <exception>
#include <string>

namespace hal {

struct DeviceException : std::exception
{
    DeviceException(std::string str) : desc(str) {}
    DeviceException(std::string str, std::string detail) {
        desc = str + "\n\t" + detail;
    }
    ~DeviceException() throw() {}
    const char* what() const throw() { return desc.c_str(); }
    std::string desc;
};

}
