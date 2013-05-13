#pragma once

#include <exception>
#include <string>

namespace hal {

struct CameraException : std::exception
{
    CameraException(std::string str) : desc(str) {}
    CameraException(std::string str, std::string detail) {
        desc = str + "\n\t" + detail;
    }
    ~CameraException() throw() {}
    const char* what() const throw() { return desc.c_str(); }
    std::string desc;
};

}
