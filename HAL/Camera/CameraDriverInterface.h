#pragma once

#include <PbMsgs/Camera.pb.h>
#include <HAL/Devices/DriverInterface.h>

namespace hal {

///////////////////////////////////////////////////////////////////////////////
// Generic camera driver interface
class CameraDriverInterface : public DriverInterface
{
public:
    // Pure virtual functions driver writers must implement:
    virtual ~CameraDriverInterface() {}
    virtual bool Capture( pb::CameraMsg& vImages ) = 0;    
};

}
