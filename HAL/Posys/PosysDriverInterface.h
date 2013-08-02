#pragma once

#include <functional>

#include <HAL/Devices/DriverInterface.h>

#include <PbMsgs/Pose.pb.h>


namespace hal {

typedef std::function<void (pb::PoseMsg&)> PosysDriverDataCallback;

///////////////////////////////////////////////////////////////////////////////
/// Generic Posys Driver Interface
class PosysDriverInterface : public DriverInterface
{
    public:
        // Pure virtual functions driver writers must implement:
        virtual ~PosysDriverInterface() {}
        virtual void RegisterPosysDataCallback(PosysDriverDataCallback callback) = 0;
};

} /* namespace */
