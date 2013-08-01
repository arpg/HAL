#pragma once

#include <functional>

#include <HAL/Devices/DriverInterface.h>

#include <PbMsgs/Imu.pb.h>


namespace hal {

typedef std::function<void (pb::ImuMsg&)> IMUDriverDataCallback;

///////////////////////////////////////////////////////////////////////////////
/// Generic IMU driver interface
class IMUDriverInterface : public DriverInterface
{
    public:
        // Pure virtual functions driver writers must implement:
        virtual ~IMUDriverInterface() {}
        virtual void RegisterIMUDataCallback(IMUDriverDataCallback callback) = 0;
};

} /* namespace */
