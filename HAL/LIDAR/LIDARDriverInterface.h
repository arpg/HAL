#pragma once

#include <functional>

#include <HAL/Devices/DriverInterface.h>

#include <HAL/Lidar.pb.h>


namespace hal {

typedef std::function<void (hal::LidarMsg&)> LIDARDriverDataCallback;

///////////////////////////////////////////////////////////////////////////////
/// Generic LIDAR driver interface
class LIDARDriverInterface : public DriverInterface
{
    public:
        // Pure virtual functions driver writers must implement:
        virtual ~LIDARDriverInterface() {}
        virtual void RegisterLIDARDataCallback(LIDARDriverDataCallback callback) = 0;
};

} /* namespace */
