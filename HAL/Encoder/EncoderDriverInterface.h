#pragma once

#include <functional>

#include <HAL/Devices/DriverInterface.h>

#include <HAL/Encoder.pb.h>


namespace hal {

typedef std::function<void (hal::EncoderMsg&)> EncoderDriverDataCallback;

///////////////////////////////////////////////////////////////////////////////
/// Generic Encoder driver interface
class EncoderDriverInterface : public DriverInterface
{
    public:
        // Pure virtual functions driver writers must implement:
        virtual ~EncoderDriverInterface() {}
        virtual void RegisterEncoderDataCallback(EncoderDriverDataCallback callback) = 0;
};

} /* namespace */
