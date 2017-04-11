#include <HAL/Devices/DeviceFactory.h>
#include "ProtoReaderDriver.h"

namespace hal
{

class ProtoReaderFactory : public DeviceFactory<CameraDriverInterface>
{
public:
    ProtoReaderFactory(const std::string& name)
        : DeviceFactory<CameraDriverInterface>(name)
    {
        Params() = {
            {"startframe", "0", "First frame to capture."},
            {"id", "0", "Id of the camera in log."},
            {"realtime", "0", "If the data should be played back at framerate"}
        };
    }

    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
    {
        const std::string file = ExpandTildePath(uri.url);
        size_t startframe  = uri.properties.Get("startframe", 0);
        int camId = uri.properties.Get("id", -1);
        bool realtime = uri.properties.Get("realtime", 0);

        ProtoReaderDriver* driver =
            new ProtoReaderDriver(file, camId, startframe, realtime);
        return std::shared_ptr<CameraDriverInterface>( driver );
    }
};

// Register this factory by creating static instance of factory
static ProtoReaderFactory g_ProtoReaderFactory1("proto");
static ProtoReaderFactory g_ProtoReaderFactory2("log");

}
