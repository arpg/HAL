#include <cstdlib>
#include <climits>

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
            {"channels", "", "List of channels to capture."}
        };
    }

    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
    {
        const std::string file = ExpandTildePath(uri.url);
        size_t startframe  = uri.properties.Get("startframe", 0);
        int camId = uri.properties.Get("id", -1);
        std::string sChannels = uri.properties.Get<std::string>("channels", "");
        std::vector<int> channels = parseChannels(sChannels);

        ProtoReaderDriver* driver =
            new ProtoReaderDriver(file, camId, startframe, channels);
        return std::shared_ptr<CameraDriverInterface>( driver );
    }

    std::vector<int> parseChannels(const std::string& sChannels) const
    {
      std::vector<int> ret;
      for (const char *cur = sChannels.data(); *cur != '\0'; ) {
        char *next;
        long int value = strtol(cur, &next, 10);
        if (next == cur)
          ++cur;
        else {
          cur = next;
          if (value != LONG_MIN && value != LONG_MAX && value >= 0)
            ret.push_back(value);
        }
      }
      return ret;
    }
};

// Register this factory by creating static instance of factory
static ProtoReaderFactory g_ProtoReaderFactory1("proto");
static ProtoReaderFactory g_ProtoReaderFactory2("log");

}
