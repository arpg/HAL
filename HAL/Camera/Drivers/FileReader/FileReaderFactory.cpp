#include <HAL/Devices/DeviceFactory.h>
#include "FileReaderDriver.h"

namespace hal
{

class FileReaderFactory : public DeviceFactory<CameraDriverInterface>
{
public:
    FileReaderFactory(const std::string& name)
        : DeviceFactory<CameraDriverInterface>(name)
    {
        std::cout << "+FileReaderFactory" << std::endl;
        Params() = {
            {"dir", "", "Prefix filename for channels."},
            {"startframe", "0", "First frame to capture."},
            {"loop", "false", "Play beginning once finished."},
            {"grey", "false", "Convert internally to greyscale."},
            {"buffer", "10", "Number of frames to cache in memory"},
        };
    }

    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
    {
        std::string Dir =   uri.properties.Get<std::string>("dir", "");
        size_t StartFrame = uri.properties.Get("startframe", 0);
        bool Loop =         uri.properties.Get("loop", false);
        size_t BufferSize = uri.properties.Get("buffer", 10);
        bool Grey =         uri.properties.Get("grey", false);
        int cvFlags = Grey ? 0 : -1;
        
        std::vector<std::string> ChannelRegex = split(uri.url,',');
        for(std::string& s : ChannelRegex) {
            s = Dir + s;
        }        

        FileReaderDriver* filereader = new FileReaderDriver(
                    ChannelRegex, StartFrame, Loop, BufferSize, cvFlags
                    );
        return std::shared_ptr<CameraDriverInterface>( filereader );
    }
};

// Register this factory by creating static instance of factory
static FileReaderFactory g_FileReaderFactory("file");

}
