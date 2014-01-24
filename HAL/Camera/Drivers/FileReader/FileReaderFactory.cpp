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
        Params() = {
            {"id","0","Camera id."},
            {"name", "FileCam", "Camera name"},
            {"startframe", "0", "First frame to capture."},
            {"loop", "false", "Play beginning once finished."},
            {"grey", "false", "Convert internally to greyscale."},
            {"buffer", "10", "Number of frames to cache in memory"},
            {"frequency", "30", "Capture frequency to emulate if needed"}
        };
    }

    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
    {
        unsigned int nCamId     = uri.properties.Get<unsigned int>("id", 0);
        std::string sName       = uri.properties.Get<std::string>("name", "FileCam");
        size_t StartFrame  = uri.properties.Get("startframe", 0);
        bool Loop          = uri.properties.Get("loop", false);
        size_t BufferSize  = uri.properties.Get("buffer", 10);
        bool Grey          = uri.properties.Get("grey", false);
//        double Frequency  = uri.properties.Get("frequency", 30.0); // default if not in files
        int cvFlags = Grey ? 0 : -1;

        std::vector<std::string> Channels = Expand(uri.url, '[', ']', ',');
        
        for(std::string& s : Channels) {
            s = ExpandTildePath(s);
        }

        FileReaderDriver* filereader = new FileReaderDriver(
                    Channels, StartFrame, Loop, BufferSize, cvFlags
                    );
        return std::shared_ptr<CameraDriverInterface>( filereader );
    }    
};

// Register this factory by creating static instance of factory
static FileReaderFactory g_FileReaderFactory("file");
static FileReaderFactory g_FileReaderFactoryS("files");

}
