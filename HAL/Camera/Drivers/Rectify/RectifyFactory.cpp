#include <HAL/Devices/DeviceFactory.h>
#include "RectifyDriver.h"

#include <calibu/cam/camera_xml.h>

namespace hal
{

class RectifyFactory : public DeviceFactory<CameraDriverInterface>
{
public:
    RectifyFactory(const std::string& name)
        : DeviceFactory<CameraDriverInterface>(name)
    {
        Params() = {
            {"file","","Cameras XML description"}
        };
    }
        
    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
    {
        const Uri input_uri = Uri(uri.url);
        
        // Create input camera
        std::shared_ptr<CameraDriverInterface> input =
                DeviceRegistry<hal::CameraDriverInterface>::Instance().Create(input_uri);
 
        std::string filename = ExpandTildePath(
                    uri.properties.Get<std::string>("file", "cameras.xml")
                    );
 
        if(!FileExists(filename))
        {
            std::string dir = input->GetDeviceProperty(hal::DeviceDirectory);
            while(!dir.empty() && !FileExists(dir+"/"+filename)) {
                dir = DirUp(dir);
            }
            filename = (dir.empty() ? "" : dir + "/") + filename;
        }

        std::shared_ptr<calibu::Rig<double>> rig = calibu::ReadXmlRig( filename );
        if(rig->NumCams() != 2) {
            throw DeviceException("Unable to find 2 cameras in file '" + filename + "'");
        }

        RectifyDriver* rectify = new RectifyDriver( input, rig );
        return std::shared_ptr<CameraDriverInterface>( rectify );
    }
};

// Register this factory by creating static instance of factory
static RectifyFactory g_RectifyFactory("rectify");

}
