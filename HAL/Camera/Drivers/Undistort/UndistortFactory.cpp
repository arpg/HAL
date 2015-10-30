#include <HAL/Devices/DeviceFactory.h>
#include "UndistortDriver.h"

#include <calibu/cam/camera_xml.h>

namespace hal
{

class UndistortFactory : public DeviceFactory<CameraDriverInterface>
{
public:
    UndistortFactory(const std::string& name)
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

        UndistortDriver* pDriver = new UndistortDriver( input, rig );
        return std::shared_ptr<CameraDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static UndistortFactory g_UndistortFactory("undistort");

}
