#include <HAL/Devices/DriverFactory.h>
#include "UndistortDriver.h"

#include <calibu/cam/camera_xml.h>

namespace hal
{

class UndistortFactory : public DriverFactory<CameraDriverInterface>
{
public:
    UndistortFactory(const std::string& name)
        : DriverFactory<CameraDriverInterface>(name) {}

    std::shared_ptr<CameraDriverInterface> CreateDriver(const Uri& uri)
    {
        // Create input camera
        const Uri input_uri = Uri(uri.url);
        std::shared_ptr<CameraDriverInterface> input =
                DeviceDriverRegistry<hal::CameraDriverInterface>::Instance().Create(input_uri);
        UndistortDriver* pDriver = new UndistortDriver( input, uri );
        return std::shared_ptr<CameraDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static UndistortFactory g_UndistortFactory("undistort");

}
