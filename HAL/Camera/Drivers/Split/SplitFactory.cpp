#include <HAL/Devices/DeviceFactory.h>
#include "SplitDriver.h"

namespace hal
{

class SplitFactory : public DeviceFactory<CameraDriverInterface>
{
public:
    SplitFactory(const std::string& name)
        : DeviceFactory<CameraDriverInterface>(name)
    {
        Params() = {
            {"roiN", "0+0+1x1", "Nth ROI."},
            {"copy", "false", "Do a deep copy of data instead of moving pointers."}
        };
    }

    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
    {
        // Create input camera
        std::shared_ptr<CameraDriverInterface> InCam =
                DeviceRegistry<hal::CameraDriverInterface>::I().Create(uri.url);

        std::vector<ImageRoi> vROI;

        const ImageRoi default_roi(0,0,1,1);

        while(true)
        {
            std::stringstream ss;
            ss << "roi" << (vROI.size() + 1);
            const std::string key = ss.str();

            if(!uri.properties.Contains(key)) {
                break;
            }

            vROI.push_back( uri.properties.Get<ImageRoi>( key, default_roi ) );
        }

        bool bCopy = uri.properties.Get("copy", false);


        SplitDriver* pDriver = new SplitDriver( InCam, vROI, bCopy );
        return std::shared_ptr<CameraDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static SplitFactory g_SplitFactory("split");

}
