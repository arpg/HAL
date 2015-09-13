#include <HAL/Devices/DeviceFactory.h>
#include "SplitDriver.h"

#include <unistd.h>

namespace hal
{

class SplitFactory : public DeviceFactory<CameraDriverInterface>
{
public:
    SplitFactory(const std::string& name)
        : DeviceFactory<CameraDriverInterface>(name)
    {
        Params() = {
            {"roiN", "0+0+WidthxHeight", "Nth ROI."},
        };
    }

    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
    {
        hal::Uri subUri(uri.url);

        // pass through properties down the chain
        subUri.SetProperties( uri.PrintProperties() );

        // Create input camera
        std::shared_ptr<CameraDriverInterface> InCam =
                DeviceRegistry<hal::CameraDriverInterface>::Instance().Create(subUri);

        std::vector<ImageRoi> vROI;

        const ImageRoi default_roi( 0, 0, InCam->Width(), InCam->Height() );

        if( !uri.scheme.compare("split") ) {
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

            if( vROI.empty() ) {
              unsigned int nImgWidth = InCam->Width();
              unsigned int nImgHeight = InCam->Height();
              if( nImgWidth > nImgHeight ) {
                  nImgWidth = nImgWidth / 2;
              } else {
                  nImgHeight = nImgHeight / 2;
              }

              vROI.push_back( ImageRoi( 0, 0, nImgWidth, nImgHeight ) );
              vROI.push_back( ImageRoi( nImgWidth, 0, nImgWidth, nImgHeight ) );
            }
        }

        SplitDriver* pDriver = new SplitDriver( InCam, vROI );
        return std::shared_ptr<CameraDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static SplitFactory g_SplitFactory("split");

}
