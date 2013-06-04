#include <HAL/Devices/DeviceFactory.h>
#include "DC1394Driver.h"


namespace hal
{

class DC1394Factory : public DeviceFactory<CameraDriverInterface>
{
public:
    DC1394Factory(const std::string& name)
        : DeviceFactory<CameraDriverInterface>(name)
    {
        Params() = {
            {"id","0","Camera id."},
            {"mode","MONO8","Video mode: RGB8, MONO8, MONO16, FORMAT7_X"},
            {"size", "640x480", "Capture resolution."},
            {"roi", "0+0+640x480", "ROI resolution for Format7."},
            {"fps", "30.0", "Capture framerate."},
            {"iso", "400", "ISO speed."},
            {"dma", "4", "Number of DMA channels."}
        };
    }

    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
    {
        unsigned int nCamId     = uri.properties.Get<unsigned int>("id", 0);
        std::string sMode       = uri.properties.Get<std::string>("mode", "MONO8");
        ImageDim Dims           = uri.properties.Get<ImageDim>("size", ImageDim(640,480));
        ImageRoi ROI            = uri.properties.Get<ImageRoi>("roi", ImageRoi(0,0,0,0));
        float fFPS              = uri.properties.Get<float>("fps", 30);
        unsigned int nISO       = uri.properties.Get<unsigned int>("iso", 400);
        unsigned int nDMA       = uri.properties.Get<unsigned int>("dma", 4);

        if( ROI.w == 0 && ROI.h == 0 ) {
            ROI.w = Dims.x;
            ROI.h = Dims.y;
        }


        dc1394video_mode_t Mode;

        if( sMode.find( "FORMAT7" ) != std::string::npos ) {
            if( sMode == "FORMAT7_0" ) {
                Mode = DC1394_VIDEO_MODE_FORMAT7_0;
//                printf(" ");
//                std::cout << "Test" << std::endl;
            } else if( sMode == "FORMAT7_1" ) {
                Mode = DC1394_VIDEO_MODE_FORMAT7_1;
            } else if( sMode == "FORMAT7_2" ) {
                Mode = DC1394_VIDEO_MODE_FORMAT7_2;
            } else if( sMode == "FORMAT7_3" ) {
                Mode = DC1394_VIDEO_MODE_FORMAT7_3;
            } else if( sMode == "FORMAT7_4" ) {
                Mode = DC1394_VIDEO_MODE_FORMAT7_4;
            } else if( sMode == "FORMAT7_5" ) {
                Mode = DC1394_VIDEO_MODE_FORMAT7_5;
            } else if( sMode == "FORMAT7_6" ) {
                Mode = DC1394_VIDEO_MODE_FORMAT7_6;
            } else {
                Mode = DC1394_VIDEO_MODE_FORMAT7_7;
            }
        } else {
            if( sMode == "MONO16" ) {
                if( ROI.w == 1024 ) {
                    Mode = DC1394_VIDEO_MODE_1024x768_MONO16;
                } else if( ROI.w == 1280 ) {
                    Mode = DC1394_VIDEO_MODE_1280x960_MONO16;
                } else if( ROI.w == 1600 ) {
                    Mode = DC1394_VIDEO_MODE_1600x1200_MONO16;
                } else {
                    Mode = DC1394_VIDEO_MODE_640x480_MONO16;
                }
            } else if( sMode == "RGB8" ) {
                if( ROI.w == 1024 ) {
                    Mode = DC1394_VIDEO_MODE_1024x768_RGB8;
                } else if( ROI.w == 1280 ) {
                    Mode = DC1394_VIDEO_MODE_1280x960_RGB8;
                } else if( ROI.w == 1600 ) {
                    Mode = DC1394_VIDEO_MODE_1600x1200_RGB8;
                } else {
                    Mode = DC1394_VIDEO_MODE_640x480_RGB8;
                }
            } else {
                // MONO8
                if( ROI.w == 1024 ) {
                    Mode = DC1394_VIDEO_MODE_1024x768_MONO8;
                } else if( ROI.w == 1280 ) {
                    Mode = DC1394_VIDEO_MODE_1280x960_MONO8;
                } else if( ROI.w == 1600 ) {
                    Mode = DC1394_VIDEO_MODE_1600x1200_MONO8;
                } else {
                    Mode = DC1394_VIDEO_MODE_640x480_MONO8;
                }
            }
        }


        dc1394speed_t Speed;
        if( nISO == 100 ) {
            Speed = DC1394_ISO_SPEED_100;
        } else if( nISO == 200 ) {
            Speed = DC1394_ISO_SPEED_200;
        } else if( nISO == 800 ) {
            Speed = DC1394_ISO_SPEED_800;
        } else {
            Speed = DC1394_ISO_SPEED_400;
        }

//        printf("%d - %s - %dx%d - %d+%d+%dx%d - %f - %d - %d\n", nCamId,sMode.c_str(),Dims.x, Dims.y, ROI.x, ROI.y, ROI.w, ROI.h, fFPS, nISO, nDMA);

        DC1394Driver* pDriver = new DC1394Driver(
                    nCamId, Mode, ROI.x, ROI.y, ROI.w, ROI.h,
                    fFPS, Speed, nDMA
                    );
        return std::shared_ptr<CameraDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static DC1394Factory g_DC1394Factory("dc1394");

}
