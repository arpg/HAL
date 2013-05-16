#pragma once

#include <dc1394/dc1394.h>

#include <HAL/Camera/CameraDriverInterface.h>


namespace hal {

class DC1394Driver : public CameraDriverInterface
{
    public:
        const static int MAX_FR = -1;
        const static int EXT_TRIG = -1;

        DC1394Driver(unsigned int nCamId, dc1394video_mode_t Mode,
                     unsigned int nTop, unsigned int nLeft,
                     unsigned int nWidth, unsigned int nHeight,
                     float fFPS, dc1394speed_t ISO,
                     unsigned int nDMA);

        bool Capture( pb::CameraMsg& vImages );

        std::string GetDeviceProperty(const std::string& sProperty);

        unsigned int Width( unsigned int idx = 0 );

        unsigned int Height( unsigned int idx = 0 );


    private:
        void _SetImageMetaDataFromCamera( pb::ImageMsg* img, dc1394camera_t* pCam );
        inline int _NearestValue(int value, int step, int min, int max);


    private:
        // TODO set bus to static?
        dc1394_t*                   m_pBus;
        dc1394camera_t*             m_pCam;
        pb::Type                    m_VideoType;
        pb::Format                  m_VideoFormat;
        unsigned int                m_nImageWidth;
        unsigned int                m_nImageHeight;

};

}
