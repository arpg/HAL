#pragma once

#include <dc1394/dc1394.h>

#include <HAL/Camera/CameraDriverInterface.h>


namespace hal {

class DC1394Driver : public CameraDriverInterface
{
    public:

        DC1394Driver(unsigned int nCamId);

        bool Capture( pb::CameraMsg& vImages );

        std::string GetDeviceProperty(const std::string& sProperty);

        unsigned int Width( unsigned int idx = 0 );

        unsigned int Height( unsigned int idx = 0 );


    private:
        void _cleanup_and_exit( dc1394camera_t *pCam );


    private:
        // TODO set bus to static?
        dc1394_t*                   m_pBus;
        dc1394camera_t*             m_pCam;
        dc1394video_mode_t          m_nVideoMode;
        dc1394framerate_t           m_nFramerate;
        unsigned int                m_nImageWidth;
        unsigned int                m_nImageHeight;

};

}
