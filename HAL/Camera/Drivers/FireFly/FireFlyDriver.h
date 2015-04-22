/*
   \file FireFlyDriver.h

 */

#ifndef _FIREFLY_H_
#define _FIREFLY_H_

#include <dc1394/dc1394.h>

#include <mvl/camera/camera.h>

#include "HAL/Camera/CameraDriverInterface.h"

namespace hal {

class FireFlyDriver : public CameraDriver
{
    public:
        FireFlyDriver();
        virtual ~FireFlyDriver();
        bool Capture( hal::CameraMsg& vImages );
        void PrintInfo();
        bool Init();
    private:
        void _cleanup_and_exit( dc1394camera_t *pCam );
    private:
        dc1394camera_t*             m_pCam[5];
        dc1394_t*                   m_pBus;
        dc1394video_mode_t          m_nVideoMode;
        dc1394framerate_t           m_nFramerate;
        unsigned int                m_nImageWidth;
        unsigned int                m_nImageHeight;
        unsigned int                m_nNumCams;
        mvl_camera_t*               m_pCMod[5];
        bool                        m_bOutputRectified;

};

}

#endif
