/*
   \file Bumblebee2Driver.h

 */

#ifndef _BUMBLEBEE2_H_
#define _BUMBLEBEE2_H_

#include <dc1394/dc1394.h>
#include "RPG/Devices/Camera/CameraDriverInterface.h"

class Bumblebee2Driver : public CameraDriver
{
    public:
        Bumblebee2Driver();
        virtual ~Bumblebee2Driver();
        bool Capture( std::vector<cv::Mat>& vImages );
        bool Init();
    private:
        dc1394camera_t*          m_pCam;
        dc1394_t*                m_pBus;
        dc1394video_mode_t       m_nVideoMode;
        dc1394framerate_t        m_nFramerate;
        dc1394featureset_t       m_vFeatures;
        unsigned int             m_nImageWidth;
        unsigned int             m_nImageHeight;
};

#endif
