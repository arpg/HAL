/*
   \file Bumblebee2Driver.h

 */

#ifndef _BUMBLEBEE2_H_
#define _BUMBLEBEE2_H_

#include <dc1394/dc1394.h>
#include <mvl/camera/camera.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include "RPG/Camera/CameraDriverInterface.h"

class Bumblebee2Driver : public CameraDriver
{
    public:
        Bumblebee2Driver();
        virtual ~Bumblebee2Driver();
        bool Capture( std::vector<rpg::ImageWrapper>& vImages );
        void PrintInfo();
        bool Init();
    private:
        ///////////////////////////////////////////////////////////////////////////////
        void _SetImageMetaDataFromCamera(rpg::ImageWrapper& img, dc1394camera_t* pCam);
        void _bayer8_to_grey8_half(unsigned char* src, unsigned char* dst, unsigned int srcWidth, unsigned int srcHeight );
        void _cleanup_and_exit( dc1394camera_t *pCam );
    private:
        dc1394camera_t*         m_pCam;
        dc1394_t*               m_pBus;
        unsigned char*          m_pDeinterlaceBuffer;
        unsigned char*          m_pDebayerBuffer;
        dc1394video_mode_t      m_nVideoMode;
        dc1394framerate_t       m_nFramerate;
        dc1394featureset_t      m_vFeatures;
        unsigned int            m_uImageWidth;
        unsigned int            m_uImageHeight;
        mvl_camera_t*		    m_pLeftCMod;
        mvl_camera_t*		    m_pRightCMod;

        int                     m_nCvOutputType;
        bool                    m_bOutputRectified;
};

#endif
