/*
   \file KinectDriver.h

 */

#ifndef _KINECT_H_
#define _KINECT_H_

#include "RPG/Devices/Camera/CameraDriverInterface.h"

#include <XnCppWrapper.h>

class KinectDriver : public CameraDriver
{
    public:
        KinectDriver();
        virtual ~KinectDriver();
        bool Capture( std::vector<rpg::ImageWrapper>& vImages );
        bool Init();
    private:
        bool                    m_bGetRGB;
        bool                    m_bGetDepth;
        unsigned int            m_nNumImgs;
        xn::Context             m_Context;
        xn::DepthGenerator      m_DepthNode;
        xn::ImageGenerator      m_ImageNode;
        xn::DepthMetaData       m_DepthMD;
        xn::ImageMetaData       m_ImageMD;
};

#endif
