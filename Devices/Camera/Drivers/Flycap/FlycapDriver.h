/*
   \file FlycapDriver.h

 */

#ifndef _FLYCAP_H_
#define _FLYCAP_H_

#include <flycapture/FlyCapture2.h>
#include "RPG/Devices/Camera/CameraDriverInterface.h"

class FlycapDriver : public CameraDriver
{
    public:
        FlycapDriver();
        virtual ~FlycapDriver();
        bool Capture( std::vector<rpg::ImageWrapper>& vImages );
        bool Init();
	private:
        void PrintError( FlyCapture2::Error error );
        void CheckError( FlyCapture2::Error error );
    private:
        FlyCapture2::Camera      m_Cam1;
        FlyCapture2::Camera      m_Cam2;
        unsigned int             m_nImgWidth;
        unsigned int             m_nImgHeight;

};

#endif
