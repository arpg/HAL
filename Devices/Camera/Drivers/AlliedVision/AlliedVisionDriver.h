/*
   \file Bumblebee2Driver.h

 */

#ifndef _ALLIEDVISIONDRIVER_H_
#define _ALLIEDVISIONDRIVER_H_

#include <mvl/camera/camera.h>
#include "RPG/Devices/Camera/CameraDriverInterface.h"

// Forward declaration of Camera type
struct AlliedVisionCamera;

class AlliedVisionDriver : public CameraDriver
{
    public:
        AlliedVisionDriver();
        virtual ~AlliedVisionDriver();
        bool Capture( std::vector<rpg::ImageWrapper>& vImages );
        bool Init();
        void Deinit();
    private:
        AlliedVisionCamera* m_cam;
};

#endif // _ALLIEDVISIONDRIVER_H_
