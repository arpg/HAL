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
        bool Capture(AlliedVisionCamera* cam, rpg::ImageWrapper& img );

        bool Init();
        void Deinit();

        bool InitCamera(AlliedVisionCamera* cam, unsigned int width, unsigned int height);
        void DeinitCamera(AlliedVisionCamera* cam);

        bool StartCamera(AlliedVisionCamera* cam);

        AlliedVisionCamera* GetFirstCamera();
    private:
        size_t m_numCams;
        std::vector<AlliedVisionCamera*> m_cam;
};

#endif // _ALLIEDVISIONDRIVER_H_
