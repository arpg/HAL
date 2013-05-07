/*
    \file Defines the interface all camera drivers must honor.
*/


#ifndef _CAMERA_DRIVER_H_
#define _CAMERA_DRIVER_H_

#include <Messages/Camera.pb.h>
#include <RPG/Utils/PropertyMap.h>

// TODO remove this once OpenCV dependency is removed
#include <opencv.hpp>

///////////////////////////////////////////////////////////////////////////////
// Generic camera driver interface
class CameraDriver
{
    public:
        // Pure virtual functions driver writers must implement:
        virtual bool Capture( pb::CameraMsg& vImages ) = 0;
        virtual void PrintInfo() = 0;
        virtual bool Init() = 0;

        virtual ~CameraDriver()
        {

        }
        CameraDriver()
        {
            m_pPropertyMap = NULL;
        }

        // Called by CameraDevice::InitDriver
        void SetPropertyMap( PropertyMap* pMap )
        {
            m_pPropertyMap = pMap;
        }
    protected:
        PropertyMap* m_pPropertyMap; // from parent device that instantiates this driver.
};
#endif
