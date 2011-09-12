/*
    \file Defines the interface all camera drivers must honor.
*/


#include <vector>
#ifndef _CAMERA_DRIVER_H_
#define _CAMERA_DRIVER_H_

#include <vector>
#include <RPG/Utils/PropertyMap.h> // so a CameraDevice can have generic "properties"

class Image
{
};

///////////////////////////////////////////////////////////////////////////////
// Generic camera driver interface
class CameraDriver
{
    public:
        // Pure virtual functions driver writers must implement:
        virtual bool Capture( std::vector<Image>& vImages ) = 0;
         virtual bool Init() = 0;
        CameraDriver()
        {
            m_pPropertyMap = NULL;
        }

        // Called by CameraDevice::InitDriver
        bool SetPropertyMap( PropertyMap* pMap )
        {
            m_pPropertyMap = pMap;
        }
    protected:
        PropertyMap* m_pPropertyMap; // from parent device that instantiates this driver.
};
#endif
