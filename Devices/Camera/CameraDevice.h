/*
   \file CameraDevice.h

   Abstract device that represents a generic camera.

 */

#ifndef _CAMERA_DEVICE_H_
#define _CAMERA_DEVICE_H_

#include <RPG/Utils/PropertyMap.h>		// so a CameraDevice can have generic "properties"
#include <RPG/Devices/Camera/CameraDriverInterface.h>
#include <RPG/Devices/Camera/Drivers/CameraDriverRegistery.h>

// Driver Creation Factory
extern CameraDriver* CreateCameraDriver( const std::string& sDriverName );

///////////////////////////////////////////////////////////////////////////////
// Generic camera device
class CameraDevice : public PropertyMap
{
    public:
        ///////////////////////////////////////////////////////////////
        CameraDevice()
            : m_pDriver(0)
        {
        }

        ///////////////////////////////////////////////////////////////
        ~CameraDevice()
        {
            DeinitDriver();
        }
        
        ///////////////////////////////////////////////////////////////
        void DeinitDriver()
        {
            if(m_pDriver) {
                delete m_pDriver;
                m_pDriver = 0;
            }            
        }

        ///////////////////////////////////////////////////////////////
        bool InitDriver( const std::string& sDriver )
        {
            DeinitDriver();
            
            m_pDriver = CreateCameraDriver( sDriver );
            if( m_pDriver ){
                m_pDriver->SetPropertyMap( this );
                const bool success = m_pDriver->Init();
                if(!success) DeinitDriver();
                return success; 
            }
            return false;
        }
        
        ///////////////////////////////////////////////////////////////
        bool IsInitialized()
        {
            return m_pDriver;
        }

        ///////////////////////////////////////////////////////////////
        bool Capture( std::vector<rpg::ImageWrapper>& vImages )
        {
            if( m_pDriver ){
                return m_pDriver->Capture( vImages );
            }
            std::cerr << "ERROR: no driver initialized!\n";
            return false;
        }

    private:
        // A camera device will create and initialize a particular driver:
        CameraDriver*          m_pDriver;
};

#endif
