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
            if(m_pDriver) {
                delete m_pDriver;
            }
        }

        /*
        ///////////////////////////////////////////////////////////////
        bool InitDriver( const std::string& sRobotModelFile, const std::string& sDeviceName )
        {
            std::string sWorldModelFile = getenv( "SIMCFG" ); // e.g., world.xml
            if( !sWorldModelFile.isempty() ){
                m_pDriver = CreateSimCameraDriver( sDriver, sWorldModelFile, sRobotModelFile );  
                // CreateSimCameraDriver will:
                // 1) connect to RobotProxy via node
                //    (start RobotProxy if not already running, name the proxy after the robot)
                // 2) ask proxy to start SimCam
                // 3) then start and connect with a NodeCam
            }
            else{
                m_pDriver = CreateCameraDriver( sDriver );
            }
            m_pDriver = CreateCameraDriver( sDriver );
            if( m_pDriver ){
                m_pDriver->SetPropertyMap( this );
                return m_pDriver->Init();
            }
            return false;
        }
        */

        bool InitDriver( const std::string& sDriver )
        {
            if(m_pDriver) {
                delete m_pDriver;
                m_pDriver = 0;
            }

            /*
            std::string sWorldModelFile = getenv( "SIMCFG" ); // e.g., world.xml
            if( !sWorldModelFile.isempty() ){
                printf("WARNING: you have asked for simulation, but you have not provided"
                        " a model of the robot.  Please use the other API\n" );
            }

            std::string sWorldModelFile = getenv( "SIMCFG" ); // e.g., world.xml
            if( !sWorldModelFile.isempty() ){
                m_pDriver = CreateSimCameraDriver( sDriver, sWorldModelFile,  ); 
                // connect to RobotProxy via node
                //  (start RobotProxy if not already running, name the proxy after the robot)
                // ask proxy to start SimCam
                // then start and connect with a NodeCam
            }
            else{
                m_pDriver = CreateCameraDriver( sDriver );
            }
            */
            m_pDriver = CreateCameraDriver( sDriver );
            if( m_pDriver ){
                m_pDriver->SetPropertyMap( this );
                return m_pDriver->Init();
            }
            return false;
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
