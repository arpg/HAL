/*
   \file IMUDevice.h

   Abstract device that represents a generic IMU.

 */

#ifndef _IMU_DEVICE_H_
#define _IMU_DEVICE_H_

#include <RPG/Utils/PropertyMap.h>
#include <RPG/Devices/IMU/IMUDriverInterface.h>
#include <RPG/Devices/IMU/Drivers/IMUDriverRegistery.h>

// Driver Creation Factory
extern IMUDriver* CreateIMUDriver( const std::string& sDriverName );

///////////////////////////////////////////////////////////////////////////////
// Generic IMU device
class IMUDevice : public PropertyMap
{
    public:
        ///////////////////////////////////////////////////////////////
        IMUDevice()
            : m_pDriver(0)
        {
        }

        ///////////////////////////////////////////////////////////////
        ~IMUDevice()
        {
            DeinitDriver();
        }

        ///////////////////////////////////////////////////////////////
        void RegisterIMUDataCallback(IMUDriverDataCallback callback)
        {
            if( m_pDriver ){
                m_pDriver->RegisterDataCallback( callback );
            }else{
                std::cerr << "ERROR: no driver initialized!\n";
            }
            return;
        }

        ///////////////////////////////////////////////////////////////
        void RegisterGPSDataCallback(GPSDriverDataCallback callback)
        {
            if( m_pDriver ){
                m_pDriver->RegisterDataCallback( callback );
            }else{
                std::cerr << "ERROR: no driver initialized!\n";
            }
            return;
        }

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

            m_pDriver = CreateIMUDriver( sDriver );
            if( m_pDriver ){
                m_pDriver->SetPropertyMap( this );
                const bool success = m_pDriver->Init();
                if(!success) DeinitDriver();
                return success;
            }

            return false;
        }

    private:
        // A IMU device will create and initialize a particular driver:
        IMUDriver*          m_pDriver;
};

#endif
