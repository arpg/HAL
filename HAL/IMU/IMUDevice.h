#pragma once

#include <HAL/Uri.h>
#include <HAL/DeviceRegistry.h>
#include <HAL/IMU/IMUDriverInterface.h>

///////////////////////////////////////////////////////////////////////////////
// Generic IMU device
class IMU : public IMUDriverInterface
{
    public:
        ///////////////////////////////////////////////////////////////
        IMU()
            : m_pDriver(0)
        {
        }

        ///////////////////////////////////////////////////////////////
        ~IMU()
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

            m_pDriver = CreateIMUDriver( sDriver );
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

    private:
        // A IMU device will create and initialize a particular driver:
        IMUDriver*          m_pDriver;
};
