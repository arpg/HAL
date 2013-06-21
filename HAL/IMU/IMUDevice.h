#pragma once

#include <HAL/Devices/SharedLoad.h>

#include <HAL/IMU/IMUDriverInterface.h>
#include <HAL/Devices/DeviceFactory.h>
#include <HAL/Utils/Uri.h>

namespace hal {

///////////////////////////////////////////////////////////////////////////////
// Generic IMU device
class IMU : public IMUDriverInterface
{
    public:
        ///////////////////////////////////////////////////////////////
        IMU()
        {
        }

        ///////////////////////////////////////////////////////////////
        IMU(const std::string& uri)
            : m_URI(uri)
        {
            m_IMU = DeviceRegistry<IMUDriverInterface>::I().Create(m_URI);
        }

        ///////////////////////////////////////////////////////////////
        ~IMU()
        {
            Clear();
        }

        ///////////////////////////////////////////////////////////////
        void Clear()
        {
            m_IMU = nullptr;
        }

        ///////////////////////////////////////////////////////////////
        void RegisterIMUDataCallback(IMUDriverDataCallback callback)
        {
            if( m_IMU ){
                m_IMU->RegisterIMUDataCallback( callback );
            }else{
                std::cerr << "ERROR: no driver initialized!\n";
            }
            return;
        }

        ///////////////////////////////////////////////////////////////
        void RegisterGPSDataCallback(GPSDriverDataCallback callback)
        {
            if( m_IMU ){
                m_IMU->RegisterGPSDataCallback( callback );
            }else{
                std::cerr << "ERROR: no driver initialized!\n";
            }
            return;
        }

        ///////////////////////////////////////////////////////////////
        std::string GetDeviceProperty(const std::string& sProperty)
        {
            return m_IMU->GetDeviceProperty(sProperty);
        }


protected:
    hal::Uri                                m_URI;
    std::shared_ptr<IMUDriverInterface>     m_IMU;

};

} /* namespace */
