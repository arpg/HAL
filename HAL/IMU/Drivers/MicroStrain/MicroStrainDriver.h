#pragma once

#include <thread>

#include <HAL/IMU/IMUDriverInterface.h>

#include "MIPSDK/mip_sdk.h"

namespace hal {

class MicroStrainDriver : public IMUDriverInterface
{
    public:
        MicroStrainDriver();
        ~MicroStrainDriver();
        void RegisterIMUDataCallback(IMUDriverDataCallback callback);

    private:
        static void ImuCallback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type);
        bool _Init();
        void _ThreadCaptureFunc();
        bool _ActivateAHRS();
        bool _ActivateGPS();

        volatile bool mShouldRun;
        mip_interface mDeviceInterface;
        std::thread mDeviceThread;

        IMUDriverDataCallback mIMUCallback;

        // properties
        bool m_bGetGPS;
        bool m_bGetAHRS;

        bool m_bGetTimeStampPpsAHRS;
        bool m_bGetTimeStampGpsCorrelationAHRS;
        bool m_bGetEulerAHRS;
        bool m_bGetQuaternionAHRS;
        bool m_bGetAccelerometerAHRS;
        bool m_bGetGyroAHRS;
        bool m_bGetMagnetometerAHRS;
        int  m_nHzGPS;
        int  m_nHzAHRS;

};

} /* namespace */
