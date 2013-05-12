#ifndef _MICROSTRAINDRIVER_H_
#define _MICROSTRAINDRIVER_H_

#include <boost/thread.hpp>

#include "HAL/IMU/IMUDriverInterface.h"

#include "MIPSDK/mip_sdk.h"

class MicroStrainDriver : public IMUDriverInterface
{
    public:
        MicroStrainDriver();
        virtual ~MicroStrainDriver();

        bool Init();
        void RegisterDataCallback(IMUDriverDataCallback callback);
        void RegisterDataCallback(GPSDriverDataCallback callback);

    private:
        static void ImuCallback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type);
        void _ThreadCaptureFunc();
        bool _ActivateAHRS();
        bool _ActivateGPS();

        volatile bool mShouldRun;
        mip_interface mDeviceInterface;
        boost::thread mDeviceThread;

        IMUDriverDataCallback mIMUCallback;
        GPSDriverDataCallback mGPSCallback;

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

#endif
