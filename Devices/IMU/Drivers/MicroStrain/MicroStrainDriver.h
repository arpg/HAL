#ifndef _MICROSTRAINDRIVER_H_
#define _MICROSTRAINDRIVER_H_

#include <boost/thread.hpp>

#include "RPG/Devices/IMU/IMUDriverInterface.h"

#include "MIPSDK/mip_sdk.h"

class MicroStrainDriver : public IMUDriver
{
    public:
        MicroStrainDriver();
        virtual ~MicroStrainDriver();

        bool Init();
        void RegisterDataCallback(IMUDriverDataCallback callback);
        
    private:
        static void ImuCallback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type);
        void _ThreadCaptureFunc();

        bool mShouldRun;
        mip_interface mDeviceInterface;
        boost::thread mDeviceThread;
        IMUDriverDataCallback mCallback;
};

#endif
