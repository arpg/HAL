#pragma once

#include <thread>

#include <HAL/IMU/IMUDriverInterface.h>
#include <HAL/Posys/PosysDriverInterface.h>
#include <HAL/Utils/Uri.h>

#include "MIPSDK/mip_sdk.h"

namespace hal {

  class MicroStrainDriver : public IMUDriverInterface
  {
    public:
      MicroStrainDriver( const Uri& uri );
      ~MicroStrainDriver();
      void RegisterIMUDataCallback(IMUDriverDataCallback callback);
      bool IsRunning() const override {
        return mShouldRun;
      }

      // Auxiliary non-standard methods for Posys integration.
      static void RegisterPosysDataCallback(PosysDriverDataCallback callback);


      std::string GetProperty(const std::string& sProperty)
      {
        return sProperty;
      }


    private:
      static IMUDriverDataCallback   mIMUCallback;
      static PosysDriverDataCallback mPosysCallback;
      static void CallbackFunc(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type);

      bool _Init();
      void _ThreadCaptureFunc();
      bool _ActivateAHRS();
      bool _ActivateGPS();

      volatile bool mShouldRun;
      mip_interface mDeviceInterface;
      std::thread mDeviceThread;

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
      std::string m_sPortName;

  };

} /* namespace */
