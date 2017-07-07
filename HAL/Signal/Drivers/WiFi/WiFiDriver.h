#pragma once

#include <thread>

#include <HAL/Signal/SignalDriverInterface.h>
#include <HAL/Posys/PosysDriverInterface.h>

namespace hal {

class WiFiDriver : public SignalDriverInterface
{
    public:
        WiFiDriver(std::string ifname,
                          int  scan_hz);
        ~WiFiDriver();
        void RegisterSignalDataCallback(SignalDriverDataCallback callback);
  bool IsRunning() const override {
    return mShouldRun;
  }

        // Auxiliary non-standard methods for Posys integration.
        static void RegisterPosysDataCallback(PosysDriverDataCallback callback);

    private:
        static SignalDriverDataCallback   mSignalCallback;
        static PosysDriverDataCallback mPosysCallback;

        bool _Init();
        void _ThreadCaptureFunc();
        bool _ActivateAHRS();
        bool _ActivateGPS();

        volatile bool mShouldRun;
        std::thread mDeviceThread;

        // properties
        int  m_nHzScan;
        std::string m_sIFName;

};

} /* namespace */

