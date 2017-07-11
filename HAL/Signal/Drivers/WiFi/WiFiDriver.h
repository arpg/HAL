#pragma once

#include <thread>
#include <string>

#include <HAL/Signal/SignalDriverInterface.h>

struct AccessPoint {
  std::string ssid;
  std::string bssid;
  int rssi;
};

std::vector<AccessPoint> scan(std::string ifname);

namespace hal {

  class WiFiDriver : public SignalDriverInterface
  {
    public:
      WiFiDriver(std::string ifname,
          int  scan_period);
      ~WiFiDriver();
      void RegisterSignalDataCallback(SignalDriverDataCallback callback);
      bool IsRunning() const override {
        return mShouldRun;
      }

    private:
      static SignalDriverDataCallback mSignalCallback;

      bool _Init();
      void _ThreadCaptureFunc();

      volatile bool mShouldRun;
      std::thread mDeviceThread;

      // properties
      int  m_nPdScan;
      std::string m_sIFName;

  };

} /* namespace */

