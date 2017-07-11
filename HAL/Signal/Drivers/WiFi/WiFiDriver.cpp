#include <iostream>

#include <HAL/Devices/DeviceException.h>
#include <HAL/Utils/TicToc.h>

#include "WiFiDriver.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic pop


using namespace hal;

SignalDriverDataCallback   WiFiDriver::mSignalCallback = nullptr;

const int DEFAULT_PACKET_TIMEOUT_MS = 1000;

#define _LINUX

///////////////////////////////////////////////////////////////////////////////
WiFiDriver::WiFiDriver(std::string ifname,
                                     int scan_period)
    : mShouldRun(false),
      m_nPdScan(scan_period),
      m_sIFName(ifname)
{
}

///////////////////////////////////////////////////////////////////////////////
WiFiDriver::~WiFiDriver()
{
    mShouldRun = false;
    mDeviceThread.join();
}

///////////////////////////////////////////////////////////////////////////////
bool WiFiDriver::_Init()
{
    mShouldRun = true;
    mDeviceThread = std::thread(std::bind(&WiFiDriver::_ThreadCaptureFunc,this));

    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////
void WiFiDriver::RegisterSignalDataCallback(SignalDriverDataCallback callback)
{
  mSignalCallback = callback;
  if(_Init() == false) {
    throw DeviceException("Error initializing Signal.");
  }
}

///////////////////////////////////////////////////////////////////////////////
void WiFiDriver::_ThreadCaptureFunc()
{
  while(mShouldRun){
    hal::SignalMsg dataSignal;
    hal::SignalScanResult *result;

    std::time_t last_scan_time = std::chrono::system_clock::to_time_t(
        std::chrono::system_clock::now());
    std::vector<AccessPoint> aps = scan(m_sIFName);

    dataSignal.set_device_time(hal::Tic());
    dataSignal.set_system_time(hal::Tic());

    for (unsigned int i=0; i<aps.size(); i++) {
      result = dataSignal.add_scan_result();
      result->set_ssid(aps[i].ssid);
      result->set_bssid(aps[i].bssid);
      result->set_rssi(aps[i].rssi);
    }

    if (mSignalCallback) {
      mSignalCallback(dataSignal);
    }
    std::this_thread::sleep_until(std::chrono::system_clock::from_time_t(
          last_scan_time + this->m_nPdScan));
  }
  mShouldRun = false;
}
