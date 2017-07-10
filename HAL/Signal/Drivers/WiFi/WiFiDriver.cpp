#include <iostream>

#include <HAL/Devices/DeviceException.h>
#include <HAL/Utils/TicToc.h>

#include "WiFiDriver.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic pop


using namespace hal;

SignalDriverDataCallback   WiFiDriver::mSignalCallback = nullptr;
PosysDriverDataCallback WiFiDriver::mPosysCallback = nullptr;

const int DEFAULT_PACKET_TIMEOUT_MS = 1000;

#define _LINUX

///////////////////////////////////////////////////////////////////////////////
WiFiDriver::WiFiDriver(std::string ifname,
                                     int scan_hz)
    : mShouldRun(false),
      m_nHzScan(scan_hz),
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


/////////////////////////////////////////////////////////////////////////////////////////
void WiFiDriver::RegisterPosysDataCallback(PosysDriverDataCallback callback)
{
  mPosysCallback = callback;
}


///////////////////////////////////////////////////////////////////////////////
void WiFiDriver::_ThreadCaptureFunc()
{
    while(mShouldRun){
        scan("en0");
    }
    mShouldRun = false;
}
