#include <HAL/Utils/TicToc.h>

#include "NinjaIMUDriver.h"

using namespace hal;


/////////////////////////////////////////////////////////////////////////////////////////
NinjaIMUDriver::NinjaIMUDriver(const std::string& sCom)
    : m_FtdiListener(FtdiListener::GetInstance())
{
  m_FtdiListener.Connect( sCom.c_str() );
}


/////////////////////////////////////////////////////////////////////////////////////////
NinjaIMUDriver::~NinjaIMUDriver()
{
  m_FtdiListener.Disconnect();
}


/////////////////////////////////////////////////////////////////////////////////////////
void NinjaIMUDriver::RegisterIMUDataCallback(IMUDriverDataCallback Callback)
{
  m_FtdiListener.RegisterIMUCallback( *Callback.target<fPtr_IMU>() );
}
