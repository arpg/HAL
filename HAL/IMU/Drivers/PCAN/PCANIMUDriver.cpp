#include <HAL/Utils/TicToc.h>

#include "PCANIMUDriver.h"

using namespace hal;


/////////////////////////////////////////////////////////////////////////////////////////
PCANIMUDriver::PCANIMUDriver(const std::string& sCom)
    : m_PCANListener(PCANListener::GetInstance())
{
  m_PCANListener.Connect( sCom.c_str() );
}


/////////////////////////////////////////////////////////////////////////////////////////
PCANIMUDriver::~PCANIMUDriver()
{
  m_PCANListener.Disconnect();
}


/////////////////////////////////////////////////////////////////////////////////////////
void PCANIMUDriver::RegisterIMUDataCallback(IMUDriverDataCallback Callback)
{
  m_PCANListener.RegisterIMUCallback( *Callback.target<fPtr_IMU>() );
}
