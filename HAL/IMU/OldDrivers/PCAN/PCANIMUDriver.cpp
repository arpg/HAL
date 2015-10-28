#include <HAL/Utils/TicToc.h>

#include "PCANIMUDriver.h"

using namespace hal;


/////////////////////////////////////////////////////////////////////////////////////////
PCANIMUDriver::PCANIMUDriver(unsigned long int baudrate, const std::string& sCom)
    : m_PCANListener(PCANListener::GetInstance())
{
  m_PCANListener.Connect( baudrate, sCom );
}


/////////////////////////////////////////////////////////////////////////////////////////
PCANIMUDriver::~PCANIMUDriver()
{
  m_PCANListener.Disconnect();
}


/////////////////////////////////////////////////////////////////////////////////////////
void PCANIMUDriver::RegisterIMUDataCallback(IMUDriverDataCallback Callback)
{
  m_PCANListener.RegisterIMUCallback( Callback );
}
