#include "PCANEncoderDriver.h"


using namespace hal;


/////////////////////////////////////////////////////////////////////////////////////////
PCANEncoderDriver::PCANEncoderDriver(unsigned long int baudrate, const std::string& sCom)
    : m_PCANListener(PCANListener::GetInstance())
{
    m_PCANListener.Connect( baudrate,sCom);
}


/////////////////////////////////////////////////////////////////////////////////////////
PCANEncoderDriver::~PCANEncoderDriver()
{
    m_PCANListener.Disconnect();
}


/////////////////////////////////////////////////////////////////////////////////////////
void PCANEncoderDriver::RegisterEncoderDataCallback(EncoderDriverDataCallback Callback)
{
  m_PCANListener.RegisterEncoderCallback( Callback );
}
