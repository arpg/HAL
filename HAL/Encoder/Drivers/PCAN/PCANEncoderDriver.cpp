#include "PCANEncoderDriver.h"


using namespace hal;


/////////////////////////////////////////////////////////////////////////////////////////
PCANEncoderDriver::PCANEncoderDriver(const std::string& sCom)
    : m_PCANListener(PCANListener::GetInstance())
{
    m_PCANListener.Connect( sCom.c_str() );
}


/////////////////////////////////////////////////////////////////////////////////////////
PCANEncoderDriver::~PCANEncoderDriver()
{
    m_PCANListener.Disconnect();
}


/////////////////////////////////////////////////////////////////////////////////////////
void PCANEncoderDriver::RegisterEncoderDataCallback(EncoderDriverDataCallback Callback)
{
  m_PCANListener.RegisterEncoderCallback( *Callback.target<fPtr_Encoder>() );
}
