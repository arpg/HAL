#include "NinjaEncoderDriver.h"


using namespace hal;


/////////////////////////////////////////////////////////////////////////////////////////
NinjaEncoderDriver::NinjaEncoderDriver(const std::string& sCom)
    : m_FtdiListener(FtdiListener::GetInstance())
{
    m_FtdiListener.Connect( sCom.c_str() );
}


/////////////////////////////////////////////////////////////////////////////////////////
NinjaEncoderDriver::~NinjaEncoderDriver()
{
    m_FtdiListener.Disconnect();
}


/////////////////////////////////////////////////////////////////////////////////////////
void NinjaEncoderDriver::RegisterEncoderDataCallback(EncoderDriverDataCallback Callback)
{
  m_FtdiListener.RegisterEncoderCallback( *Callback.target<fPtr_Encoder>() );
}
