#include "MicroStrainPosysDriver.h"

#include <HAL/IMU/Drivers/MicroStrain/MicroStrainDriver.h>

using namespace hal;


/////////////////////////////////////////////////////////////////////////////////////////
MicroStrainPosysDriver::MicroStrainPosysDriver(std::string filename)
    : m_callback(nullptr)
{
}

/////////////////////////////////////////////////////////////////////////////////////////
MicroStrainPosysDriver::~MicroStrainPosysDriver()
{
}

/////////////////////////////////////////////////////////////////////////////////////////
void MicroStrainPosysDriver::RegisterPosysDataCallback(PosysDriverDataCallback callback)
{
    m_callback = callback;
}


