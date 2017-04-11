#include "GladiatorDriver.h"

#include <sys/time.h>

#include <HAL/Utils/TicToc.h>

namespace hal {
using namespace hal;

  //Data is reported in fixed point, adjust for reporting in floating point
#define GYRO_TO_DEG_S (0.001f) // Gyro X,Y,Z
#define GYRO_TO_RAD_S (1.745329e-5f) // 0.001 * PI / 180.0
#define ACCL_TO_G (0.0001f) // Accel X,Y,Z
#define ACCL_TO_M_S2 (9.81e-4f) // 0.0001 * 9.81

///////////////////////////////////////////////////////////////////////////
GladiatorDriver::GladiatorDriver(const char* mPort)
{
  Init(mPort);
}

///////////////////////////////////////////////////////////////////////////
GladiatorDriver::~GladiatorDriver()
{

}


///////////////////////////////////////////////////////////////////////////
bool GladiatorDriver::Init(const char* mPort)
{
  //Instantiate a Gladiator device on the given port
  printf("GladiatorDriver: Opening IMU on port: %s\n", mPort);
  m_dev = new GladiatorIMU(mPort);
  m_dev->start();

  //The device is now (potentially) producing data asynchronously

  //Spawn a service thread to wait on data to become available and call the callback routine
  
  return true;
}
  
///////////////////////////////////////////////////////////////////////////
void GladiatorDriver::RegisterIMUDataCallback(IMUDriverDataCallback callback)
{
  m_IMUCallback = callback;
  if( !m_DeviceThread.joinable() ) {
    // start capture thread
    m_bShouldRun = true;
    m_DeviceThread = std::thread( &GladiatorDriver::service, this );
  }
}
  
  void GladiatorDriver::service()
  {
    while (m_bShouldRun)
      {
	imu_data_t* rawIMU = (imu_data_t*) m_dev->WaitData();
	//Translate the imu packet from the fixed point to the floating point
	 hal::ImuMsg dataIMU;
	 hal::VectorMsg* pbAccel = dataIMU.mutable_accel();
	 pbAccel->add_data(rawIMU->accel_x*ACCL_TO_M_S2);
	 pbAccel->add_data(rawIMU->accel_y*ACCL_TO_M_S2);
	 pbAccel->add_data(rawIMU->accel_z*ACCL_TO_M_S2);
	 
	 hal::VectorMsg* pbGyro = dataIMU.mutable_gyro();
	 pbGyro->add_data(rawIMU->gyro_x*GYRO_TO_RAD_S);
	 pbGyro->add_data(rawIMU->gyro_y*GYRO_TO_RAD_S);
	 pbGyro->add_data(rawIMU->gyro_z*GYRO_TO_RAD_S);
	 double halStamp = rawIMU->tv_sec + ((double)rawIMU->tv_nsec)/1e9;
	 
	 if( m_IMUCallback )
	   {
	     //Pass through the timestamp
	     dataIMU.set_device_time(halStamp);
	     dataIMU.set_system_time(halStamp);
	     m_IMUCallback( dataIMU );
	   }
      }
  }
}



