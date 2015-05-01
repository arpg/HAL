#include "PhidgetsDriver.h"

#include <sys/time.h>

#include <HAL/Utils/TicToc.h>

namespace hal {

//callback that will run if the Spatial is attached to the computer
int CCONV AttachHandler(CPhidgetHandle spatial, void *userptr)
{
  ((PhidgetsDriver *)userptr)->_AttachHandler(spatial);
  return 0;
}

//callback that will run if the Spatial is detached from the computer
int CCONV DetachHandler(CPhidgetHandle spatial, void *userptr)
{
  ((PhidgetsDriver *)userptr)->_DetachHandler(spatial);
  return 0;
}

//callback that will run if the Spatial generates an error
int CCONV ErrorHandler(CPhidgetHandle spatial, void *userptr, int ErrorCode, const char *unknown)
{
  ((PhidgetsDriver *)userptr)->_ErrorHandler(spatial,ErrorCode,unknown);
  return 0;
}

//callback that will run at datarate
//data - array of spatial event data structures that holds the spatial data packets that were sent in this event
//count - the number of spatial data event packets included in this event
int CCONV SpatialDataHandler(CPhidgetSpatialHandle spatial, void *userptr, CPhidgetSpatial_SpatialEventDataHandle *data, int count)
{
  ((PhidgetsDriver *)userptr)->_SpatialDataHandler(spatial,data,count);
  return 0;
}

} /* namespace */

using namespace hal;


///////////////////////////////////////////////////////////////////////////
PhidgetsDriver::PhidgetsDriver() : m_hSpatial(0)
{
  Init();
}

///////////////////////////////////////////////////////////////////////////
PhidgetsDriver::~PhidgetsDriver()
{
  //since user input has been read, this is a signal to terminate the program so we will close the phidget and delete the object we created
  CPhidget_close((CPhidgetHandle)m_hSpatial);
  CPhidget_delete((CPhidgetHandle)m_hSpatial);
}

///////////////////////////////////////////////////////////////////////////
void PhidgetsDriver::_AttachHandler(CPhidgetHandle spatial)
{
  //Set the data rate for the spatial events
  int minRate;
  CPhidgetSpatial_getDataRateMax((CPhidgetSpatialHandle)spatial,&minRate);
  CPhidgetSpatial_setDataRate((CPhidgetSpatialHandle)spatial, minRate);

  //    int serialNo;
  //    CPhidget_getSerialNumber(spatial, &serialNo);
  //    printf("Spatial %10d attached!", serialNo);
}

///////////////////////////////////////////////////////////////////////////
void PhidgetsDriver::_DetachHandler(CPhidgetHandle /*spatial*/)
{
  //    int serialNo;
  //    CPhidget_getSerialNumber(spatial, &serialNo);
  //    printf("Spatial %10d detached! \n", serialNo);
}

///////////////////////////////////////////////////////////////////////////
void PhidgetsDriver::_ErrorHandler(CPhidgetHandle /*spatial*/, int ErrorCode, const char *unknown)
{
  printf("Phidgets Driver: Error handled. %d - %s \n", ErrorCode, unknown);
}

///////////////////////////////////////////////////////////////////////////
void PhidgetsDriver::_SpatialDataHandler(CPhidgetSpatialHandle /*spatial*/, CPhidgetSpatial_SpatialEventDataHandle *data, int count)
{
  int i;
  /*
    printf("Number of Data Packets in this event: %d\n", count);
    for(i = 0; i < count; i++)
    {
        printf("=== Data Set: %d ===\n", i);
        printf("Acceleration> x: %6f  y: %6f  x: %6f\n", data[i]->acceleration[0], data[i]->acceleration[1], data[i]->acceleration[2]);
        printf("Angular Rate> x: %6f  y: %6f  x: %6f\n", data[i]->angularRate[0], data[i]->angularRate[1], data[i]->angularRate[2]);
        printf("Magnetic Field> x: %6f  y: %6f  x: %6f\n", data[i]->magneticField[0], data[i]->magneticField[1], data[i]->magneticField[2]);
        printf("Timestamp> seconds: %d -- microseconds: %d\n", data[i]->timestamp.seconds, data[i]->timestamp.microseconds);
    }

    printf("---------------------------------------------\n");
    /**/

  for(i = 0; i < count; i++){
    hal::ImuMsg imuData;

    hal::VectorMsg* pbVec = imuData.mutable_accel();
    pbVec->add_data(data[i]->acceleration[0]);
    pbVec->add_data(data[i]->acceleration[1]);
    pbVec->add_data(data[i]->acceleration[2]);

    pbVec = imuData.mutable_gyro();
    pbVec->add_data(data[i]->angularRate[0]);
    pbVec->add_data(data[i]->angularRate[1]);
    pbVec->add_data(data[i]->angularRate[2]);

    pbVec = imuData.mutable_mag();
    pbVec->add_data(data[i]->magneticField[0]);
    pbVec->add_data(data[i]->magneticField[1]);
    pbVec->add_data(data[i]->magneticField[2]);

    imuData.set_system_time(hal::Tic());
    imuData.set_device_time(data[i]->timestamp.seconds + data[i]->timestamp.microseconds * 1E-6);

    if( m_ImuCallback ) {
      m_ImuCallback(imuData);
    }
  }
}

///////////////////////////////////////////////////////////////////////////
bool PhidgetsDriver::Init()
{
  //create the spatial object
  CPhidgetSpatial_create(&m_hSpatial);

  //Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
  CPhidget_set_OnAttach_Handler((CPhidgetHandle)m_hSpatial, AttachHandler, this);
  CPhidget_set_OnDetach_Handler((CPhidgetHandle)m_hSpatial, DetachHandler, this);
  CPhidget_set_OnError_Handler((CPhidgetHandle)m_hSpatial, ErrorHandler, this);

  //Registers a callback that will run according to the set data rate that will return the spatial data changes
  //Requires the handle for the Spatial, the callback handler function that will be called,
  //and an arbitrary pointer that will be supplied to the callback function (may be NULL)
  CPhidgetSpatial_set_OnSpatialData_Handler(m_hSpatial, SpatialDataHandler, this);

  //open the spatial object for device connections
  CPhidget_open((CPhidgetHandle)m_hSpatial, -1);

  return true;
}

///////////////////////////////////////////////////////////////////////////
void PhidgetsDriver::RegisterIMUDataCallback(IMUDriverDataCallback callback)
{
  m_ImuCallback = callback;
}


