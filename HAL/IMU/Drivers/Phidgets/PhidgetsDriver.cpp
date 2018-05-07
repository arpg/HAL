#include "PhidgetsDriver.h"

#include <sys/time.h>

#include <HAL/Utils/TicToc.h>

namespace hal {

// callback that will run if the Spatial is attached to the computer
void CCONV AttachHandler(PhidgetHandle spatial, void *userptr) {
  reinterpret_cast<PhidgetsDriver *>(userptr)->_AttachHandler(spatial);
}

// callback that will run if the Spatial is detached from the computer
void CCONV DetachHandler(PhidgetHandle spatial, void *userptr) {
  reinterpret_cast<PhidgetsDriver *>(userptr)->_DetachHandler(spatial);
}

// callback that will run if the Spatial generates an error
void CCONV ErrorHandler(PhidgetHandle spatial, void *userptr,
    Phidget_ErrorEventCode ErrorCode, const char *unknown) {
  reinterpret_cast<PhidgetsDriver *>(userptr)->_ErrorHandler(spatial, ErrorCode, unknown);
}

// callback that will run at datarate
// data - array of spatial event data structures that holds the spatial data packets that were sent in this event
// count - the number of spatial data event packets included in this event
void CCONV SpatialDataHandler(PhidgetSpatialHandle spatial, void *userptr,
    const double accel[3], const double angular[3], const double mag_field[3],
    double timestamp) {
  reinterpret_cast<PhidgetsDriver *>(userptr)->_SpatialDataHandler(spatial,
      accel, angular, mag_field, timestamp);
}

} // namespace hal


///////////////////////////////////////////////////////////////////////////
hal::PhidgetsDriver::PhidgetsDriver() : m_hSpatial(0) {
  Init();
}

///////////////////////////////////////////////////////////////////////////
hal::PhidgetsDriver::~PhidgetsDriver() {
  // since user input has been read, this is a signal to terminate the program so we will close the phidget and delete the object we created
  Phidget_close((PhidgetHandle)m_hSpatial);
  // Phidget_delete((PhidgetHandle)m_hSpatial);
}

///////////////////////////////////////////////////////////////////////////
void hal::PhidgetsDriver::_AttachHandler(PhidgetHandle spatial) {
  // Set the data rate for the spatial events
  uint32_t interval;
  PhidgetSpatial_getMinDataInterval((PhidgetSpatialHandle)spatial, &interval);
  PhidgetSpatial_setDataInterval((PhidgetSpatialHandle)spatial, interval);

  //    int serialNo;
  //    Phidget_getSerialNumber(spatial, &serialNo);
  //    printf("Spatial %10d attached!", serialNo);
}

///////////////////////////////////////////////////////////////////////////
void hal::PhidgetsDriver::_DetachHandler(PhidgetHandle /*spatial*/) {
  //    int serialNo;
  //    Phidget_getSerialNumber(spatial, &serialNo);
  //    printf("Spatial %10d detached! \n", serialNo);
}

///////////////////////////////////////////////////////////////////////////
void hal::PhidgetsDriver::_ErrorHandler(PhidgetHandle /*spatial*/,
    Phidget_ErrorEventCode ErrorCode, const char *unknown) {
  printf("Phidgets Driver: Error handled. %d - %s \n", ErrorCode, unknown);
}

///////////////////////////////////////////////////////////////////////////
void hal::PhidgetsDriver::_SpatialDataHandler(PhidgetSpatialHandle /*spatial*/,
    const double accel[3], const double angular[3], const double mag_field[3],
    double timestamp) {

  hal::ImuMsg imuData;

  hal::VectorMsg* pbVec = imuData.mutable_accel();
  pbVec->add_data(accel[0]);
  pbVec->add_data(accel[1]);
  pbVec->add_data(accel[2]);

  pbVec = imuData.mutable_gyro();
  pbVec->add_data(angular[0]);
  pbVec->add_data(angular[1]);
  pbVec->add_data(angular[2]);

  pbVec = imuData.mutable_mag();
  pbVec->add_data(mag_field[0]);
  pbVec->add_data(mag_field[1]);
  pbVec->add_data(mag_field[2]);

  imuData.set_system_time(hal::Tic());
  imuData.set_device_time(timestamp);

  if (m_ImuCallback) {
    m_ImuCallback(imuData);
  }
}

///////////////////////////////////////////////////////////////////////////
bool hal::PhidgetsDriver::Init() {
  // create the spatial object
  PhidgetSpatial_create(&m_hSpatial);

  // Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
  Phidget_setOnAttachHandler((PhidgetHandle)m_hSpatial, AttachHandler, this);
  Phidget_setOnDetachHandler((PhidgetHandle)m_hSpatial, DetachHandler, this);
  Phidget_setOnErrorHandler((PhidgetHandle)m_hSpatial, ErrorHandler, this);

  // Registers a callback that will run according to the set data rate that will return the spatial data changes
  // Requires the handle for the Spatial, the callback handler function that will be called,
  // and an arbitrary pointer that will be supplied to the callback function (may be NULL)
  PhidgetSpatial_setOnSpatialDataHandler(m_hSpatial, SpatialDataHandler, this);

  // open the spatial object for device connections
  Phidget_open((PhidgetHandle)m_hSpatial);

  return true;
}

///////////////////////////////////////////////////////////////////////////
void hal::PhidgetsDriver::RegisterIMUDataCallback(IMUDriverDataCallback callback) {
  m_ImuCallback = callback;
}
