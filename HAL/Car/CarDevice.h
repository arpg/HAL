/*
   \file CarDevice.h

   Abstract device that represents a generic car.

 */

#ifndef _CAR_DEVICE_H_
#define _CAR_DEVICE_H_

#include <HAL/Car/CarDriverInterface.h>
#include <HAL/Devices/DeviceFactory.h>
#include <HAL/Utils/Uri.h>

namespace hal {
///////////////////////////////////////////////////////////////////////////////
// Generic car device
class Car : public CarDriverInterface {
public:
  ///////////////////////////////////////////////////////////////
  Car()
  {
  }

  ///////////////////////////////////////////////////////////////
  Car(const std::string& uri)
    :m_uri(uri)
  {
    m_car = DeviceRegistry<CarDriverInterface>::Instance().Create(m_uri);
  }

  ///////////////////////////////////////////////////////////////
  Car(const hal::Uri& uri)
    :m_uri(uri)
  {
    m_car = DeviceRegistry<CarDriverInterface>::Instance().Create(m_uri);
  }

  ///////////////////////////////////////////////////////////////
  ~Car()
  {
    Clear();
  }

  inline void Clear() {
    m_car = nullptr;
  }

  inline void Reset() {
    Clear();
    m_car = DeviceRegistry<CarDriverInterface>::Instance().Create(m_uri);
  }

  ///////////////////////////////////////////////////////////////
  virtual bool ApplyCommand( double flTorque, double flSteering,
                             double flCommand_Time ) {
    return m_car->ApplyCommand(flTorque,flSteering,flCommand_Time);
  }

protected:
  hal::Uri                            m_uri;
  std::shared_ptr<CarDriverInterface> m_car;
};

}

#endif
