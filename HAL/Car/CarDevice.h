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

	std::string GetDeviceProperty(const std::string& sProperty) {
		return m_car->GetDeviceProperty(sProperty);
	}

  void RegisterCarStateDataCallback(CarStateDataCallback callback) {
    m_callback = callback;
    if( m_car ){
      m_car->RegisterCarStateDataCallback( callback );
    }else{
      std::cerr << "ERROR: failed to register StateDriverDataCallback!\n";
    }
    return;
  }

  ///////////////////////////////////////////////////////////////
  virtual void UpdateCarCommand( CarCommandMsg& car_command ) {
    m_car->UpdateCarCommand(car_command);
  }

protected:
  hal::Uri                            m_uri;
  std::shared_ptr<CarDriverInterface> m_car;
  CarStateDataCallback m_callback;
};

}

#endif
