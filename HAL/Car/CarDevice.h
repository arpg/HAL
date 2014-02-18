/*
   \file CarDevice.h

   Abstract device that represents a generic car.

 */

#ifndef _CAR_DEVICE_H_
#define _CAR_DEVICE_H_

#include <HAL/Car/CarDriverInterface.h>
#include <HAL/Devices/DeviceFactory.h>
#include <HAL/Utils/Uri.h>

///////////////////////////////////////////////////////////////////////////////
// Generic car device
class Car : public CarDriverInterface
{
    public:
        ///////////////////////////////////////////////////////////////
        Car()
        {
        }

        ///////////////////////////////////////////////////////////////
        Car(const std::string& uri)
          :m_uri(uri)
        {
          m_cam = DeviceRegistry<CarDriverInterface>::I().Create(m_uri,"Car");
        }

        ///////////////////////////////////////////////////////////////
        Car(const hal::Uri& uri)
          :m_uri(uri)
        {
          m_cam = DeviceRegistry<CarDriverInterface>::I().Create(m_uri,"Car");
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
          m_car = DeviceRegistry<CarDriverInterface>::I().Create(m_uri);
        }

        ///////////////////////////////////////////////////////////////
        virtual bool ApplyCommand( float flTorque, float flSteering ) {
          m_car->ApplyCommand(flTorque,flSteering);
        }

    protected:
        hal::Uri                            m_uri;
        std::shared_ptr<CarDriverInterface> m_car;
};

#endif
