/*
   \file CarDevice.h

   Abstract device that represents a generic car.

 */

#ifndef _CAR_DEVICE_H_
#define _CAR_DEVICE_H_

#include <RPG/Utils/PropertyMap.h>		// so a CarDevice can have generic "properties"
#include <RPG/Devices/Car/CarDriverInterface.h>
#include <RPG/Devices/Car/Drivers/CarDriverRegistery.h>

// Driver Creation Factory
extern CarDriver* CreateCarDriver( const std::string& sDriverName );

///////////////////////////////////////////////////////////////////////////////
// Generic car device
class CarDevice : public PropertyMap
{
    public:
        ///////////////////////////////////////////////////////////////
        CarDevice()
            : m_pDriver(0)
        {
        }

        ///////////////////////////////////////////////////////////////
        ~CarDevice()
        {
            if(m_pDriver) {
                delete m_pDriver;
            }
        }

        ///////////////////////////////////////////////////////////////
        bool InitDriver( const std::string& sDriver )
        {
            if(m_pDriver) {
                delete m_pDriver;
                m_pDriver = 0;
            }

            m_pDriver = CreateCarDriver( sDriver );
            if( m_pDriver ){
                m_pDriver->SetPropertyMap( this );
                return m_pDriver->Init();
            }
            return false;
        }

        ///////////////////////////////////////////////////////////////
        virtual bool ApplyCommand( float flVelocity, float flSteering ) = 0;

    private:
        // A car device will create and initialize a particular driver:
        CarDriver*          m_pDriver;
};

#endif
