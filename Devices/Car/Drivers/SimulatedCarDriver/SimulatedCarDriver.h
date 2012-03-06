/*
    \file Simulated car driver declaration
*/

#ifndef SIMULATED_CAR_DRIVER_H
#define SIMULATED_CAR_DRIVER_H

// each car driver can have a generic set of "properties"
#include "PropertyMap.h"

#include "CarDriverInterface.h"

///////////////////////////////////////////////////////////////////////////////
// Simulated Car Driver
///////////////////////////////////////////////////////////////////////////////
class SimulatedCarDriver : public CarDriverInterface
{
    public:
	SimulatedCarDriver();
        virtual ~SimulatedCarDriver() {}

        // Declarations of implementations of Generic Car Driver Interface's pure virtual functions
	bool Initialize( void );
	bool ApplyCommand( void );

	// Simulated Car Driver-specific function declarations
	bool ApplyCommand( float flVelocity, float flSteering );

	// Declarations and definitions of getters and setters for member variables
        bool GetInitialized( void ) { return m_bIsInitialized; }
        void SetInitialized( bool bIsInitialized ) { m_bIsInitialized = bIsInitialized; }
        float GetVelocity( void ) { return m_flVelocity; }
        void SetVelocity( float flVelocity ) { m_flVelocity = flVelocity; }
        float GetSteering( void ) { return m_flSteering; }
        void SetSteering( float flSteering ) { m_flSteering = flSteering; }

    private:
	// Member variable declarations
	bool m_bIsInitialized;	// have we been initialized yet?
        float m_flVelocity;
	float m_flSteering;
};

#endif	// SIMULATED_CAR_DRIVER_H

