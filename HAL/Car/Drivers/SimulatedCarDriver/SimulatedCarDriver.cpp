/*
    \file Simulated car driver definition
*/

#include "SimulatedCarDriver.h"

SimulatedCarDriver::SimulatedCarDriver()
{
	m_bIsInitialized = false;
	m_flVelocity = 0.0f;
	m_flSteering = 0.0f;
}

bool SimulatedCarDriver::Initialize( void )
{
	assert( m_pPropertyMap );
	m_pPropertyMap->PrintPropertyMap();

	// Read from property map, which should contain everything we need to initialize our variables
        m_flVelocity = m_pPropertyMap->GetProperty<float>( "flVelocity", 0.0f );
        m_flSteering = m_pPropertyMap->GetProperty<float>( "flSteering", 0.0f );

	m_bIsInitialized = true;
	return true;
}

bool SimulatedCarDriver::ApplyCommand( float flVelocity, float flSteering )
{
	// Make sure we've been initialized first
	if ( !m_bIsInitialized )
		return false;

	// TODO: Do something?
}
