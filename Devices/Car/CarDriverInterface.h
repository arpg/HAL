/*
    \file Master interface for all car drivers
*/

#ifndef CAR_DRIVER_INTERFACE_H
#define CAR_DRIVER_INTERFACE_H

// each car driver can have a generic set of "properties"
#include "RPG/Utils/PropertyMap.h"

///////////////////////////////////////////////////////////////////////////////
// Generic Car Driver Interface
///////////////////////////////////////////////////////////////////////////////
class CarDriverInterface
{
    public:
	CarDriverInterface()
        {
            m_pPropertyMap = NULL;
        }
	virtual ~CarDriverInterface() {}

        // Pure virtual functions car drivers must implement
	virtual bool Init( void ) = 0;

    virtual bool ApplyCommand( float flVelocity, float flSteering ) = 0;

	// Getters and setters
        PropertyMap *GetPropertyMap( void ) { return m_pPropertyMap; }
        void SetPropertyMap( PropertyMap *pPropertyMap ) { m_pPropertyMap = pPropertyMap; }

    protected:
        PropertyMap *m_pPropertyMap; // set by car drivers that implement this interface
};

#endif	// CAR_DRIVER_INTERFACE_H

