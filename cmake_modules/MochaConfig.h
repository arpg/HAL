/*
 * File:   MochaConfig.h
 * Author: jmf
 *
 * Created on February 16, 2012, 5:17 PM
 */

#ifndef MOCHACONFIG_H
#define	MOCHACONFIG_H

//#include <CVars/CVar.h>


class MochaConfig
{
public:
    MochaConfig()
        /*
        :
    m_dZThreshold( CVarUtils::CreateCVar<>( "tracker.zThreshold", 0.25, "Threshold to consider vehicle still on ground.") ),
    m_nNumRecRamps( CVarUtils::CreateCVar<>( "tracker.NumRecRamps", 0u, "Number of rectangular ramps." ) ),
    m_nNumCirRamps( CVarUtils::CreateCVar<>( "tracker.NumCirRamps", 0u, "Number of circular ramps." ) ),
    m_dRampWidth( CVarUtils::CreateCVar<>( "tracker.RampWidth", 0.4, "Width of rectangular ramp (in meters)." ) ),
    m_dRampHeight( CVarUtils::CreateCVar<>( "tracker.RampHeight", 0.5, "Height of rectangular ramp (in meters)." ) ),
    m_dRampRadius( CVarUtils::CreateCVar<>( "tracker.RampRadius", 0.5, "Radius of circular ramps (in meters)." ) )
*/
    {
        m_dZThreshold = 0.25;
    }
    double             m_dZThreshold;
    unsigned int       m_nNumRecRamps;
    unsigned int       m_nNumCirRamps;
    double             m_dRampWidth;
    double             m_dRampHeight;
    double             m_dRampRadius;

};


#endif	/* MOCHACONFIG_H */

