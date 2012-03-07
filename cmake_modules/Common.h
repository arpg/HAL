/*
 * File:   Common.h
 * Author: jmf
 *
 * Created on February 17, 2012, 10:51 AM
 */

#ifndef COMMON_H
#define	COMMON_H

#include "MochaConfig.h"
#include <eigen3/Eigen/Eigen>
#include <sys/time.h>

extern MochaConfig g_MochaConfig;

enum eLocType { VT_AIR, VT_GROUND, VT_REC_RAMP, VT_CIR_RAMP };

namespace Eigen{
typedef Matrix<double,5,1> Vector5d ;
typedef Matrix<double,6,1> Vector6d ;
}

// Sensor IDs as per Vicon Tracker
// The vehicle should always be 0
// The rectangular ramps should be next
// The circular ramps should be last
#define CAR 0
#define RAMP1 1     // rectangular ramp
#define RAMP2 2     // circular ramp
#define RAMP3 3     // circular ramp
#define RAMP4 4     // circular ramp

// Aux Time Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline double Tic()
{
    struct timeval tv;
    gettimeofday(&tv, 0);
    return tv.tv_sec + 1e-6 * (tv.tv_usec);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline double RealTime()
{
    return Tic();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline double Toc( double  dTic )
{
    return Tic() - dTic;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline double TocMS( double  dTic )
{
    return ( Tic() - dTic )*1000.;
}



#endif	/* COMMON_H */
