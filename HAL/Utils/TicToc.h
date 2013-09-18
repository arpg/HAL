/*
 * File:   TicToc.h
 * Author: guest
 *
 * Created on April 23, 2012, 9:45 PM
 */

#ifndef TICTOC_H
#define	TICTOC_H

#include <sys/time.h>
#include <time.h>

#ifdef __MACH__
#include <mach/clock.h>
#include <mach/mach.h>
#endif

namespace  hal
{

////////////////////////////////////////////////////////////////////////////////
// Aux Time Functions
inline double Tic()
{
#ifdef __MACH__
    clock_serv_t cclock;
    mach_timespec_t mts;
    host_get_clock_service(mach_host_self(), SYSTEM_CLOCK, &cclock);
    clock_get_time(cclock, &mts);
    mach_port_deallocate(mach_task_self(), cclock);

    return mts.tv_sec + mts.tv_nsec * 1e-9;
#elif _POSIX_TIMERS > 0
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
#else
    struct timeval tv;
    gettimeofday(&tv, 0);
    return tv.tv_sec + 1e-6 * (tv.tv_usec);
#endif
}

////////////////////////////////////////////////////////////////////////////////
inline double RealTime()
{
    return Tic();
}

////////////////////////////////////////////////////////////////////////////////
inline double Toc( double  dTic )
{
    return Tic() - dTic;
}

////////////////////////////////////////////////////////////////////////////////
inline double TocMS( double  dTic )
{
    return ( Tic() - dTic )*1000.;
}

}

#endif	/* TICTOC_H */
