/* 
 * File:   TicToc.h
 * Author: guest
 *
 * Created on April 23, 2012, 9:45 PM
 */

#ifndef TICTOC_H
#define	TICTOC_H

#include <sys/time.h>

////////////////////////////////////////////////////////////////////////////////
// Aux Time Functions
inline double Tic()
{
    struct timeval tv;
    gettimeofday(&tv, 0);
    return tv.tv_sec + 1e-6 * (tv.tv_usec);
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


#endif	/* TICTOC_H */

