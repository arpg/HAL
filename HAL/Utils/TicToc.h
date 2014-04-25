/*
 * File:   TicToc.h
 * Author: guest
 *
 * Created on April 23, 2012, 9:45 PM
 */

#ifndef TICTOC_H
#define TICTOC_H

#include <sys/time.h>
#include <time.h>

#ifdef __MACH__
#include <mach/clock.h>
#include <mach/mach_time.h>
#endif

namespace  hal {

/** High-precision timers.
 *
 * Not guaranteed to be relative to anything in particular.
 */
inline double Tic() {
#ifdef __MACH__
  // From Apple Developer Q&A @
  // https://developer.apple.com/library/mac/qa/qa1398/_index.html

  // This doesn't change, so we set it up once
  static mach_timebase_info_data_t timebase;
  if (timebase.denom == 0) {
    mach_timebase_info(&timebase);
  }

  double secs = static_cast<double>(mach_absolute_time()) *
      timebase.numer / timebase.denom * 1e-9;
  return secs;
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

/** Get the seconds since the Epoch */
inline double RealTime() {
#if _POSIX_TIMERS > 0
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec + ts.tv_nsec * 1e-9;
#else
  struct timeval tv;
  gettimeofday(&tv, 0);
  return tv.tv_sec + 1e-6 * (tv.tv_usec);
#endif
}

/** Time since the time given by a Tic() call */
inline double Toc(double dTic) {
    return Tic() - dTic;
}

inline double TocMS(double dTic) {
  return (Tic() - dTic)*1000.;
}
}  // namespace hal

#endif	/* TICTOC_H */
