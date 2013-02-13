/*
 * These methods are used for devices that replay sensor information (ie. logs) and are used to
 * synchronize multiple devices through timestamps. The QUEUE holds the _next_ timestamp of the
 * sensor reading. The QUEUE is ordered non-decreasing, so whenever the time inserted is on top
 * of the QUEUE, then the sensor can deliver the reading.
 *
 * The general template to use this is as follows:
 *
 * - On the sensor's INIT function, read ahead the upcoming event's timestamp and use PushTime()
 *   to push it to the QUEUE.
 *
 * - In the CAPTURE method, lock_wait() until the timestamp on top of the QUEUE [accessed via
 *   NextTime()] matches the one you pushed. If it does, give reading to user and call
 *   PopAndPushTime() which will pop your time from the QUEUE and push in the NEW time of your
 *   NEXT event.
 *
 */

#ifndef VIRTUALDEVICE_H
#define VIRTUALDEVICE_H

#include <queue>

#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>


namespace VirtualDevice {


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Queue of Virtual Devices
extern std::priority_queue< double, std::vector<double>, std::greater<double> >     QUEUE;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Mutex Lock
extern boost::mutex                 MUTEX;
extern boost::condition_variable    CONDVAR;

/// Get time on top of queue
inline double NextTime()
{
    if( QUEUE.size() == 0 ) {
        return 0;
    } else {
        return QUEUE.top();
    }
}

inline void WaitForTime(double nextTime)
{
    boost::mutex::scoped_lock lock(MUTEX);

    // check if timestamp is the top of the queue
    while( NextTime() < nextTime ) {
        CONDVAR.wait( lock );
    }
    // sweet, we are good to go!
    lock.unlock();
}



/// Push time on top of queue
inline void PushTime( double T )
{
    // don't push in bad times
    // (0 is a special time when no timestamps are in use)
    if( T >= 0 ) {
        boost::mutex::scoped_lock lock(MUTEX);
        QUEUE.push( T );
    }
}


/// Pop top of queue and push new time
inline void PopAndPushTime( double T )
{
    // get lock
    boost::mutex::scoped_lock lock(MUTEX);

    // pop top of queue which is what got us the lock in the first place!
    QUEUE.pop();

    // don't push in bad times
    // (0 is a special time when no timestamps are in use)
    if( T >= 0 ) {
        QUEUE.push( T );
    }

    // notify waiting threads that a change in the QUEUE has occured
    CONDVAR.notify_all();
}



/// Pop time on top of queue. This should never be used unless in some really really special
/// circumstances. Use PopAndPushTime() instead!
inline void PopTime()
{
    boost::mutex::scoped_lock lock(MUTEX);
    QUEUE.pop();
}



}

#endif // VIRTUALDEVICE_H
