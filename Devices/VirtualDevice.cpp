#include "VirtualDevice.h"

#include <exception>
#include <queue>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

namespace VirtualDevice {

////////////////////////////////////////////////////////////////////////////////
/// Queue of Virtual Devices
std::priority_queue< double,
    std::vector<double>,
    std::greater<double> >    QUEUE;

////////////////////////////////////////////////////////////////////////////////
/// Mutex Lock
boost::mutex                  MUTEX;
boost::condition_variable     CONDVAR;

////////////////////////////////////////////////////////////////////////////////
/// Wait for correct time to elapse if REALTIME is true
bool                          REALTIME = false;

////////////////////////////////////////////////////////////////////////////////
/// TO_READ can be used to step through and pause events
unsigned long                 EVENTS_TO_QUEUE = std::numeric_limits<unsigned long>::max();

////////////////////////////////////////////////////////////////////////////////
inline bool IsPaused()
{
    return EVENTS_TO_QUEUE == 0;
}

////////////////////////////////////////////////////////////////////////////////
double NextTime()
{
    if( QUEUE.size() == 0 ) {
        return 0;
    } else {
        return QUEUE.top();
    }
}

////////////////////////////////////////////////////////////////////////////////
void WaitForTime(double nextTime)
{
    boost::mutex::scoped_lock lock(MUTEX);

    // check if timestamp is the top of the queue
    while( NextTime() < nextTime ) {
        CONDVAR.wait( lock );
    }
    
    // TODO: Sleep for appropriate amount of time.
    if(REALTIME) {
        boost::this_thread::sleep(boost::posix_time::milliseconds(1E3/1000.0));        
    }
}

////////////////////////////////////////////////////////////////////////////////
void PushTime( double T )
{
    // don't push in bad times
    // (0 is a special time when no timestamps are in use)
    if( T >= 0 ) {
        boost::mutex::scoped_lock lock(MUTEX);
        QUEUE.push( T );
    }
}

////////////////////////////////////////////////////////////////////////////////
void PopAndPushTime( double T )
{
    boost::mutex::scoped_lock lock(MUTEX);
    
    // Hold up the device at the top of the queue whilst time is 'paused'
    while(IsPaused()) {
        CONDVAR.wait( lock );        
    }

    // pop top of queue which is what got us the lock in the first place!
    QUEUE.pop();

    // don't push in bad times
    // (0 is a special time when no timestamps are in use)
    if( T >= 0 ) {
        QUEUE.push( T );
    }
    
    // Signify that event has been queued
    EVENTS_TO_QUEUE--;

    // notify waiting threads that a change in the QUEUE has occured
    CONDVAR.notify_all();
}

////////////////////////////////////////////////////////////////////////////////
void PopTime()
{
    boost::mutex::scoped_lock lock(MUTEX);
    QUEUE.pop();

    // notify waiting threads that a change in the QUEUE has occured
    CONDVAR.notify_all();
}

////////////////////////////////////////////////////////////////////////////////
void SetRealtime(bool realtime)
{
    REALTIME = realtime;
}

////////////////////////////////////////////////////////////////////////////////
void PauseTime()
{
    boost::mutex::scoped_lock lock(MUTEX);
    
    EVENTS_TO_QUEUE = 0;
}

////////////////////////////////////////////////////////////////////////////////
void UnpauseTime()
{
    boost::mutex::scoped_lock lock(MUTEX);

    EVENTS_TO_QUEUE = std::numeric_limits<unsigned long>::max();

    // notify waiting threads that a change in the QUEUE has occured
    CONDVAR.notify_all();    
}

////////////////////////////////////////////////////////////////////////////////
void TogglePauseTime()
{
    boost::mutex::scoped_lock lock(MUTEX);
    
    if(IsPaused()) {
        // unpause
        EVENTS_TO_QUEUE = std::numeric_limits<unsigned long>::max();
        CONDVAR.notify_all();    
    }else{
        // pause
        EVENTS_TO_QUEUE = 0;
    }
}

////////////////////////////////////////////////////////////////////////////////
void StepTime(int numEvents)
{
    boost::mutex::scoped_lock lock(MUTEX);
    EVENTS_TO_QUEUE = numEvents;    
    CONDVAR.notify_all();    
}




}
