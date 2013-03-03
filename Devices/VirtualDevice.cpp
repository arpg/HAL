#include "VirtualDevice.h"

#include <exception>
#include <queue>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

namespace VirtualDevice {

////////////////////////////////////////////////////////////////////////////////
/// Queue of Virtual Devices
std::priority_queue< double, std::vector<double>, std::greater<double> >  QUEUE;

////////////////////////////////////////////////////////////////////////////////
/// Mutex Lock
boost::mutex                                                              MUTEX;
boost::condition_variable                                                 CONDVAR;

////////////////////////////////////////////////////////////////////////////////
/// Playback
bool                                                                      REALTIME = false;

////////////////////////////////////////////////////////////////////////////////
/// Rogue time value representing paused playback
const double PAUSED_VALUE = -std::numeric_limits<double>::max();

////////////////////////////////////////////////////////////////////////////////
inline bool IsPaused()
{
    return (QUEUE.size() > 0 && QUEUE.top() == PAUSED_VALUE);
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
    
    // sweet, we are good to go!
    lock.unlock();
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
    // get lock
    boost::mutex::scoped_lock lock(MUTEX);
    
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

    // Push dummy time that exists before any other event
    QUEUE.push( PAUSED_VALUE );
}

////////////////////////////////////////////////////////////////////////////////
void UnpauseTime()
{
    boost::mutex::scoped_lock lock(MUTEX);
    
    // Check that time was actually paused
    if(NextTime() != PAUSED_VALUE ) {
        std::cerr << "Unpause called without corresponding pause" << std::endl;
        throw std::exception();
    }
    
    QUEUE.pop();

    // notify waiting threads that a change in the QUEUE has occured
    CONDVAR.notify_all();    
}

////////////////////////////////////////////////////////////////////////////////
void TogglePauseTime()
{
    boost::mutex::scoped_lock lock(MUTEX);
    
    if(NextTime() == PAUSED_VALUE) {
        // unpause
        QUEUE.pop();
        CONDVAR.notify_all();            
    }else{
        // pause
        QUEUE.push( PAUSED_VALUE );
    }
}



}
