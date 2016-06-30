#include "DeviceTime.h"

// Hack to enable sleep_for (GCC < 4.8)
#ifndef _GLIBCXX_USE_NANOSLEEP
#define _GLIBCXX_USE_NANOSLEEP
#endif  // _GLIBCXX_USE_NANOSLEEP

#include <exception>
#include <queue>
#include <atomic>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <iostream>

namespace hal {
namespace DeviceTime {

////////////////////////////////////////////////////////////////////////////////
/// Queue of Virtual Devices
static std::priority_queue<double,
                           std::vector<double>,
                           std::greater<double> > QUEUE;

////////////////////////////////////////////////////////////////////////////////
/// Mutex Lock
static std::mutex MUTEX;
static std::condition_variable CONDVAR;

////////////////////////////////////////////////////////////////////////////////
/// Wait for correct time to elapse if REALTIME is true
static bool REALTIME = false;

////////////////////////////////////////////////////////////////////////////////
/// TO_READ can be used to step through and pause events
static std::atomic<uint64_t> EVENTS_TO_QUEUE(
    std::numeric_limits<uint64_t>::max());

////////////////////////////////////////////////////////////////////////////////
inline bool IsPaused()
{
    return EVENTS_TO_QUEUE == 0;
}

////////////////////////////////////////////////////////////////////////////////
void ResetTime()
{
    std::lock_guard<std::mutex> lock(MUTEX);
    // clear queue
    QUEUE = std::priority_queue< double, std::vector<double>, std::greater<double> >();
}

////////////////////////////////////////////////////////////////////////////////
double NextTime()
{
    //std::lock_guard<std::mutex> lock(MUTEX);
    if( QUEUE.empty() ) {
        return 0;
    } else {
        return QUEUE.top();
    }
}

////////////////////////////////////////////////////////////////////////////////
void WaitForTime(double nextTime)
{
    // check if timestamp is the top of the queue
    // if not, wait until the older timestamp is popped by another thread.
    std::unique_lock<std::mutex> lock(MUTEX);
    CONDVAR.wait( lock, [=]{return  NextTime() >= nextTime;});

    // TODO: Sleep for appropriate amount of time.
    if(REALTIME) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

////////////////////////////////////////////////////////////////////////////////
void PushTime( double T )
{
    // don't push in bad times
    // (0 is a special time when no timestamps are in use)
    if( T >= 0 ) {
        std::lock_guard<std::mutex> lock(MUTEX);
        QUEUE.push( T );
    }
}

////////////////////////////////////////////////////////////////////////////////
void PopAndPushTime( double T )
{
    std::unique_lock<std::mutex> lock(MUTEX);

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
    std::lock_guard<std::mutex> lock(MUTEX);
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
    std::lock_guard<std::mutex> lock(MUTEX);

    EVENTS_TO_QUEUE = 0;
}

////////////////////////////////////////////////////////////////////////////////
void UnpauseTime()
{
    std::lock_guard<std::mutex> lock(MUTEX);

    EVENTS_TO_QUEUE = std::numeric_limits<uint64_t>::max();

    // notify waiting threads that a change in the QUEUE has occured
    CONDVAR.notify_all();
}

////////////////////////////////////////////////////////////////////////////////
void TogglePauseTime()
{
    std::lock_guard<std::mutex> lock(MUTEX);

    if(IsPaused()) {
        // unpause
        EVENTS_TO_QUEUE = std::numeric_limits<uint64_t>::max();
        CONDVAR.notify_all();
    }else{
        // pause
        EVENTS_TO_QUEUE = 0;
    }
}

////////////////////////////////////////////////////////////////////////////////
void StepTime(int numEvents)
{
    std::lock_guard<std::mutex> lock(MUTEX);
    EVENTS_TO_QUEUE = numEvents;
    CONDVAR.notify_all();
}




}
}
