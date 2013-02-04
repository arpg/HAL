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


/// Push time on top of queue
inline void PushTime( double T )
{
//    std::cout.precision(15);
//    std::cout << "Initial Push Time: " << std::fixed << T << std::endl;

    // don't push in bad times
    if( T >= 0 ) {
        boost::mutex::scoped_lock lock(MUTEX);
        QUEUE.push( T );
    }
//    std::cout << "-------- Top of Queue[" << QUEUE.size() << "] is: " << std::fixed << NextTime() << std::endl;
}


/// Pop top of queue and push new time
inline void PopAndPushTime( double T )
{
//    std::cout.precision(15);
//    std::cout << "Pushing Time: " << std::fixed << T << std::endl;

    boost::mutex::scoped_lock lock(MUTEX);
    QUEUE.pop();

    // don't push in bad times
    if( T >= 0 ) {
        QUEUE.push( T );
    }
//    std::cout << "-------- Top of Queue[" << QUEUE.size() << "] is: " << std::fixed << NextTime() << std::endl;

    CONDVAR.notify_all();
}



/// Pop time on top of queue
inline void PopTime()
{
    boost::mutex::scoped_lock lock(MUTEX);
    QUEUE.pop();
}



}

#endif // VIRTUALDEVICE_H
