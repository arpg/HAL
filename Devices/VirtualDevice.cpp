
#include "VirtualDevice.h"


boost::mutex                                                                VirtualDevice::MUTEX;
boost::condition_variable                                                   VirtualDevice::CONDVAR;
std::priority_queue< double, std::vector<double>, std::greater<double> >    VirtualDevice::QUEUE;
