#pragma once

/* Header file to enable threading and ergo callback */
#include <thread>

/* Header file to make it in image defined by HAL */
#include <HAL/LIDAR/LIDARDriverInterface.h>

/* Header files for socket variable */
#include <sys/types.h>
#include <sys/socket.h>

#define BUFLEN 1248

namespace hal {

class VelodyneDriver : public LIDARDriverInterface
{
public:
    VelodyneDriver(int port=2368);
    ~VelodyneDriver();
    void RegisterLIDARDataCallback(LIDARDriverDataCallback callback);

private:
    void _ThreadFunc();

private:
    /* Variable for HAL compatibility */
    bool                    m_running;
    std::thread             m_callbackThread;
    LIDARDriverDataCallback   m_callback;

    /* Velodyne specific Variables */
    int			    m_port;
    int			    m_socketDescriptor;
};

} /* namespace */
