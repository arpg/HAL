#ifndef PHIDGETSDRIVER_H
#define PHIDGETSDRIVER_H

#include <boost/thread.hpp>
#include <phidget21.h>
#include "RPG/Devices/IMU/IMUDriverInterface.h"

int CCONV AttachHandler(CPhidgetHandle spatial, void *userptr);
int CCONV DetachHandler(CPhidgetHandle spatial, void *userptr);
int CCONV ErrorHandler(CPhidgetHandle spatial, void *userptr, int ErrorCode, const char *unknown);
int CCONV SpatialDataHandler(CPhidgetSpatialHandle spatial, void *userptr, CPhidgetSpatial_SpatialEventDataHandle *data, int count);

class PhidgetsDriver : public IMUDriver
{
public:
    PhidgetsDriver();
    ~PhidgetsDriver();
    bool Init();
    void RegisterDataCallback(IMUDriverDataCallback callback);
    void RegisterDataCallback(GPSDriverDataCallback /*callback*/) { }
    void _AttachHandler(CPhidgetHandle spatial);
    void _DetachHandler(CPhidgetHandle spatial);
    void _ErrorHandler(CPhidgetHandle spatial, int ErrorCode, const char *unknown);
    void _SpatialDataHandler(CPhidgetSpatialHandle spatial, CPhidgetSpatial_SpatialEventDataHandle *data, int count);
private:
    CPhidgetSpatialHandle m_hSpatial;
    IMUDriverDataCallback m_ImuCallback;
};

#endif // PHIDGETSDRIVER_H
