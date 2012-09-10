/*
    \file Defines the interface all IMU drivers must honor.
*/


#ifndef _IMU_DRIVER_H_
#define _IMU_DRIVER_H_

#include <Eigen/Eigen>
#include <RPG/Utils/PropertyMap.h>
#include <boost/function.hpp>


namespace Eigen
{
    typedef Matrix<double,6,1> Vector6d;
}

enum IMUDataType
{
    IMU_GPS_LLH          = 0,
    IMU_AHRS_QUATERNION  = 2,
    IMU_AHRS_EULER       = 4,
    IMU_AHRS_ACCEL       = 8,
    IMU_AHRS_GYRO        = 16,
    IMU_AHRS_MAG         = 32
};

struct IMUData {
    IMUDataType        type;
    Eigen::Vector6d    llh;         // (latitude,longitude,height)
    Eigen::Quaterniond rotation;    
    Eigen::Vector3f    euler;       // (roll,pitch,yaw)
    Eigen::Vector3f    accel;
    Eigen::Vector3f    gyro;
    Eigen::Vector3f    mag;
};

typedef boost::function<void (const IMUData&)> IMUDriverDataCallback;

///////////////////////////////////////////////////////////////////////////////
// Generic IMU driver interface
class IMUDriver
{
    public:
        // Pure virtual functions driver writers must implement:
        virtual void RegisterDataCallback(IMUDriverDataCallback callback) = 0;
        virtual bool Init() = 0;

        IMUDriver()
        {
            m_pPropertyMap = NULL;
        }

        // Called by IMUDevice::InitDriver
        void SetPropertyMap( PropertyMap* pMap )
        {
            m_pPropertyMap = pMap;
        }
    protected:
        PropertyMap* m_pPropertyMap; // from parent device that instantiates this driver.
};
#endif
