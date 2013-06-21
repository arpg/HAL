#pragma once

#include <Eigen/Eigen>
#include <functional>

#include <HAL/Devices/DriverInterface.h>

namespace Eigen
{
    typedef Matrix<double,6,1> Vector6d;
} /* namespace */

namespace hal {

enum IMUDataType
{
    IMU_AHRS_TIMESTAMP_PPS = 1,
    IMU_AHRS_QUATERNION    = 2,
    IMU_AHRS_EULER         = 4,  // rad
    IMU_AHRS_ACCEL         = 8,  // g
    IMU_AHRS_GYRO          = 16, // rad s^-1
    IMU_AHRS_MAG           = 32
};

enum GPSDataType
{
    IMU_GPS_LLH            = 1
};

struct IMUData
{
    IMUData()
        : timestamp_system(0), data_present((IMUDataType)0)
    {}

    double             timestamp_system; // System time - Always present [s]
    int                data_present;     // Bitmask of which data is present

    double             timestamp_pps;    // AHRS accurate timestamp [s]
    Eigen::Quaterniond rotation;         // Absolute Orientation
    Eigen::Vector3f    euler;            // Absolute Orientation (roll,pitch,yaw) [rad]
    Eigen::Vector3f    accel;            // Body Acceleration [m/s^2]
    Eigen::Vector3f    gyro;             // Body Angular velocity [rad/s]
    Eigen::Vector3f    mag;              // Magnetomitor Vector [Guass]
};

struct GPSData
{
    GPSData()
        : timestamp_system(0), data_present((GPSDataType)0)
    {}

    double             timestamp_system; // System time - Always present [s]
    int                data_present;     // Bitmask of which data is present

    Eigen::Vector6d    llh;              // (latitude,longitude,height)
};

typedef std::function<void (const IMUData&)> IMUDriverDataCallback;
typedef std::function<void (const GPSData&)> GPSDriverDataCallback;

///////////////////////////////////////////////////////////////////////////////
// Generic IMU driver interface
class IMUDriverInterface : public DriverInterface
{
    public:
        // Pure virtual functions driver writers must implement:
        virtual ~IMUDriverInterface() {}
        virtual void RegisterDataCallback(IMUDriverDataCallback callback) = 0;
        virtual void RegisterDataCallback(GPSDriverDataCallback callback) = 0;
};

} /* namespace */
