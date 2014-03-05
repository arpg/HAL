#include <HAL/Devices/DeviceFactory.h>

#include "MicroStrainDriver.h"

namespace hal
{

class MicroStrainFactory : public DeviceFactory<IMUDriverInterface>
{
public:
    MicroStrainFactory(const std::string& name)
        : DeviceFactory<IMUDriverInterface>(name)
    {
        Params() = {
            {"accel", "1", "Capture accelerometer data."},
            {"gyro", "1", "Capture gyro data."},
            {"mag", "0", "Capture magnetometer data."},
            {"gps", "0", "Capture GPS data."},
            {"gpshz", "1", "GPS capture rate in Hz."},
            {"imuhz", "200", "IMU capture rate in Hz."}
        };
    }

    std::shared_ptr<IMUDriverInterface> GetDevice(const Uri& uri)
    {
        bool capture_accel = uri.properties.Get("accel", true);
        bool capture_gyro = uri.properties.Get("gyro", true);
        bool capture_mag = uri.properties.Get("mag", false);
        bool capture_gps = uri.properties.Get("gps", false);
        int gps_hz = uri.properties.Get("gpshz", 1);
        int imu_hz = uri.properties.Get("imuhz", 200);

        MicroStrainDriver* pDriver =
            new MicroStrainDriver(uri.url, capture_accel, capture_gyro,
                                  capture_mag, capture_gps, gps_hz, imu_hz);
        return std::shared_ptr<IMUDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static MicroStrainFactory g_MicroStrainFactory("microstrain");

}
