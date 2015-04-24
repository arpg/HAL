#include <iostream>

#include <android/log.h>
#include <android/looper.h>
#include <android/sensor.h>

#include <HAL/Utils/TicToc.h>
#include "./AndroidIMUDriver.h"

#define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, "AndroidIMUDriver", __VA_ARGS__))
#define LOGW(...) ((void)__android_log_print(ANDROID_LOG_WARN, "AndroidIMUDriver", __VA_ARGS__))

using namespace hal;

const int DEFAULT_PACKET_TIMEOUT_MS = 1000;
const double GRAVITY_MAGNITUDE = 9.80665;

// This resides in native_app_glue.h, which we don't have right now
enum {LOOPER_ID_USER = 3};

AndroidIMUDriver::AndroidIMUDriver() : should_run_(false) {
  sensor_manager_ = ASensorManager_getInstance();
  accelerometer_ = ASensorManager_getDefaultSensor(sensor_manager_,
                                                   ASENSOR_TYPE_ACCELEROMETER);
  gyroscope_ = ASensorManager_getDefaultSensor(sensor_manager_,
                                               ASENSOR_TYPE_GYROSCOPE);
  magnetometer_ = ASensorManager_getDefaultSensor(sensor_manager_,
                                                  ASENSOR_TYPE_MAGNETIC_FIELD);
  device_thread_ = std::thread(std::bind(&AndroidIMUDriver::SensorLoop, this));
}

AndroidIMUDriver::~AndroidIMUDriver() {
  should_run_ = false;
  device_thread_.join();
}

void AndroidIMUDriver::SensorLoop() {
  event_queue_ = ASensorManager_createEventQueue(
      sensor_manager_,
      ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS),
      LOOPER_ID_USER, nullptr, nullptr);
  ASensorEventQueue_enableSensor(event_queue_, accelerometer_);
  ASensorEventQueue_enableSensor(event_queue_, gyroscope_);
  ASensorEventQueue_enableSensor(event_queue_, magnetometer_);

  ASensorEventQueue_setEventRate(event_queue_, accelerometer_, (1000L/60)*1000);
  ASensorEventQueue_setEventRate(event_queue_, gyroscope_, (1000L/60)*1000);
  ASensorEventQueue_setEventRate(event_queue_, magnetometer_, (1000L/60)*1000);

  hal::ImuMsg imu_msg;
  while (should_run_) {
    // Read all pending events.
    int ident;
    int events;
    while ((ident = ALooper_pollAll(-1, nullptr, &events, nullptr)) >= 0) {
      // If a sensor has data, process it now.
      if (ident != LOOPER_ID_USER) continue;

      ASensorEvent event;
      while (ASensorEventQueue_getEvents(event_queue_, &event, 1) > 0) {
        imu_msg.Clear();

        if (accelerometer_ && event.type == ASENSOR_TYPE_ACCELEROMETER) {
          hal::VectorMsg* accel = imu_msg.mutable_accel();
          accel->add_data(event.acceleration.x);
          accel->add_data(event.acceleration.y);
          accel->add_data(event.acceleration.z);
        } else if (gyroscope_ && event.type == ASENSOR_TYPE_GYROSCOPE) {
          hal::VectorMsg* gyro = imu_msg.mutable_gyro();
          gyro->add_data(event.uncalibrated_gyro.x_uncalib);
          gyro->add_data(event.uncalibrated_gyro.y_uncalib);
          gyro->add_data(event.uncalibrated_gyro.z_uncalib);
        } else if (magnetometer_ && event.type == ASENSOR_TYPE_MAGNETIC_FIELD) {
          hal::VectorMsg* mag = imu_msg.mutable_mag();
          mag->add_data(event.magnetic.x);
          mag->add_data(event.magnetic.y);
          mag->add_data(event.magnetic.z);
        }

        imu_msg.set_device_time(event.timestamp * 1e-9);
        if (imu_callback_) {
          imu_callback_(imu_msg);
        }
      }
    }
  }
  should_run_ = false;
}


void AndroidIMUDriver::RegisterIMUDataCallback(IMUDriverDataCallback callback) {
  imu_callback_ = callback;
}
