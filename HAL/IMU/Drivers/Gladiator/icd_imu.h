#ifndef __ICD_IMU_H_
#define __ICD_IMU_H_
#include <stdint.h>

/* The same imu_data_t is used for both ADIS and Gladiator output
   Originally developed at Astrobotic (https://www.astrobotic.com). Used with permission
 */

typedef struct 
{
   /*note: timespec is platform dependent, 8 bytes on ARM32, 16 on x86_64, ? on i686
    The idea is to use a x86_64 timespec, with the 32-bit platforms type promoted to fit
    */
  uint64_t tv_sec;
  uint64_t tv_nsec;
  int32_t accel_x; /* Accel units are in 10000-g */
  int32_t accel_y;
  int32_t accel_z;
  int32_t gyro_x; /* Gyro units are in 1000-deg/s */
  int32_t gyro_y;
  int32_t gyro_z;
  int16_t temp; /* Temp is centi-deg C */
} __attribute__((packed)) imu_data_t;


#endif
