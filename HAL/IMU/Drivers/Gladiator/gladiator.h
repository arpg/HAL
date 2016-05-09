#ifndef __GLADIATOR_H_
#define __GLADIATOR_H_

#include <stdint.h>
#include "ThreadedObject.h"
#include "ThreadedCommand.h"
#include "commport.h"
#include "icd_imu.h"
#include <math.h>
/* Gladiator IMU LM Series 
   Based on sample code provided by Gladiator Technologies in their Software Interface Document, Rev 2012-11-06
   Originally developed at Astrobotic (https://www.astrobotic.com). Used with permission
*/



/* Struct pumped over the serial line to host */

typedef struct {
  uint8_t uSync;
  uint8_t uCount;
  int16_t iGyrX; // 0.01 deg/sec
  int16_t iGyrY; // 0.01 deg/sec
  int16_t iGyrZ; // 0.01 deg/sec
  int16_t iAccX; // 0.001 g
  int16_t iAccY; // 0.001 g
  int16_t iAccZ; // 0.001 g
  int16_t iTemp; // 0.01 deg C
  uint8_t uStatus;
  uint8_t uChecksum;
} __attribute__((packed)) gladiator_rx_msg_t;

// Note that the structure below has a checksum as the 4th byte, but
// this sum actually includes not only the 3 byte previous, but the 4 bytes
// following (payload). It is located where it is so that the float value will
// be 4-byte aligned with the structure. The checksum still works the same,
// i.e. when all 8 bytes are summed, the result should be zero.

typedef struct {
  uint8_t sync;
  uint8_t cmd[2];
  uint8_t cksum;
  uint32_t payload; // generally a float value, but in a few cases is a Uint32
} __attribute__((packed)) gladiator_tx_msg_t;

class GladiatorIMU;
enum GladiatorCommand {CMD_LOADK = 0, CMD_SETRATE, CMD_GETSTATUS, CMD_GETSERIAL, CMD_GETBOARD};
enum GladiatorMode {MODE_100HZ = 0x00010004, MODE_200HZ = 0x00010001};


class gladiator_cmd_t : public ThreadedCommand
{
 public:
  gladiator_cmd_t(GladiatorIMU *host, GladiatorCommand m_req, uint32_t value);
  ~gladiator_cmd_t();
  GladiatorCommand command;
  uint32_t payload;
};


class GladiatorIMU : public ThreadedObject
{
 public:
  GladiatorIMU(const char *serPort);
  ~GladiatorIMU();
  void run();


 private:
  void loadFilterK(float kVal);
  void setSampleRate(uint32_t mode);
  void pauseOutput();
  CommPort *port;
  void handleCommand(gladiator_cmd_t* cmd);
  imu_data_t* acquireData();
  gladiator_rx_msg_t* getSingleMessage();
  void sendMessage(gladiator_tx_msg_t *output );
  imu_data_t* translate(gladiator_rx_msg_t* input);
  void handleStatusByte(gladiator_rx_msg_t *msg);
  void setAccelScale(uint8_t bits);
  void setGyroScale(uint8_t bits);
  void makeAccel(int32_t src, int32_t* dest);
  void makeGyro(int32_t src, int32_t* dest);
  void makeTemp(int16_t src, int16_t* dest);
  int epsilonEqual( float a, float b, float epsilon );
  static const uint8_t msgSuccess[22];
  uint8_t revNumber[2]; //zero is major, one is minor
  uint8_t productCode;
  uint8_t releaseLevel;
  uint8_t filterNumber;
  uint8_t boardNumber[4];
  uint8_t serialNumber[4];
  uint8_t statusByte[2]; //index follows bit 6 (0=test info, 1=status)
  uint8_t gyroType;
  float accelScale;
  float gyroScale; 

};

#endif
