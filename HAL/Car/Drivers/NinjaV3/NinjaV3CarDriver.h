#pragma once

#include <HAL/Car/CarDriverInterface.h>
#include <HAL/Utils/Uri.h>
#include "ComDriver.h"
#include <string>
#include <thread>
#include <mutex>

//#define DELIMITER0 (char)0xAA
//#define DELIMITER1 (char)0x55
//#define DELIMITER2 (char)0xE1
//#define DELIMITER3 (char)0x1E
#define DELIMITER0 0xAA
#define DELIMITER1 0x55
#define DELIMITER2 0xE1
#define DELIMITER3 0x1E

namespace hal {

class NinjaV3CarDriver : public CarDriverInterface {
 public:
  NinjaV3CarDriver(std::string dev,int baud);
  virtual ~NinjaV3CarDriver();
  void UpdateCarCommand(CarCommandMsg &command_msg) override;
  void RegisterCarStateDataCallback(CarStateDataCallback callback);

 private:
  ComportDriver       comport_driver_;
  std::string         dev_name_;
  int                 baudrate_;
  static CarStateDataCallback car_state_callback;
  std::thread comport_write_thread;
  std::thread comport_read_thread;
  bool Init();
  void ComportWriteThread();
  void ComportReadThread();
  hal::CarCommandMsg pbCommandMsg_;
  std::string packet_delimiter_;
  int serial_buffer_write_delay_;
  bool com_connected;
  std::mutex tr_mutex;
  struct ComTrDataPack{
    char delimiter[4];
    float  steering_angle;
    float  motor_power_percent;
    unsigned int dev_time;
    unsigned int chksum;
    ComTrDataPack(){
      delimiter[0] = DELIMITER0;
      delimiter[1] = DELIMITER1;
      delimiter[2] = DELIMITER2;
      delimiter[3] = DELIMITER3;
    }
  };
  struct ComRecDataPack
  {
    char delimiter[4];
    float  enc0;
    float  enc1;
    float  enc2;
    float  enc3;
    float   steer_ang;
    float   swing_ang0;
    float   swing_ang1;
    float   swing_ang2;
    float   swing_ang3;
    float   motor_current;
    unsigned int dev_time;
    unsigned int chksum;
  };

};
}
