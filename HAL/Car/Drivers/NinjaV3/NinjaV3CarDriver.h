#pragma once

#include <HAL/Car/CarDriverInterface.h>
#include <HAL/Utils/Uri.h>
#include "ComDriver.h"
#include <string>
#include <thread>

namespace hal {

class NinjaV3CarDriver : public CarDriverInterface {
 public:
  NinjaV3CarDriver(std::string dev,int baud);
  virtual ~NinjaV3CarDriver();
  bool SendCarCommand(CarCommandMsg &command_msg) override;
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
  hal::CarStateMsg pbStateMsg_;
  hal::VectorMsg* pbStateMsg_encoders_;
  hal::VectorMsg* pbStateMsg_swing_angles_;
  SensorPacket sensor_packet_;
  std::string packet_delimiter_;
  bool com_connected;
};
}
