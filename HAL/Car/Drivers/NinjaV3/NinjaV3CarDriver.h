#pragma once

#include <HAL/Car/CarDriverInterface.h>
#include <HAL/Utils/Uri.h>
#include "FtdiDriver.h"
#include <string>
#include <thread>

namespace hal {

class NinjaV3CarDriver : public CarDriverInterface {
 public:
  NinjaV3CarDriver(const hal::Uri &uri);
  virtual ~NinjaV3CarDriver();
  bool ApplyCommand(double motor_current, double steering) override;
  void RegisterCarStateDataCallback(CarStateDriverDataCallback callback);

 private:
  ComportDriver       comport_driver_;
  std::string         dev_name_;
  int                 dev_baudrate_;
  static CarStateDriverDataCallback car_state_callback;
  std::thread comport_write_thread;
  std::thread comport_read_thread;
  bool Init(std::string port, int baudrate);
  void ComportWriteThread();
  void ComportReadThread();
  hal::VectorMsg *pbEncoderMsg;
  hal::VectorMsg *pbSwingAngleMsg;
  hal::CarCommandMsg pbCommandMsg;
  hal::CarStateMsg pbStateMsg;
  SensorPacket sensor_packet_;
  CommandPacket command_packet_;
};
}
