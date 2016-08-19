#include "NinjaV3CarDriver.h"
#include <stdlib.h>
#include <unistd.h>
#include <HAL/Devices/DeviceException.h>
#include <HAL/Car.pb.h>

namespace hal {

CarStateDataCallback NinjaV3CarDriver::car_state_callback = nullptr;
const int DEFAULT_COMPORT_WRITE_TIMEOUT_MS = 1000;
const int DEFAULT_COMPORT_READ_TIMEOUT_MS = 1000;
const int num_wheels = 4;

NinjaV3CarDriver::NinjaV3CarDriver(std::string dev, int baud) {
  baudrate_ = baud;
  dev_name_ = dev;
  packet_delimiter_ = 0xAA55AA55;
  com_connected = false;
  pbStateMsg_encoders_ = pbStateMsg_.mutable_encoders();
  pbStateMsg_swing_angles_ = pbStateMsg_.mutable_swing_angles();
  if(Init() == false) {
    throw DeviceException("Error connecting to Comport device.");
  }
}

NinjaV3CarDriver::~NinjaV3CarDriver() {
  comport_read_thread.join();
  comport_write_thread.join();
}

void NinjaV3CarDriver::RegisterCarStateDataCallback(CarStateDataCallback callback) {
  car_state_callback = callback;
}
bool NinjaV3CarDriver::Init(){
  if( comport_driver_.Connect(dev_name_,baudrate_) ) {
   com_connected = true;
   // create data fields for encoder and swing_angle sensors
   for (size_t ii = 0; ii < num_wheels; ii++) {
     pbStateMsg_encoders_->add_data(0);
     pbStateMsg_swing_angles_->add_data(0);
   }
   comport_write_thread = std::thread(std::bind(&NinjaV3CarDriver::ComportWriteThread, this));
   comport_read_thread = std::thread(std::bind(&NinjaV3CarDriver::ComportReadThread, this));
  }
  return com_connected;
}



void NinjaV3CarDriver::ComportWriteThread(){
  std::string msg;
  while(com_connected) {
    // Calculate and add Checksum
    pbCommandMsg_.checksum = comport_driver_._CalcChksum(msg.c_str(),pbCommandMsg_.ByteSize());
    msg.clear();
    pbCommandMsg_.AppendToString(msg);
    // send the command
    comport_driver_._WriteComPort(packet_delimiter_.c_str(),packet_delimiter_.size());
    comport_driver_._WriteComPort(msg.c_str(),pbCommandMsg_.ByteSize());
    std::cout << "car command sent ." << std::endl;
    if(DEFAULT_COMPORT_WRITE_TIMEOUT_MS) {
      std::this_thread::sleep_for(std::chrono::milliseconds(DEFAULT_COMPORT_WRITE_TIMEOUT_MS));
    }
  }
}

void NinjaV3CarDriver::ComportReadThread(){
  CarStateMsg msg;
  msg.set_device_id(2);
  car_state_callback(msg);
  return;
  if(car_state_callback != nullptr) {
    while(com_connected) {
      int num_bytes = comport_driver_.ReadSensorPacket(sensor_packet_);
      if(num_bytes) {
#warning "do something for reading, if there was a data received then trigger the callback "
      }
      if(DEFAULT_COMPORT_READ_TIMEOUT_MS) {
        std::this_thread::sleep_for(std::chrono::milliseconds(DEFAULT_COMPORT_READ_TIMEOUT_MS));
      }
    }
  } else {
    std::cout << "Warning: RegisterCarStateDataCallback() has not been registered." <<std::endl;
  }
}

bool NinjaV3CarDriver::SendCarCommand(CarCommandMsg &command_msg){
  pbCommandMsg_ = command_msg;
}
