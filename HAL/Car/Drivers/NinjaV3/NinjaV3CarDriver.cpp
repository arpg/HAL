#include "NinjaV3CarDriver.h"
#include <stdlib.h>
#include <unistd.h>
#include <HAL/Devices/DeviceException.h>
#include <HAL/Car.pb.h>

namespace hal {

CarStateDataCallback NinjaV3CarDriver::car_state_callback = nullptr;
const int DEFAULT_COMPORT_WRITE_TIMEOUT_MS = 1000;
const int DEFAULT_COMPORT_READ_TIMEOUT_MS = 1000;

NinjaV3CarDriver::NinjaV3CarDriver(const Uri& uri) {
  dev_name_ = uri.properties.Get<std::string>("dev", "/dev/cu.USB0");
  dev_baudrate_ = std::stoi(uri.properties.Get<std::string>("baud", "11500"));
  com_connected = false;
}

NinjaV3CarDriver::~NinjaV3CarDriver() {
  comport_read_thread.join();
  comport_write_thread.join();
}

void NinjaV3CarDriver::RegisterCarStateDataCallback(CarStateDataCallback callback) {
  car_state_callback = callback;
  if(Init(dev_name_,dev_baudrate_) == false) {
    throw DeviceException("Error connecting to Ftdi device.");
  }
}

bool NinjaV3CarDriver::Init(std::string port, int baudrate){
   if( comport_driver_.Connect(port,baudrate) ) {
     com_connected = true;
     comport_write_thread = std::thread(std::bind(&NinjaV3CarDriver::ComportWriteThread, this));
     comport_read_thread = std::thread(std::bind(&NinjaV3CarDriver::ComportReadThread, this));
   }
   return true;
}

void NinjaV3CarDriver::ComportWriteThread(){
  comport_driver_.SendCommandPacket(command_packet_.m_nSpeed,command_packet_.m_nSteering);
  while(com_connected) {
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
#warning "update the command_packet_ here. then packet will be sent by write thread"
  pbCommandMsg = command_msg;
  if(!car_state_callback) {
    throw DeviceException("CarStateDataCallBack has not been registered.");
  }

}
}
