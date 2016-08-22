#include "NinjaV3CarDriver.h"
#include <stdlib.h>
#include <unistd.h>
#include <HAL/Devices/DeviceException.h>
#include <HAL/Car.pb.h>

namespace hal {

CarStateDataCallback NinjaV3CarDriver::car_state_callback = nullptr;
const int DEFAULT_COMPORT_WRITE_TIMEOUT_MS = 1000;
const int DEFAULT_COMPORT_READ_TIMEOUT_MS = 1000;

NinjaV3CarDriver::NinjaV3CarDriver(std::string dev, int baud) {
  baudrate_ = baud;
  dev_name_ = dev;
  com_connected = false;
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
  if( comport_driver_.Connect(dev_name_,baudrate_,sizeof(ComRecDataPack)) ) {
   com_connected = true;
   comport_write_thread = std::thread(std::bind(&NinjaV3CarDriver::ComportWriteThread, this));
   comport_read_thread = std::thread(std::bind(&NinjaV3CarDriver::ComportReadThread, this));
  }
  return com_connected;
}



void NinjaV3CarDriver::ComportWriteThread(){
  ComTrDataPack tr_data_pack;
  while(com_connected) {
//    tr_data_pack.motor_power_percent = (float)pbCommandMsg_
    // send packet type, type=1 means data
    char pack_type = 1;
    comport_driver_.WriteComPort((unsigned char*)&pack_type,1);
    if(DEFAULT_COMPORT_WRITE_TIMEOUT_MS) {
      std::this_thread::sleep_for(std::chrono::milliseconds(DEFAULT_COMPORT_WRITE_TIMEOUT_MS));
    }
  }
}

void NinjaV3CarDriver::ComportReadThread(){
  CarStateMsg msg;
  car_state_callback(msg);
  return;
  if(car_state_callback != nullptr) {
    while(com_connected) {
      int num_bytes =0;
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

void NinjaV3CarDriver::SendCarCommand(CarCommandMsg &command_msg){
  pbCommandMsg_ = command_msg;

}
}
