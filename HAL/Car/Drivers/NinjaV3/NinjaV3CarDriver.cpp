#include "NinjaV3CarDriver.h"
#include <stdlib.h>
#include <unistd.h>
#include <HAL/Devices/DeviceException.h>
#include <HAL/Car.pb.h>

namespace hal {

CarStateDataCallback NinjaV3CarDriver::car_state_callback = nullptr;

NinjaV3CarDriver::NinjaV3CarDriver(std::string dev, int baud) {
  baudrate_ = baud;
  dev_name_ = dev;
  com_connected = false;
  // caculate the time to wait after writing to send buffer
  serial_buffer_write_delay_ = (int)((sizeof(ComTrDataPack)*10*1000)/baud)+1;
  if(Init() == false) {
    throw DeviceException("Could not connect to Comport device.");
  } else {
    std::cout << "Connection to NinjaV3Car successfull." << std::endl;
  }
}

NinjaV3CarDriver::~NinjaV3CarDriver() {
  if(com_connected) {
    comport_driver_.Disconnect();
  }
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
    {
    std::lock_guard<std::mutex> mutex_lock(tr_mutex);
    tr_data_pack.motor_power_percent = (float)pbCommandMsg_.throttle_percent();
    tr_data_pack.steering_angle = (float)pbCommandMsg_.steering_angle();
    std::cout << "steering is -> " << tr_data_pack.steering_angle << std::endl;
    tr_data_pack.dev_time = (float)pbCommandMsg_.device_time();
    }
    // TODO: dev_time should be set to correct system time
    // calculate checksum from whole packet
    unsigned int chksum = 0;
    uint8_t* data = (uint8_t*)(&tr_data_pack);
    for(int ii=0;ii<(int)sizeof(ComTrDataPack)-4;ii++){
      chksum += data[ii];
    }
    // send packet
    comport_driver_.WriteComPort((uint8_t*)(&tr_data_pack),sizeof(ComTrDataPack));
    std::this_thread::sleep_for(std::chrono::milliseconds(serial_buffer_write_delay_));
  }
}

void NinjaV3CarDriver::ComportReadThread(){
  ComRecDataPack rec_data_pack;
  int rec_data_pack_size = sizeof(ComRecDataPack);
  hal::CarStateMsg pbStateMsg_;
  while(car_state_callback == nullptr);
  std::cout << "RegisterCarStateDataCallback() has been registered." <<std::endl;
  while(com_connected) {
    // read data from buffer
    int bytes_received = 0;
    uint8_t buff[rec_data_pack_size];
    for(int ii=0; ii<rec_data_pack_size;ii++) {
      buff[ii] = 0;
    }
    bytes_received = comport_driver_.ReadSensorPacket((uint8_t*)(&buff),rec_data_pack_size);
//    std::cout << "data is: "  << std::endl;
//    for(int ii=0;ii<bytes_received;ii++)
//      std::cout << " , " << buff[ii] ;
//    std::cout << std::endl;
    if(bytes_received == -1) {
      throw DeviceException("Error: NinjaV3 Disconnected.");
    }
    // check header if didn't match do the alignment
    if( !((buff[0]==DELIMITER0) && (buff[1]==DELIMITER1) && (buff[2]==DELIMITER2) && (buff[3]==DELIMITER3))) {
      std::cerr << "Received packet size is " << bytes_received << " expected #bytes was " << rec_data_pack_size << std::endl;
      // align
      int cnt;
      for(cnt=0;cnt<rec_data_pack_size;cnt++) {
        if((buff[cnt]==DELIMITER0 && buff[(cnt+1)%rec_data_pack_size]==DELIMITER1 && buff[(cnt+2)%rec_data_pack_size]==DELIMITER2 && buff[(cnt+3)%rec_data_pack_size]==DELIMITER3) ) {
          break;
        }
      }
      std::cout << "cnt is -> " << cnt << std::endl;
      if(cnt==rec_data_pack_size-1) {
        std::cout << "Could not find the packet header." << std::endl;
      }
      if(cnt != 0) {
        std::cout << "Loosing NinjaV3 COM packet." << std::endl;
        // read extra bytes in the begining to get ride of them
        bytes_received = comport_driver_.ReadSensorPacket((uint8_t*)(&buff),cnt);
      }
    } else {
      // check received checksum
      unsigned int sum = 0;
      for( int ii=0 ; ii<rec_data_pack_size-4 ; ii++ ){
        sum += buff[ii];
      }
      memcpy(&rec_data_pack,&buff,rec_data_pack_size);
      if( rec_data_pack.chksum == sum ){
        pbStateMsg_.set_motor_current((double)rec_data_pack.motor_current);
        pbStateMsg_.set_steer_angle((double)rec_data_pack.steer_ang);
        pbStateMsg_.set_swing_angle_fl((double)rec_data_pack.swing_ang0);
        pbStateMsg_.set_swing_angle_rl((double)rec_data_pack.swing_ang1);
        pbStateMsg_.set_swing_angle_rr((double)rec_data_pack.swing_ang2);
        pbStateMsg_.set_swing_angle_fr((double)rec_data_pack.swing_ang3);
        pbStateMsg_.set_wheel_speed_fl((double)rec_data_pack.enc0);
        pbStateMsg_.set_wheel_speed_rl((double)rec_data_pack.enc1);
        pbStateMsg_.set_wheel_speed_rr((double)rec_data_pack.enc2);
        pbStateMsg_.set_wheel_speed_fr((double)rec_data_pack.enc3);
        pbStateMsg_.set_device_time(rec_data_pack.dev_time);
        car_state_callback(pbStateMsg_);
      }else{
        std::cerr << "Error:  checksum didn't match" << std::endl;
      }
    }
  }
}

void NinjaV3CarDriver::UpdateCarCommand(CarCommandMsg &command_msg){
  std::lock_guard<std::mutex> mutex_lock(tr_mutex);
  pbCommandMsg_ = command_msg;
}
}
