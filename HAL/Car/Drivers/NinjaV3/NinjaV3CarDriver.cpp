#include "NinjaV3CarDriver.h"
#include <stdlib.h>
#include <unistd.h>
#include <HAL/Devices/DeviceException.h>
#include <HAL/Car.pb.h>
#include <stdint.h>
#include <HAL/Utils/TicToc.h>
namespace hal {

CarStateDataCallback NinjaV3CarDriver::car_state_callback = nullptr;

NinjaV3CarDriver::NinjaV3CarDriver(std::string dev, int baud) {
  baudrate_ = baud;
  dev_name_ = dev;
  should_run_ = true;
  // caculate the time to wait after writing to send buffer
  serial_buffer_write_delay_ = (int)((sizeof(ComTrDataPack)*10*1000)/baud)+1;
  if(Init() == false) {
    throw DeviceException("Could not connect to Comport device.");
  } else {
    std::cout << "Connection to NinjaV3Car successful." << std::endl;
  }
}

NinjaV3CarDriver::~NinjaV3CarDriver() {
  shouldrun_mutex.lock();
  should_run_ = false;
  shouldrun_mutex.unlock();

  if(comport_read_thread.joinable()) {
    comport_read_thread.join();
  }
  if(comport_write_thread.joinable()) {
    comport_write_thread.join();
  }

//  comport_mutex.lock();
  comport_driver2_.Disconnect();
  comport_driver_.Disconnect();
//  comport_mutex.unlock();

}

void NinjaV3CarDriver::RegisterCarStateDataCallback(CarStateDataCallback callback) {
  car_state_callback = callback;
}

bool NinjaV3CarDriver::Init(){
  bool conneciton_status = comport_driver_.Connect(dev_name_,baudrate_,sizeof(ComRecDataPack));
  bool conneciton_status2 = comport_driver2_.Connect(dev_name_,baudrate_,sizeof(ComRecDataPack));
  if( conneciton_status && conneciton_status2 ) {
    std::cout << "comport connected" << std::endl;
    comport_write_thread = std::thread(std::bind(&NinjaV3CarDriver::ComportWriteThread, this));
    comport_read_thread = std::thread(std::bind(&NinjaV3CarDriver::ComportReadThread, this));
  }
  return conneciton_status&conneciton_status2;
}

void NinjaV3CarDriver::ComportWriteThread(){
  ComTrDataPack tr_data_pack;
  while(should_run_) {
    shouldrun_mutex.unlock();
    cmdmsg_mutex.lock();
    tr_data_pack.motor_power_percent = (float)pbCommandMsg_.throttle_percent();
    tr_data_pack.steering_angle = (float)pbCommandMsg_.steering_angle();
    tr_data_pack.rear_steering_angle = (float)pbCommandMsg_.rear_steering_angle();
    tr_data_pack.dev_time = (float)pbCommandMsg_.device_time();
    cmdmsg_mutex.unlock();

    // TODO: dev_time should be set to correct system time
    // calculate checksum from whole packet
    unsigned int chksum = 0;
    uint8_t* data = (uint8_t*)(&tr_data_pack);
    for(int ii=0;ii<(int)sizeof(ComTrDataPack)-4;ii++){
      chksum += data[ii];
    }
    tr_data_pack.chksum = chksum;
//    uint8_t trbuff[sizeof(tr_data_pack)];
//    memcpy(&trbuff,&tr_data_pack,sizeof(tr_data_pack));
//    std::cout << "data is: "  << std::endl;
//    for(int ii=0;ii<sizeof(tr_data_pack);ii++)
//      std::cout << " , " << static_cast<int>(trbuff[ii]);
//    std::cout << std::endl;

    // send packet
//    std::cout << "data sent ." << std::endl;
    //comport_mutex.lock();
    comport_driver_.WriteComPort((uint8_t*)(&tr_data_pack),sizeof(ComTrDataPack));
    //comport_mutex.unlock();
    std::this_thread::sleep_for(std::chrono::milliseconds(serial_buffer_write_delay_));
    shouldrun_mutex.lock();
  }
  shouldrun_mutex.unlock();
//  std::cout << "exiting write" << std::endl;
}

void NinjaV3CarDriver::ComportReadThread(){
  ComRecDataPack rec_data_pack;
  int rec_data_pack_size = sizeof(ComRecDataPack);
  hal::CarStateMsg pbStateMsg_;
  while(car_state_callback == nullptr);
//  std::cout << "RegisterCarStateDataCallback() has been registered." <<std::endl;
  while(should_run_) {
    shouldrun_mutex.unlock();
    // read data from buffer
    int bytes_received = 0;
    uint8_t buff[rec_data_pack_size];
    for(int ii=0; ii<rec_data_pack_size;ii++) {
      buff[ii] = 0;
    }

   // comport_mutex.lock();
    bytes_received = comport_driver2_.ReadSensorPacket((uint8_t*)(&buff),rec_data_pack_size);
   // comport_mutex.unlock();
//    std::cout << "data is: "  << std::endl;
//    for(int ii=0;ii<bytes_received;ii++)
//      std::cout << " , " << static_cast<int>(buff[ii]);
//    std::cout << std::endl;
    if(bytes_received == -1) {
      throw DeviceException("Error: NinjaV3 Disconnected.");
    }
    // check header if didn't match do the alignment
    if( !((buff[0]==DELIMITER0) && (buff[1]==DELIMITER1) && (buff[2]==DELIMITER2) && (buff[3]==DELIMITER3))) {
      if(bytes_received != rec_data_pack_size) {
       // std::cerr << "Received packet size is " << bytes_received << " expected #bytes was " << rec_data_pack_size << std::endl;
      } else {
        std::cerr << "Packet headers didn't match." << std::endl;
      }
      // align
      int cnt;
      for(cnt=0;cnt<rec_data_pack_size;cnt++) {
        if((buff[cnt]==DELIMITER0 && buff[(cnt+1)%rec_data_pack_size]==DELIMITER1 && buff[(cnt+2)%rec_data_pack_size]==DELIMITER2 && buff[(cnt+3)%rec_data_pack_size]==DELIMITER3) ) {
          break;
        }
      }
      //std::cout << "Allignment shift is " << cnt << std::endl;
      if(cnt==rec_data_pack_size-1) {
        //std::cout << "Could not find the packet header." << std::endl;
      }
      if(cnt != 0) {
        //std::cout << "Loosing COM packet." << std::endl;
        // read extra bytes in the begining to get ride of them

     //   comport_mutex.lock();
        bytes_received = comport_driver2_.ReadSensorPacket((uint8_t*)(&buff),cnt);
      //  comport_mutex.unlock();
      }
    } else {

//      std::cout << "data is: "  << std::endl;
//      for(int ii=0;ii<bytes_received;ii++)
//        std::cout << " , " << static_cast<int>(buff[ii]);
//      std::cout << std::endl;
      // check received checksum
      unsigned int sum = 0;
      for( int ii=0 ; ii<rec_data_pack_size-4 ; ii++ ){
        sum += buff[ii];
      }
      memcpy(&rec_data_pack,&buff,rec_data_pack_size);
      if( rec_data_pack.chksum == sum ){
        pbStateMsg_.set_motor_current((rec_data_pack.motor_current));
        pbStateMsg_.set_batt_volt((double)rec_data_pack.batt_volt);
        pbStateMsg_.set_steer_angle((double)(rec_data_pack.steer_ang));
        pbStateMsg_.set_rear_steer_angle((double)(rec_data_pack.rear_steer_ang));
        pbStateMsg_.set_swing_angle_fl((double)(rec_data_pack.swing_ang0));
        pbStateMsg_.set_swing_angle_rl((double)(rec_data_pack.swing_ang1));
        pbStateMsg_.set_swing_angle_rr((double)(rec_data_pack.swing_ang2));
        pbStateMsg_.set_swing_angle_fr((double)(rec_data_pack.swing_ang3));
        pbStateMsg_.set_wheel_speed_fl((double)(rec_data_pack.enc0));
        pbStateMsg_.set_wheel_speed_rl((double)(rec_data_pack.enc1));
        pbStateMsg_.set_wheel_speed_rr((double)(rec_data_pack.enc2));
        pbStateMsg_.set_wheel_speed_fr((double)(rec_data_pack.enc3));
        pbStateMsg_.set_device_time((rec_data_pack.dev_time));
        car_state_callback(pbStateMsg_);
      }else{
//        std::cerr << "Error:  checksum didn't match. received: " << rec_data_pack.chksum << " calculated: " << sum << std::endl;
      }
    }
    shouldrun_mutex.lock();
  }
  shouldrun_mutex.unlock();
//  std::cout << "exiting read" << std::endl;
}

void NinjaV3CarDriver::UpdateCarCommand(CarCommandMsg &command_msg){
  cmdmsg_mutex.lock();
  pbCommandMsg_ = command_msg;
  cmdmsg_mutex.unlock();
}
}
