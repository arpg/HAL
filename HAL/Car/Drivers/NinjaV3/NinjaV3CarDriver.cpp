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
    tr_data_pack.motor_power_percent = (float)pbCommandMsg_.throttle_percent();
    tr_data_pack.steering_angle = (float)pbCommandMsg_.steering_angle();
    // TODO: dev_time should be set to correct system time
    tr_data_pack.dev_time = 0;
    // calculate checksum from whole packet
    unsigned int chksum = 0;
    unsigned char* data = (unsigned char*)(&tr_data_pack);
    for(int ii=0;ii<sizeof(ComTrDataPack);ii++){
      chksum += data[ii];
    }
    // send packet
    comport_driver_.WriteComPort((unsigned char*)(&tr_data_pack),sizeof(ComTrDataPack));
    std::this_thread::sleep_for(std::chrono::milliseconds(serial_buffer_write_delay_));
  }
}

void NinjaV3CarDriver::ComportReadThread(){
  ComRecDataPack rec_data_pack;
  hal::CarStateMsg pbStateMsg_;
  while(com_connected) {
    if(car_state_callback != nullptr) {
      // read data from buffer
      int bytes_received = 0;
      bytes_received = comport_driver_.ReadSensorPacket((unsigned char*)(&rec_data_pack),sizeof(ComRecDataPack));
      if(bytes_received != sizeof(ComRecDataPack)) {
        std::cout << "Received packet size is " << bytes_received << " expected #bytes was" << sizeof(ComRecDataPack) << std::endl;
        // align
//        int ii = 0;
//        while( bytes[ii] != FTDI_PACKET_DELIMITER1 && bytes[ii+1] != FTDI_PACKET_DELIMITER2 ) {
//            ii++;
//        }
//  //      printf("  ii is: %d  ",ii);
//        if( ii != 0 ) {
//  //          std::cout << "LOSING PACKET!" << std::endl;
//            bytesRead = read(m_PortHandle, bytes, ii);
//  //          for(int jj=0;jj<bytesRead;jj++)
//  //              printf(" %d,",bytes[jj]);
//  //          printf("\n********************\n");
//            return 0;
//        }

      } else {
        // check received checksum
        unsigned int sum = 0;
        unsigned char* data = (unsigned char*)(&rec_data_pack);
        for( int ii=0 ; ii<sizeof(ComRecDataPack) ; ii++ ){
          sum += data[ii];
        }
        if( rec_data_pack.chksum == sum ){
          if(rec_data_pack.pack_type == 2) {
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
          } else {
            std::cout << "Error: wrong packet type received" << std::endl;
          }
        }else{
          std::cout << "Error: packet checksum didn't match" << std::endl;
        }
      }
    } else {
      std::cout << "Warning: RegisterCarStateDataCallback() has not been registered." <<std::endl;
    }
  }
}

void NinjaV3CarDriver::SendCarCommand(CarCommandMsg &command_msg){
  pbCommandMsg_ = command_msg;
}
}
