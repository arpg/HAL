#pragma once

#include <iostream>
#include <stdint.h>
#include <cmath>
#include <stdio.h> // standard input / output functions
#include <string.h> // string function definitions
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <termios.h> // POSIX terminal control definitionss
#include <time.h>   // time calls
#include <sys/time.h>
#include <time.h>

// ??? how should I include these header files?
#include "PCANEncoderDriver.h"
#include <HAL/IMU/Drivers/PCAN/PCANIMUDriver.h>

#include <libpcan.h>
//#include <pcan.h>
#define B125K 125000
#define B500K 500000
#define B1M   1000000

#define Lex_YawRate_id         0x24
#define Lex_YawRate_mul     1//0.244
#define Lex_YawRate_offset  0//-125

#define Lex_XAcc_id            0x24
#define Lex_XAcc_mul        1//0.03589
#define Lex_XAcc_offset     0//-18.375

#define Lex_YAcc_id            0x24
#define Lex_YAcc_mul        1//0.03589
#define Lex_YAcc_offset     0//-18.375
// 548 brake id
// 800 brake
// 705,720 gear
// 835 timer
// 176 front
// ,178

#define Lex_FrontWheelOdom_id          176  //0xB0
#define Lex_FrontWheelOdom_mul         0.01 // Km/h
#define Lex_FrontWheelOdom_offset      0

#define Lex_RearWheelOdom_id          178   //0xB2
#define Lex_RearWheelOdom_mul         0.01  // Km/h
#define Lex_RearWheelOdom_offset      0

#define Lex_GearState_id            0x3BC

#define Lex_SteeringWheel_id        0x25
#define Lex_SteeringWheel_mul       1.5
#define Lex_SteeringWheel_offset    -140
// no zero data
#define Lex_MemSteeringZero_id        0x25
#define Lex_MemSteeringZero_mul       1.5
#define Lex_MemSteeringZero_offset    0x800

#define Lex_Pinion_id                 0x260
#define Lex_Pinion_mul                1//0.001
#define Lex_Pinion_offset             0

#include <HAL/IMU/IMUDriverInterface.h>
#include <HAL/Utils/TicToc.h>
#include <HAL/Encoder/EncoderDriverInterface.h>

struct CANMessage
{
    unsigned int id;       // 11/29 bit code
    char  msgtype;         // bits of MSGTYPE_*
    char  len;             // count of data bytes (0..8)
    char  data[8];         // data bytes, up to 8
    // TODO: findout how dgc_get_time() works then define timestamp type
};

struct CANParsedPkg
{
    double YawRate;
    double EncRate_FL;
    double EncRate_FR;
    double EncRate_RL;
    double EncRate_RR;
    double GearState;
    double Acc_x;
    double Acc_y;
    double SteeringAngle;
    double MemSteeringZero;
    double Pinion;

};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class PCANListener
{

public:
  ///////////////////////////////////////////////////////////////////////////////
  static PCANListener& GetInstance()
  {
    static PCANListener s_Instance;
    return s_Instance;
  }

  ///////////////////////////////////////////////////////////////////////////////
  PCANListener() : m_bIsConnected(false), m_IMUCallback(nullptr), m_EncoderCallback(nullptr)
  {
  }

  ///////////////////////////////////////////////////////////////////////////////
  ~PCANListener()
  {
    Disconnect();
  }

  ///////////////////////////////////////////////////////////////////////////////
  void RegisterIMUCallback(hal::IMUDriverDataCallback callback)
  {
    m_IMUCallback = callback;
    if( m_Running == false ) {
      m_Running = true;
      m_CallbackThread = std::thread( &PCANListener::_ThreadFunc, this );
    }
  }

  ///////////////////////////////////////////////////////////////////////////////
  void RegisterEncoderCallback(hal::EncoderDriverDataCallback callback)
  {
    m_EncoderCallback = callback;
    if( m_Running == false ) {
      m_Running = true;
      m_CallbackThread = std::thread( &PCANListener::_ThreadFunc, this );
    }
  }

  ///////////////////////////////////////////////////////////////////////////////
  bool Connect(const char *path)
  {
    // Default path is "/dev/pcan32"
    if( m_bIsConnected == false ) {
     //open the given path
//      m_bIsConnected = _OpenCANBus(path, B500K);
      m_bIsConnected = _OpenCANBus("/dev/pcan32", B500K);
    }
    return m_bIsConnected;
  }

  ///////////////////////////////////////////////////////////////////////////////
  void Disconnect()
  {
    m_Running = false;
    if( m_CallbackThread.joinable() ) {
      m_CallbackThread.join();
    }
    if(m_bIsConnected) {
      _CloseCANBus(m_PortHandle);
    }
  }

private:

  ///////////////////////////////////////////////////////////////////////////////
  void _CloseCANBus(HANDLE pcanHandle)
  {
    std::cout << "End PCAN\n" << std::endl;
    if(pcanHandle) {
      CAN_Close(pcanHandle);
      pcanHandle = NULL;
    }
    m_bIsConnected = false;
  }

  ///////////////////////////////////////////////////////////////////////////////
  int _ReadCANBus(CANMessage* pkg)
  {
    static TPCANRdMsg   Rdmsg;
    static const int    timeout = 10000; /* 0.01 seconds. this is a default value */
//    pthread_mutex_lock(&pcan_mutex);
      int read_error = LINUX_CAN_Read_Timeout(m_PortHandle, &Rdmsg, timeout);
//    pthread_mutex_unlock(&pcan_mutex);
//    message->timeStamp = dgc_get_time();
    TPCANMsg msg = Rdmsg.Msg;
    if(!read_error) {
      // TODO? we read "len", but then only copy 8 bytes?
      // might cause problems for msgtype extended
      pkg->id = msg.ID;
      pkg->msgtype = msg.MSGTYPE;
      pkg->len = msg.LEN;
      for(int i=0;i<8;++i) {
        pkg->data[i] = msg.DATA[i];
      }
    }else
    {
      std::cout << "CAN Bus read Error!\n" << std::endl;
      PrintParsedError(read_error);
    }
    return read_error;
  }

  ///////////////////////////////////////////////////////////////////////////////
  int _WriteCANBus(CANMessage* pkg)
  {
    TPCANMsg Wmsg;
    Wmsg.MSGTYPE = pkg->msgtype;
    Wmsg.ID = pkg->id;
    Wmsg.LEN = pkg->len;
    for(int i=0;i<8;++i) {
      Wmsg.DATA[i] = pkg->data[i];
    }
    int error = LINUX_CAN_Write_Timeout(m_PortHandle, &Wmsg, 0);

    if(error)
    {
      std::cout << "CAN Bus Write Error!\n" << std::endl;
      return 0;
    }else
      return error;
  }
///////////////////////////////////////////////////////////////////////////////
  void PrintParsedError(int err_int)
  {
      if(err_int){
          std::cout << "Error : " << std::endl;
          switch(err_int)
          {
              case 0x0001:  std::cout << "transmit buffer full\n" << std::endl; break;
              case 0x0002:  std::cout << "overrun in receive buffer\n" << std::endl; break;
              case 0x0004:  std::cout << "bus error, errorcounter limit reached\n" << std::endl; break;
              case 0x0008:  std::cout << "bus error, errorcounter limit reached\n" << std::endl; break;
              case 0x0010:  std::cout << "bus error, 'bus off' state entered\n" << std::endl; break;
              case 0x0020:  std::cout << "receive queue is empty\n" << std::endl; break;
              case 0x0040:  std::cout << "receive queue overrun\n" << std::endl; break;
              case 0x0080:  std::cout << "transmit queue full \n" << std::endl; break;
              case 0x0100:  std::cout << "test of controller registers failed\n" << std::endl; break;
              case 0x0200:  std::cout << "Win95/98/ME only\n" << std::endl; break;
              case 0x2000:  std::cout << "can't create resource\n" << std::endl; break;
              case 0x4000:  std::cout << "illegal parameter\n" << std::endl; break;
              case 0x8000:  std::cout << "value out of range\n" << std::endl; break;
              case 0x1C00:  std::cout << "wrong handle, handle error\n" << std::endl; break;
          }
      }
  }
  ///////////////////////////////////////////////////////////////////////////////
/////// CAN BUS error codes
///#define CAN_ERR_OK             0x0000  // no error
///#define CAN_ERR_XMTFULL        0x0001  // transmit buffer full
///#define CAN_ERR_OVERRUN        0x0002  // overrun in receive buffer
///#define CAN_ERR_BUSLIGHT       0x0004  // bus error, errorcounter limit reached
///#define CAN_ERR_BUSHEAVY       0x0008  // bus error, errorcounter limit reached
///#define CAN_ERR_BUSOFF         0x0010  // bus error, 'bus off' state entered
///#define CAN_ERR_QRCVEMPTY      0x0020  // receive queue is empty
///#define CAN_ERR_QOVERRUN       0x0040  // receive queue overrun
///#define CAN_ERR_QXMTFULL       0x0080  // transmit queue full
///#define CAN_ERR_REGTEST        0x0100  // test of controller registers failed
///#define CAN_ERR_NOVXD          0x0200  // Win95/98/ME only
///#define CAN_ERR_RESOURCE       0x2000  // can't create resource
///#define CAN_ERR_ILLPARAMTYPE   0x4000  // illegal parameter
///#define CAN_ERR_ILLPARAMVAL    0x8000  // value out of range
///#define CAN_ERRMASK_ILLHANDLE  0x1C00  // wrong handle, handle error

  bool _OpenCANBus(const char* comPortPath,const int baudRate = B500K)
  {
    int init_err, status_err;
    bool connection_flag = false;
    m_PortHandle = LINUX_CAN_Open(comPortPath, O_RDWR); // O_RDWR -> open the port in Blocking mode
    if(m_PortHandle) {
      switch(baudRate) {
      case B125K: {
          init_err = CAN_Init(m_PortHandle, CAN_BAUD_125K, CAN_INIT_TYPE_ST);
          break;
      }
      case B500K: {
        init_err = CAN_Init(m_PortHandle, CAN_BAUD_500K, CAN_INIT_TYPE_ST);
        break;
      }
      case B1M: {
        init_err = CAN_Init(m_PortHandle, CAN_BAUD_1M, CAN_INIT_TYPE_ST);
        break;
      }
      default:
        init_err = CAN_Init(m_PortHandle, CAN_BAUD_1M, CAN_INIT_TYPE_ST);
      }

      status_err = CAN_Status(m_PortHandle);
      if(init_err != 0) // || (status_err != 0))
        std::cout << "CAN Status Error : CAN bus initialization failed.\n" << std::endl;
      else{
        std::cout << "CAN bus initialized successfully.\n" << std::endl;
        connection_flag = true;
      }

  }else
        std::cout << "Can not open PCAN device, (Please chech device name).\n" << std::endl;
  return connection_flag;
}

private:
  CANParsedPkg Init_CANParsedPkg(CANParsedPkg pkg)
  {
    pkg.Acc_x = 0;
    pkg.Acc_y = 0;
    pkg.EncRate_FL = 0;
    pkg.EncRate_FR = 0;
    pkg.EncRate_RL = 0;
    pkg.EncRate_RR = 0;
    pkg.GearState = 0;
    pkg.SteeringAngle = 0;
    pkg.YawRate = 0;
    pkg.MemSteeringZero = 0;
    pkg.Pinion = 0;

    return pkg;
  }
  int read_int_from_12_signed_bits(char data1, char data2)
  {
      const uint8_t upper_mask = 0x0F; // = 0000 1111
      data1 = data1 & upper_mask;

      const uint8_t sign_test = 0x08; // = 0000 1000

      int result;
      if (data1 & sign_test)
      {
          data1 = ~data1 & upper_mask;
          data2 = ~data2;
          result = ((data2&0xFF) + (data1&0xFF)*256 + 1) * -1;
      } else {
          result = (data2&0xFF) + (data1&0xFF)*256;
      }
      return result;
  }

  int read_int_from_9_signed_bits(char data1, char data2)
  {
      const uint8_t upper_mask = 0x01;
      data1 = data1 & upper_mask;

      const uint8_t sign_test = 0x01;

      int result;
      if (data1 & sign_test)
      {
          data1 = ~data1 & upper_mask;
          data2 = ~data2;
          result = ((data2&0xFF) + (data1&0xFF)*256 + 1) * -1;
      } else {
          result = (data2&0xFF) + (data1&0xFF)*256;
      }
      return result;
  }
  CANParsedPkg ParseLexusCanMessage(CANMessage* RawMsg)
  {
    static CANParsedPkg ParsedMsg;
    Init_CANParsedPkg(ParsedMsg);
    if( RawMsg->id == Lex_YawRate_id)
        ParsedMsg.YawRate = (short)(read_int_from_9_signed_bits(RawMsg->data[0],RawMsg->data[1]))*Lex_YawRate_mul+Lex_YawRate_offset;
    if( RawMsg->id == Lex_XAcc_id)
      ParsedMsg.Acc_x = (short)(read_int_from_9_signed_bits(RawMsg->data[2],RawMsg->data[3]))*Lex_XAcc_mul+Lex_XAcc_offset;
    if( RawMsg->id == Lex_YAcc_id)
      ParsedMsg.Acc_y = (short)(read_int_from_9_signed_bits(RawMsg->data[4],RawMsg->data[5]))*Lex_YAcc_mul+Lex_YAcc_offset;

    if( RawMsg->id == Lex_FrontWheelOdom_id)
      ParsedMsg.EncRate_FR = (short)(((RawMsg->data[0] & 0x00FF)<<8)|(RawMsg->data[1] & 0x00FF))*Lex_FrontWheelOdom_mul+Lex_FrontWheelOdom_offset;
    if( RawMsg->id == Lex_FrontWheelOdom_id)
      ParsedMsg.EncRate_FL = (short)(((RawMsg->data[2] & 0x00FF)<<8)|(RawMsg->data[3] & 0x00FF))*Lex_FrontWheelOdom_mul+Lex_FrontWheelOdom_offset;
    if( RawMsg->id == Lex_RearWheelOdom_id)
      ParsedMsg.EncRate_RR = (short)(((RawMsg->data[0] & 0x003F)<<8)|(RawMsg->data[1] & 0x00FF))*Lex_RearWheelOdom_mul+Lex_RearWheelOdom_offset;
    if( RawMsg->id == Lex_RearWheelOdom_id)
      ParsedMsg.EncRate_RL = (short)(((RawMsg->data[2] & 0x003F)<<8)|(RawMsg->data[3] & 0x00FF))*Lex_RearWheelOdom_mul+Lex_RearWheelOdom_offset;
#if(true)
//    if( RawMsg->id == Lex_SteeringWheel_id)
    {
        std::cout << "\tD0:" << (RawMsg->data[0]&0xFF);
        std::cout << "\tD1:" << (RawMsg->data[1]&0xFF);
        std::cout << "\tD2:" << (RawMsg->data[2]&0xFF);
        std::cout << "\tD3:" << (RawMsg->data[3]&0xFF);
        std::cout << "\tD4:" << (RawMsg->data[4]&0xFF);
        std::cout << "\tD5:" << (RawMsg->data[5]&0xFF);
        std::cout << "\tD6:" << (RawMsg->data[6]&0xFF);
        std::cout << std::endl;
    }
#endif
    if( RawMsg->id == Lex_SteeringWheel_id)
      ParsedMsg.SteeringAngle = (double)(read_int_from_12_signed_bits(RawMsg->data[0],RawMsg->data[1]))*Lex_SteeringWheel_mul+Lex_SteeringWheel_offset;
    if( RawMsg->id == Lex_MemSteeringZero_id)
      ParsedMsg.MemSteeringZero = (double)(read_int_from_12_signed_bits(RawMsg->data[2],RawMsg->data[3]))*Lex_MemSteeringZero_mul+Lex_MemSteeringZero_offset;

    if( RawMsg->id == Lex_Pinion_id)
      ParsedMsg.Pinion = (short)((((RawMsg->data[3] & 0x00FF)<<8)|(RawMsg->data[4] & 0x00FF))*Lex_Pinion_mul+Lex_Pinion_offset)*180/M_PI;

    return ParsedMsg;
  }

  /////////////////////////////////////////////////////////////////////////////////////////
  void _ThreadFunc()
  {
    pb::ImuMsg      pbIMU;
    pb::EncoderMsg  pbEncoder;
    CANMessage      RawPkg;
    CANParsedPkg    Pkt;
    while( m_Running ) {

      if( _ReadCANBus(&RawPkg) ) {
        std::cout << "Error reading CAN Bus port.\n" << std::endl;
        continue;
      }

      if ((RawPkg.id == Lex_Pinion_id))
      {
          Pkt = ParseLexusCanMessage(&RawPkg);
          std::cout << "\t:" << std::hex << RawPkg.id;

#if(false)
          std::cout << "\t Acc:" << Pkt.Acc_x << ",\t" << Pkt.Acc_y ;
          std::cout << "\tYaw:" << Pkt.YawRate;
          std::cout << "\tEncFL:" << Pkt.EncRate_FL;
          std::cout << "\tEncFR:" << Pkt.EncRate_FR;
          std::cout << "\tEncRL:" << Pkt.EncRate_RL;
          std::cout << "\tEncRR:" << Pkt.EncRate_RR;

          std::cout << "\tPin:" << Pkt.Pinion;
//          std::cout << "\tStrZ:" << Pkt.MemSteeringZero;
//          std::cout << "\tStrA: " << Pkt.SteeringAngle;
          std::cout << std::endl;
#endif
}
      if( m_IMUCallback ) {
        pbIMU.Clear();

        pbIMU.set_id(2); // changed the Id from 1 to 2
        pbIMU.set_device_time( hal::Tic() );

        pb::VectorMsg* pbVec = pbIMU.mutable_accel();
        pbVec->add_data(Pkt.YawRate);
        pbVec->add_data(Pkt.YawRate);
        pbVec->add_data(Pkt.YawRate);

        pbVec = pbIMU.mutable_gyro();
        pbVec->add_data(0); //Gyro X
        pbVec->add_data(0); //Gyro Y
        pbVec->add_data(Pkt.YawRate); //Gyro Z

        (m_IMUCallback)(pbIMU);
      }
/*
      if( m_EncoderCallback ) {
        pbEncoder.Clear();
        pbEncoder.set_device_time( hal::Tic() );

//        // encoders
//        pbEncoder.set_label(0, "ENC_RATE_FL");
//        pbEncoder.set_data(0, Pkt.EncRate_FL);
//        pbEncoder.set_label(1, "ENC_RATE_FR");
//        pbEncoder.set_data(1, Pkt.EncRate_FR);
//        pbEncoder.set_label(2, "ENC_RATE_RL");
//        pbEncoder.set_data(2, Pkt.EncRate_RL);
//        pbEncoder.set_label(3, "ENC_RATE_RR");
//        pbEncoder.set_data(3, Pkt.EncRate_RR);

        // encoders
        pbEncoder.set_label(0, "ENC_LF");
        pbEncoder.set_data(0, Pkt.EncRate_FL);
        pbEncoder.set_label(1, "ENC_RF");
        pbEncoder.set_data(1, Pkt.EncRate_FR);
        pbEncoder.set_label(2, "ENC_LB");
        pbEncoder.set_data(2, Pkt.EncRate_RL);
        pbEncoder.set_label(3, "ENC_RB");
        pbEncoder.set_data(3, Pkt.EncRate_RR);
        (m_EncoderCallback)(pbEncoder);
      }
*/    }
  }

private:

private:
  bool                            m_bIsConnected;
  HANDLE                          m_PortHandle;

  bool                            m_Running;
  std::thread                     m_CallbackThread;
  hal::IMUDriverDataCallback           m_IMUCallback;
  hal::EncoderDriverDataCallback       m_EncoderCallback;
};
