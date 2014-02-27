#pragma once

#include <iostream>
#include <stdint.h>
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
#include <libpcan.h>
//#include <pcan.h>
#define B125K 125000
#define B500K 500000
#define B1M   1000000

#define Lex_YawRate_id         0x24
#define Lex_YawRate_mul     0.244
#define Lex_YawRate_offset  -125

#define Lex_XAcc_id            0x24
#define Lex_XAcc_mul        0.03589
#define Lex_XAcc_offset     -18.375

#define Lex_YAcc_id            0x24
#define Lex_YAcc_mul        0.03589
#define Lex_YAcc_offset     -18.375

#define Lex_WheelOdom_id          0xAA
#define Lex_WheelOdom_mul         0.01
#define Lex_WheelOdom_offset      -67.67

#define Lex_GearState_id          0x3BC

#define Lex_SteeringWheel_id        0x25
#define Lex_SteeringWheel_mul       1
#define Lex_SteeringWheel_offset    0

#include <HAL/IMU/IMUDriverInterface.h>
#include <HAL/Utils/TicToc.h>
#include <HAL/Encoder/EncoderDriverInterface.h>

using fPtr_IMU = void(*)(pb::ImuMsg& IMUdata);
using fPtr_Encoder = void(*)(pb::EncoderMsg& Encoderdata);

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
  void RegisterIMUCallback(fPtr_IMU callback)
  {
    m_IMUCallback = callback;
    if( m_Running == false ) {
      m_Running = true;
      m_CallbackThread = std::thread( &PCANListener::_ThreadFunc, this );
    }
  }

  ///////////////////////////////////////////////////////////////////////////////
  void RegisterEncoderCallback(fPtr_Encoder callback)
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
      m_PortHandle = _OpenCANBus(path, B500K);

      if(!m_PortHandle){
        std::cout << "HAL: Failed to open at 500Kbps, aborting...\n" << std::endl;
      } else {
        std::cout << "HAL: CAN BUS '%s'' opened.\n" << std::endl;
        m_bIsConnected = true;
      }
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
  HANDLE _OpenCANBus(const char* comPortPath,const int baudRate = B500K)
  {
    int init_err, status_err;
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
      if((init_err != 0) || (status_err != 0))
        std::cout << "PCAN initialization failed.\n" << std::endl;
      else
        std::cout << "Unable to open PCAN device.\n" << std::endl;
  }
  return m_PortHandle;
}

private:

  CANParsedPkg ParseLexusCanMessage(CANMessage* RawMsg)
  {
    static CANParsedPkg ParsedMsg;
    if( RawMsg->id == Lex_YawRate_id)
      ParsedMsg.YawRate = (((RawMsg->data[1] & 0x0003)<<8)|(RawMsg->data[2]))*Lex_YawRate_mul+Lex_YawRate_offset;
    if( RawMsg->id == Lex_XAcc_id)
      ParsedMsg.Acc_x = (((RawMsg->data[3] & 0x0003)<<8)|(RawMsg->data[4]))*Lex_XAcc_mul+Lex_XAcc_offset;
    if( RawMsg->id == Lex_YAcc_id)
      ParsedMsg.Acc_y = (((RawMsg->data[5] & 0x0003)<<8)|(RawMsg->data[6]))*Lex_YAcc_mul+Lex_YAcc_offset;
//    if( RawMsg->id == Lex_SteeringWheel_id)
//      ParsedMsg.SteeringWheel = (((RawMsg->data[?] & 0x0003)<<8)|(RawMsg->data[?]))*Lex_SteeringWheel_mul+Lex_SteeringWheel_offset;
    if( RawMsg->id == Lex_WheelOdom_id)
      ParsedMsg.EncRate_FR = (double)(((RawMsg->data[1] & 0x007F)<<8)|(RawMsg->data[2]))*Lex_SteeringWheel_mul+Lex_SteeringWheel_offset;
    if( RawMsg->id == Lex_WheelOdom_id)
      ParsedMsg.EncRate_FL = (((RawMsg->data[3] & 0x007F)<<8)|(RawMsg->data[4]))*Lex_SteeringWheel_mul+Lex_SteeringWheel_offset;
    if( RawMsg->id == Lex_WheelOdom_id)
      ParsedMsg.EncRate_RR = (((RawMsg->data[5] & 0x007F)<<8)|(RawMsg->data[6]))*Lex_SteeringWheel_mul+Lex_SteeringWheel_offset;
    if( RawMsg->id == Lex_WheelOdom_id)
      ParsedMsg.EncRate_RL = (((RawMsg->data[7] & 0x007F)<<8)|(RawMsg->data[8]))*Lex_SteeringWheel_mul+Lex_SteeringWheel_offset;

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

      Pkt = ParseLexusCanMessage(&RawPkg);

      if( m_IMUCallback ) {
        pbIMU.Clear();

        pbIMU.set_id(2); // changed the Id from 1 to 2
        pbIMU.set_device_time( hal::Tic() );

        pb::VectorMsg* pbVec = pbIMU.mutable_accel();
        pbVec->add_data(Pkt.Acc_x);
        pbVec->add_data(Pkt.Acc_y);
        pbVec->add_data(0);

        pbVec = pbIMU.mutable_gyro();
        pbVec->add_data(0); //Gyro X
        pbVec->add_data(0); //Gyro Y
        pbVec->add_data(Pkt.YawRate); //Gyro Z

        (*m_IMUCallback)(pbIMU);
      }

      if( m_EncoderCallback ) {
        pbEncoder.Clear();
        pbEncoder.set_device_time( hal::Tic() );

        // encoders
        pbEncoder.set_label(0, "ENC_RATE_FL");
        pbEncoder.set_data(0, Pkt.EncRate_FL);
        pbEncoder.set_label(1, "ENC_RATE_FR");
        pbEncoder.set_data(1, Pkt.EncRate_FR);
        pbEncoder.set_label(2, "ENC_RATE_RL");
        pbEncoder.set_data(2, Pkt.EncRate_RL);
        pbEncoder.set_label(3, "ENC_RATE_RR");
        pbEncoder.set_data(3, Pkt.EncRate_RR);

        (*m_EncoderCallback)(pbEncoder);
      }

    }
  }

private:

private:
  bool                            m_bIsConnected;
  HANDLE                          m_PortHandle;

  bool                            m_Running;
  std::thread                     m_CallbackThread;
  fPtr_IMU                        m_IMUCallback;
  fPtr_Encoder                    m_EncoderCallback;
};
