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
#include <vector>
#include <pthread.h>
#include <pcan_device.h>
#include <libpcan.h>
#include <pcan.h>
#define B125K 125000
#define B500K 500000
#define B1M   1000000

#include <HAL/IMU/IMUDriverInterface.h>
#include <HAL/Utils/TicToc.h>
#include <HAL/Encoder/EncoderDriverInterface.h>

#ifndef PrintMessage
#  ifdef ANDROID
#    include <android/log.h>
#    define PrintMessage( ... ) \
  (void)__android_log_print(ANDROID_LOG_INFO,  "HAL", __VA_ARGS__);
#  else
#    define PrintMessage( ... ) \
  printf( __VA_ARGS__ );
#  endif
#endif

using fPtr_IMU = void(*)(pb::ImuMsg& IMUdata);
using fPtr_Encoder = void(*)(pb::EncoderMsg& Encoderdata);

typedef int PCANHandle;

struct CanMessage
{
    unsigned int id;       // 11/29 bit code
    char  msgtype;         // bits of MSGTYPE_*
    char  len;             // count of data bytes (0..8)
    char  data[8];         // data bytes, up to 8
    // TODO: findout how dgc_get_time() works then define timestamp type
    unsigned long int timeStamp;
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

      if(m_PortHandle <= 0){
        PrintMessage("HAL: Failed to open at 500Kbps, aborting...\n");
      } else {
        PrintMessage("HAL: CAN BUS '%s'' opened.\n",path);
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
  void _CloseCANBus(PCANHandle pcanHandle)
  {
    PrintMessage("End PCAN\n");
    if(pcanHandle) {
      CAN_Close(pcanHandle);
      pcanHandle = NULL;
    }
    m_bIsConnected = false;
  }

  ///////////////////////////////////////////////////////////////////////////////
  int _ReadCANBus(CanMessage* pkg)
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
      PrintMessage("CAN Bus read Error!\n");
      PrintParsedError(read_error);
    }
    return read_error;
  }

  ///////////////////////////////////////////////////////////////////////////////
  int _WriteCANBus(CanMessage* pkg)
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
      PrintMessage("CAN Bus Write Error!\n");
      return 0;
    }else
      return error;
  }
///////////////////////////////////////////////////////////////////////////////
  void PrintParsedError(int err_int)
  {
      if(err_int){
          printf("Error : ");
          switch(err_int)
          {
              case 0x0001:  PrintMessage("transmit buffer full\n"); break;
              case 0x0002:  PrintMessage("overrun in receive buffer\n"); break;
              case 0x0004:  PrintMessage("bus error, errorcounter limit reached\n"); break;
              case 0x0008:  PrintMessage("bus error, errorcounter limit reached\n"); break;
              case 0x0010:  PrintMessage("bus error, 'bus off' state entered\n"); break;
              case 0x0020:  PrintMessage("receive queue is empty\n"); break;
              case 0x0040:  PrintMessage("receive queue overrun\n"); break;
              case 0x0080:  PrintMessage("transmit queue full \n"); break;
              case 0x0100:  PrintMessage("test of controller registers failed\n"); break;
              case 0x0200:  PrintMessage("Win95/98/ME only\n"); break;
              case 0x2000:  PrintMessage("can't create resource\n"); break;
              case 0x4000:  PrintMessage("illegal parameter\n"); break;
              case 0x8000:  PrintMessage("value out of range\n"); break;
              case 0x1C00:  PrintMessage("wrong handle, handle error\n"); break;
          }
      }
  }
  ///////////////////////////////////////////////////////////////////////////////
  PCANHandle _OpenCANBus(const char* comPortPath,const int baudRate = B500K)
  {
    int init_err, status_err;
    m_PortHandle = LINUX_CAN_Open(comPortPath, O_RDWR); // O_RDWR -> open the port in Blocking mode
    if(pcan_handle) {
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
      if((init_err != 0) || (status_err != 0)) {
        PrintMessage("PCAN initialization failed.\n");
        return -1;
      }
    } else {
        PrintMessage("Unable to open PCAN device.\n");
        return -1;
    }
    return pcan_handle;
  }


private:
  /////////////////////////////////////////////////////////////////////////////////////////
  void _ThreadFunc()
  {
    pb::ImuMsg      pbIMU;
    pb::EncoderMsg  pbEncoder;
    SensorPacket    Pkt;

    while( m_Running ) {

      if( ReadSensorPacket(Pkt) != sizeof(SensorPacket) ) {
        PrintMessage("Error reading virtual com port.\n");
        continue;
      }

      if( m_IMUCallback ) {
        pbIMU.Clear();

        pbIMU.set_id(1);
        pbIMU.set_device_time( hal::Tic() );

        pb::VectorMsg* pbVec = pbIMU.mutable_accel();
        pbVec->add_data(Pkt.Acc_x);
        pbVec->add_data(Pkt.Acc_y);
        pbVec->add_data(Pkt.Acc_z);

        pbVec = pbIMU.mutable_gyro();
        pbVec->add_data(Pkt.Gyro_x);
        pbVec->add_data(Pkt.Gyro_y);
        pbVec->add_data(Pkt.Gyro_z);

        pbVec = pbIMU.mutable_mag();
        pbVec->add_data(Pkt.Mag_x);
        pbVec->add_data(Pkt.Mag_y);
        pbVec->add_data(Pkt.Mag_z);

        (*m_IMUCallback)(pbIMU);
      }

      if( m_EncoderCallback ) {
        pbEncoder.Clear();

        pbEncoder.set_device_time( hal::Tic() );

        // encoders
        pbEncoder.set_label(0, "ENC_LF");
        pbEncoder.set_data(0, Pkt.Enc_LF);
        pbEncoder.set_label(1, "ENC_RF");
        pbEncoder.set_data(1, Pkt.Enc_RF);
        pbEncoder.set_label(2, "ENC_LB");
        pbEncoder.set_data(2, Pkt.Enc_LB);
        pbEncoder.set_label(3, "ENC_RB");
        pbEncoder.set_data(3, Pkt.Enc_RB);

        // adcs
        pbEncoder.set_label(4, "ADC_LB");
        pbEncoder.set_data(4, Pkt.ADC_LB);
        pbEncoder.set_label(5, "ADC_LF_YAW");
        pbEncoder.set_data(5, Pkt.ADC_LF_yaw);
        pbEncoder.set_label(6, "ADC_LF_ROLL");
        pbEncoder.set_data(6, Pkt.ADC_LF_rol);
        pbEncoder.set_label(7, "ADC_RB");
        pbEncoder.set_data(7, Pkt.ADC_RB);
        pbEncoder.set_label(8, "ADC_RF_YAW");
        pbEncoder.set_data(8, Pkt.ADC_RF_yaw);
        pbEncoder.set_label(9, "ADC_RF_ROLL");
        pbEncoder.set_data(9, Pkt.ADC_RF_rol);

        (*m_EncoderCallback)(pbEncoder);
      }
    }
  }


private:
  bool                            m_bIsConnected;
  PCANHandle                      m_PortHandle;

  bool                            m_Running;
  std::thread                     m_CallbackThread;
  fPtr_IMU                        m_IMUCallback;
  fPtr_Encoder                    m_EncoderCallback;
};
