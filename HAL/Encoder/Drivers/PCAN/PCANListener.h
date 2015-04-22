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

#include <libpcan.h>

#include <HAL/IMU/Drivers/PCAN/PCANIMUDriver.h>

#include "PCANEncoderDriver.h"

struct CANParsedPkg
{
    enum CANParsedPkgType{tEncoder_F,tEncoder_R,tIMU,tSteeringAngle,tPinion,tNULL};
    CANParsedPkgType PkgType;
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
    double PinionAngle;
};

struct CANMessage
{
    unsigned int id;       // 11/29 bit code
    char  msgtype;         // bits of MSGTYPE_*
    char  len;             // count of data bytes (0..8)
    char  data[8];         // data bytes, up to 8
    // TODO: findout how dgc_get_time() works then define timestamp type
};


#include <HAL/ThirdParty/ThirdPartyConfig.h>

#ifdef HAL_HAVE_LEXUSISF12
#include <HAL/ThirdParty/ThirdParty/PCAN/LexusISF2012.h>
#endif


#define B125K 125000
#define B500K 500000
#define B1M   1000000

#include <HAL/IMU/IMUDriverInterface.h>
#include <HAL/Utils/TicToc.h>
#include <HAL/Encoder/EncoderDriverInterface.h>


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
  bool Connect(unsigned long int baudrate, std::string path)
  {
    // Default path is "/dev/pcan32"
    if( m_bIsConnected == false ) {
     //open the given path
      m_bIsConnected = _OpenCANBus(path.c_str(), baudrate);
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

  bool IsRunning() const {
    return m_Running;
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
    int init_err=0, status_err=0;
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
      std::cout << "CAN Status : " << status_err << "\n" << std::endl;
      if(init_err != 0)
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

  /////////////////////////////////////////////////////////////////////////////////////////
  void _ThreadFunc()
  {
    hal::ImuMsg      pbIMU;
    hal::EncoderMsg  pbEncoderRear;
    hal::EncoderMsg  pbEncoderFront;
    hal::EncoderMsg  pbSteering;
    CANMessage      RawPkg;
    CANParsedPkg    Pkg;
#ifdef HAL_HAVE_LEXUSISF12
    LexusISF12      Car;
#endif

    while( m_Running ) {

      if( _ReadCANBus(&RawPkg) ) {
        std::cout << "Error reading CAN Bus port.\n" << std::endl;
        continue;
      }
      Pkg.PkgType = CANParsedPkg::tNULL;
#ifdef HAL_HAVE_LEXUSISF12
      Pkg = Car.ParseLexusCanMessage(&RawPkg);
//      std::cout << "\t:" << std::hex << RawPkg.id;
      #if(false)
          PrintLexusMsgs(Pkg);
      #endif
#endif
    if( m_IMUCallback && (Pkg.PkgType==CANParsedPkg::tIMU) ) {
        pbIMU.Clear();

        pbIMU.set_id(2); // changed the Id from 1 to 2
        pbIMU.set_device_time( hal::Tic() );

        hal::VectorMsg* pbVec = pbIMU.mutable_accel();
        pbVec->add_data(Pkg.Acc_x);
        pbVec->add_data(Pkg.Acc_y);

        pbVec = pbIMU.mutable_gyro();
        pbVec->add_data(Pkg.YawRate); //Gyro Z

        (m_IMUCallback)(pbIMU);
    }

    if( m_EncoderCallback && (Pkg.PkgType==CANParsedPkg::tEncoder_F) ) {
        pbEncoderFront.Clear();
        pbEncoderFront.set_device_time( hal::Tic() );

        // encoders
        pbEncoderFront.add_label("ENC_RATE_FL");
        pbEncoderFront.add_data(Pkg.EncRate_FL);
        pbEncoderFront.add_label("ENC_RATE_FR");
        pbEncoderFront.add_data(Pkg.EncRate_FR);
        (m_EncoderCallback)(pbEncoderFront);
    }

    if( m_EncoderCallback && (Pkg.PkgType==CANParsedPkg::tEncoder_R) ) {
        pbEncoderRear.Clear();
        pbEncoderRear.set_device_time( hal::Tic() );

        pbEncoderRear.add_label("ENC_RATE_RL");
        pbEncoderRear.add_data(Pkg.EncRate_RL);
        pbEncoderRear.add_label("ENC_RATE_RR");
        pbEncoderRear.add_data(Pkg.EncRate_RR);
        (m_EncoderCallback)(pbEncoderRear);
    }

//    pbEncoder.label_size()
//    pbEncoder.label().data(ii)

    if( m_EncoderCallback && (Pkg.PkgType==CANParsedPkg::tSteeringAngle) ) {
        pbSteering.Clear();
        pbSteering.set_device_time( hal::Tic() );

        pbSteering.add_label("RAW_STEERING_DATA");
        pbSteering.add_data(Pkg.SteeringAngle);
        (m_EncoderCallback)(pbSteering);
    }
   }
}

private:
  bool                            m_bIsConnected;
  HANDLE                          m_PortHandle;
  bool                            m_Running;
  std::thread                     m_CallbackThread;
  hal::IMUDriverDataCallback           m_IMUCallback;
  hal::EncoderDriverDataCallback       m_EncoderCallback;
};
