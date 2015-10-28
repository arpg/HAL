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

using fPtr_IMU = void(*)(hal::ImuMsg& IMUdata);
using fPtr_Encoder = void(*)(hal::EncoderMsg& Encoderdata);

typedef int ComPortHandle;

#define FTDI_PACKET_DELIMITER1 0xD0
#define FTDI_PACKET_DELIMITER2 0xAA

#pragma pack(1)
struct CommandPacket
{
  char m_cDelimiter1;
  char m_cDelimiter2;
  char m_cSize;
  int m_nSteering;
  int m_nSpeed;
  unsigned short int CHECK_SUM;
};

#pragma pack(1)
struct SensorPacket
{
  char m_cDelimiter1;
  char m_cDelimiter2;
  char m_cSize;
  short int   Acc_x;
  short int   Acc_y;
  short int   Acc_z;
  short int   Gyro_x;
  short int   Gyro_y;
  short int   Gyro_z;
  short int   Mag_x;
  short int   Mag_y;
  short int   Mag_z;
  int   Enc_LB;
  int   Enc_LF;
  int   Enc_RB;
  int   Enc_RF;
  short int   ADC_LB;
  short int   ADC_LF_yaw;
  short int   ADC_LF_rol;
  short int   ADC_RB;
  short int   ADC_RF_yaw;
  short int   ADC_RF_rol;
  unsigned short int CHECK_SUM;
};


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class FtdiListener
{

public:
  ///////////////////////////////////////////////////////////////////////////////
  static FtdiListener& GetInstance()
  {
    static FtdiListener s_Instance;
    return s_Instance;
  }

  ///////////////////////////////////////////////////////////////////////////////
  FtdiListener() : m_bIsConnected(false), m_IMUCallback(nullptr), m_EncoderCallback(nullptr)
  {
  }

  ///////////////////////////////////////////////////////////////////////////////
  ~FtdiListener()
  {
    Disconnect();
  }

  ///////////////////////////////////////////////////////////////////////////////
  void RegisterIMUCallback(fPtr_IMU callback)
  {
    m_IMUCallback = callback;
    if( m_Running == false ) {
      m_Running = true;
      m_CallbackThread = std::thread( &FtdiListener::_ThreadFunc, this );
    }
  }


  ///////////////////////////////////////////////////////////////////////////////
  void RegisterEncoderCallback(fPtr_Encoder callback)
  {
    m_EncoderCallback = callback;
    if( m_Running == false ) {
      m_Running = true;
      m_CallbackThread = std::thread( &FtdiListener::_ThreadFunc, this );
    }
  }

  ///////////////////////////////////////////////////////////////////////////////
  bool Connect(const char *path)
  {
    if( m_bIsConnected == false ) {
      //open the given path
      m_PortHandle = _OpenComPort(path, B115200);

      if(m_PortHandle <= 0){
        PrintMessage("HAL: Failed to open at 115200bps, aborting...\n");
      } else {
        PrintMessage("HAL: Com port '%s'' opened.\n",path);
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
      _CloseComPort(m_PortHandle);
    }
  }

  bool IsRunning() const override {
    return m_Running;
  }

  ///////////////////////////////////////////////////////////////////////////////
  void SendCommandPacket(const int nSteering, const int nSpeed)
  {
    CommandPacket Pkt;
    Pkt.m_cDelimiter1 = FTDI_PACKET_DELIMITER1;
    Pkt.m_cDelimiter2 = FTDI_PACKET_DELIMITER2;
    Pkt.m_cSize = sizeof(CommandPacket);
    Pkt.m_nSpeed = nSpeed;
    Pkt.m_nSteering = nSteering;

    unsigned char* ptr = (unsigned char*)&Pkt.m_cDelimiter1;
    Pkt.CHECK_SUM = 0;
    for( size_t ii = 0; ii < sizeof(CommandPacket)-2; ++ii ) {
      Pkt.CHECK_SUM += ptr[ii];
    }

    _WriteComPort(m_PortHandle,(unsigned char *)(&Pkt),sizeof(CommandPacket));
  }


  ///////////////////////////////////////////////////////////////////////////////
  int ReadSensorPacket(SensorPacket& Pkt)
  {
    return _ReadComPort(m_PortHandle,(unsigned char *)(&Pkt),sizeof(SensorPacket));
  }


private:

  ///////////////////////////////////////////////////////////////////////////////
  void _CloseComPort(ComPortHandle comPort)
  {
    close(comPort);
    m_bIsConnected = false;

  }

  ///////////////////////////////////////////////////////////////////////////////
  int _ReadComPort(ComPortHandle comPort, unsigned char* bytes, size_t bytesToRead)
  {
    int bytesRead = read(comPort, bytes, bytesToRead);

    // align
    int ii = 0;
    while( bytes[ii] != FTDI_PACKET_DELIMITER1 && bytes[ii+1] != FTDI_PACKET_DELIMITER2 ) {
      ii++;
    }
    if( ii != 0 ) {
      PrintMessage("Lost a packet due to misalignment!\n");
      bytesRead = read(comPort, bytes, ii);
      return 0;
    }

    unsigned short int sum = 0;
    for( size_t ii = 0; ii < bytesToRead-2; ++ii ) {
      sum += bytes[ii];
    }
    unsigned short int chksum = *(unsigned short int*)(bytes + bytesToRead - 2);
    if( sum != chksum ) {
      PrintMessage("Checksum failed!\n");
      return 0;
    }

    return bytesRead;
  }

  ///////////////////////////////////////////////////////////////////////////////
  int _WriteComPort(ComPortHandle comPort, unsigned char* bytesToWrite, int size)
  {
    int bytesWritten = write(comPort, bytesToWrite, size);
    PrintMessage("wrote %d bytes\n",bytesWritten);
    return bytesWritten;
  }


  ///////////////////////////////////////////////////////////////////////////////
  int _Purge(ComPortHandle comPortHandle)
  {
    if (tcflush(comPortHandle,TCIOFLUSH)==-1){
      PrintMessage("Virtual com port flush failed!\n");
      return false;

    }
    return true;
  }

  ///////////////////////////////////////////////////////////////////////////////
  ComPortHandle _OpenComPort(const char* comPortPath,const int baudRate = B115200)
  {
    struct termios options;
    ComPortHandle comPort = open(comPortPath, O_RDWR | O_NOCTTY | O_SYNC /*| O_NONBLOCK*/ );

    if(comPort == -1) { //Opening of port failed
      PrintMessage("Unable to open com port '%s'\n Errno = %i, %s\n", comPortPath, errno,strerror(errno));
      return -1;
    }

    //Get the current options for the port...

    tcgetattr(comPort, &options);

    //set the baud rate to 115200

    cfsetospeed(&options, baudRate);
    cfsetispeed(&options, baudRate);

    //set the number of data bits.
    options.c_cflag &= ~CSIZE;  // Mask the character size bits
    options.c_cflag |= CS8;

    //set the number of stop bits to 1
    options.c_cflag &= ~CSTOPB;

    //Set parity to None
    options.c_cflag &=~PARENB;

    //set for non-canonical (raw processing, no echo, etc.)
    options.c_iflag = IGNPAR; // ignore parity check close_port(int
    options.c_oflag = 0; // raw output
    options.c_lflag = 0; // raw input

    //Time-Outs -- won't work with NDELAY option in the call to open
    options.c_cc[VMIN]  = sizeof(SensorPacket);   // block reading until RX x characers. If x = 0, it is non-blocking.
    options.c_cc[VTIME] = 100;   // Inter-Character Timer -- i.e. timeout= x*.1 s

    //Set local mode and enable the receiver
    options.c_cflag |= (CLOCAL | CREAD);

    //Purge serial port buffers
    _Purge(comPort);

    //Set the new options for the port...
    int status=tcsetattr(comPort, TCSANOW, &options);

    if (status != 0){ //For error message

      PrintMessage("Configuring com port failed.\n");
      return status;

    }

    //Purge serial port buffers
    _Purge(comPort);

    return comPort;

  }


private:
  /////////////////////////////////////////////////////////////////////////////////////////
  void _ThreadFunc()
  {
    hal::ImuMsg      pbIMU;
    hal::EncoderMsg  pbEncoder;
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

        hal::VectorMsg* pbVec = pbIMU.mutable_accel();
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
  ComPortHandle                   m_PortHandle;

  bool                            m_Running;
  std::thread                     m_CallbackThread;
  fPtr_IMU                        m_IMUCallback;
  fPtr_Encoder                    m_EncoderCallback;
};
