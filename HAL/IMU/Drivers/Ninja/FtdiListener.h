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
    FtdiListener() : m_bIsConnected(false)
    {
    }

    ///////////////////////////////////////////////////////////////////////////////
    ~FtdiListener()
    {
        if(m_bIsConnected){
            _CloseComPort(m_PortHandle);
        }
    }

    ///////////////////////////////////////////////////////////////////////////////
    bool Connect(const char *path)
    {
        if( m_bIsConnected == false ) {
            //open the given path
            m_PortHandle = _OpenComPort(path, B115200);

            if(m_PortHandle <= 0){
                printf("Failed to open at 115200bps, aborting...\n");
            } else {
                m_bIsConnected = true;
            }
        }
        return m_bIsConnected;
    }


    ///////////////////////////////////////////////////////////////////////////////
    void Disconnect()
    {
        if(m_bIsConnected) {
            _CloseComPort(m_PortHandle);
        }
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
    int _ReadComPort(ComPortHandle comPort, unsigned char* bytes, int bytesToRead)
    {
      int bytesRead = read(comPort, bytes, bytesToRead);

      // align
      int ii = 0;
      while( bytes[ii] != FTDI_PACKET_DELIMITER1 && bytes[ii+1] != FTDI_PACKET_DELIMITER2 ) {
          ii++;
      }
      if( ii != 0 ) {
          std::cerr << "HAL: Lost a packet due to misalignment!" << std::endl;
          bytesRead = read(comPort, bytes, ii);
      }

      return bytesRead;

    }

    ///////////////////////////////////////////////////////////////////////////////
    int _WriteComPort(ComPortHandle comPort, unsigned char* bytesToWrite, int size)
    {
        int bytesWritten = write(comPort, bytesToWrite, size);
        //printf("wrote %d bytes\n",bytesWritten);
        return bytesWritten;
    }


    ///////////////////////////////////////////////////////////////////////////////
    int _Purge(ComPortHandle comPortHandle)
    {
      if (tcflush(comPortHandle,TCIOFLUSH)==-1){

        printf("flush failed\n");
        return false;

      }
      return true;
    }

    ///////////////////////////////////////////////////////////////////////////////
    ComPortHandle _OpenComPort(const char* comPortPath,const int baudRate = B115200)
    {
      struct termios options;
      ComPortHandle comPort = open(comPortPath, O_RDWR | O_NOCTTY | O_SYNC /*| O_NONBLOCK*/ );

      if (comPort== -1){ //Opening of port failed

        printf("Unable to open com Port %s\n Errno = %i, %s\n", comPortPath, errno,strerror(errno));
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

        printf("Configuring comport failed\n");
        return status;

      }

      //Purge serial port buffers
      _Purge(comPort);

      return comPort;

    }


private:
    bool            m_bIsConnected;
    ComPortHandle   m_PortHandle;

};
