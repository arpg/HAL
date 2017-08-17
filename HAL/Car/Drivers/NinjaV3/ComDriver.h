#ifndef COMDRIVER_H__
#define COMDRIVER_H__
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
#include <string>

typedef int ComPortHandle;

class ComportDriver
{

public:
    ComportDriver() : m_bIsConnected(false) { }

    ~ComportDriver()
    {
      if(m_bIsConnected){
        _CloseComPort();
      }
    }

    int WriteComPort(unsigned char* bytesToWrite, int size){
      return _WriteComPort(bytesToWrite,size);
    }

    bool Connect(const std::string& path,int baudrate, int readbufsize)
    {
      //open the given path
      m_readbuffsize = readbufsize;
      m_PortHandle = _OpenComPort(path.c_str(),baudrate);
      if(m_PortHandle <= 0){
        printf("Failed to open port at specified baudrate, aborting...\n");
        m_bIsConnected = false;
      } else {
        m_bIsConnected = true;
      }
      return m_bIsConnected;
    }

    int ReadSensorPacket(unsigned char* data, int size)
    {
      return _ReadComPort(data,size);
    }

    void Disconnect(){
      if(m_bIsConnected){
        _CloseComPort();
        m_bIsConnected = false;
      }
    }

private:
    
    void _CloseComPort()
    {
      close(m_PortHandle);
      m_bIsConnected = false;

    }

    int _ReadComPort(unsigned char* bytes, int bytesToRead)
    {
      int bytesRead = read(m_PortHandle, bytes, bytesToRead);

      return bytesRead;

    }

    int _WriteComPort(unsigned char* bytesToWrite, int size)
    {
        int bytesWritten = write(m_PortHandle, bytesToWrite, size);
        return bytesWritten;
    }

    int _Purge(ComPortHandle comPortHandle)
    {
      if (tcflush(comPortHandle,TCIOFLUSH)==-1){

        printf("flush failed\n");
        return false;

      }
      return true;
    }

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

      //set the baud rate

//      cfsetospeed(&options, baudRate);
//      cfsetispeed(&options, baudRate);
      cfsetospeed(&options, B115200);
      cfsetispeed(&options, B115200);


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
      options.c_cc[VMIN]  = m_readbuffsize;   // block reading until RX x characers. If x = 0, it is non-blocking.
      options.c_cc[VTIME] = 1;   // Inter-Character Timer -- i.e. timeout= x*.1 s

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
    int             m_readbuffsize;
};

#endif  //COMDRIVER_H__
