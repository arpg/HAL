#pragma once
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

#define FTDI_PACKET_DELIMITER1 0xD0
#define FTDI_PACKET_DELIMITER2 0xAA

#pragma pack(1)
struct CommandPacket
{
    char m_cDelimiter1;
    char m_cDelimiter2;
    char m_cSize;
    float  m_nSteering;
    float  m_nSpeed;
    char timestamp;
    unsigned short int m_nChksum;
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
    int         Enc_LB;
    int         Enc_LF;
    int         Enc_RB;
    int         Enc_RF;
    short int   ADC_Steer;
    short int   ADC_LB;
    short int   ADC_LF;
    short int   ADC_RB;
    short int   ADC_RF;
    char timestamp;
    unsigned short int chksum;
};


class ComportDriver
{

public:
    ComportDriver() : m_bIsConnected(false), m_bSimulateConnection(false) { }

    ~ComportDriver()
    {
      if(m_bIsConnected && !m_bSimulateConnection ){
        _CloseComPort(m_PortHandle);
      }
    }

    bool Connect(const std::string& path,int baudrate)
    {
      if( path == "sim"){
        printf("Simulating serial connection\n");
        m_bSimulateConnection = true;
        return true;
      }

      //open the given path
      m_PortHandle = _OpenComPort(path.c_str(),baudrate);

      if(m_PortHandle <= 0){
        printf("Failed to open port at specified baudrate, aborting...\n");
        m_bIsConnected = false;
      } else {
        m_bIsConnected = true;
      }
      return m_bIsConnected;
    }

    void SendCommandPacket( const float nSpeed, const float nSteering )
    {
      if( m_bSimulateConnection ){
        return;
      }

//    double dAccel = std::min(500.0,std::max(0.0,cmd.accel()));
//    double dPhi = std::min(500.0,std::max(0.0,cmd.phi()));
        static CommandPacket Pkt;
        Pkt.m_cDelimiter1 = FTDI_PACKET_DELIMITER1;
        Pkt.m_cDelimiter2 = FTDI_PACKET_DELIMITER2;
        Pkt.m_cSize = sizeof(CommandPacket);
        Pkt.m_nSpeed = nSpeed;
        Pkt.m_nSteering = nSteering;
        Pkt.timestamp++;
        Pkt.m_nChksum = _CalcChksum((unsigned char *)(&Pkt),sizeof(CommandPacket)-2);
        //printf("\n\n checksum isssss ::::: %d  -   %d    ,   %d\n\n",Pkt.m_nChksum,(unsigned char)(Pkt.m_nChksum & 0xFF),(unsigned char)((Pkt.m_nChksum >> 8)& 0xFF));
        //printf("nSpeed: %f   nSteering: %f \n",Pkt.m_nSpeed, Pkt.m_nSteering);
        _WriteComPort(m_PortHandle,(unsigned char *)(&Pkt),sizeof(CommandPacket));
    }

    int ReadSensorPacket(SensorPacket& Pkt)
    {
/*      if( m_bSimulateConnection ){
        static SensorPacket state = {};
        Pkt = state;
        state.Enc_LF++;
        state.Enc_LB++;
        state.Enc_RF++;
        state.Enc_RB++;
        state.ADC_Steer = 5.0*sin(0.01*state.Enc_LF);
        state.ADC_LF = 10.0*sin(0.01*state.Enc_LF);
        state.ADC_LB = 10.0*sin(0.01*state.Enc_LF);
        state.ADC_RF = 10.0*sin(0.01*state.Enc_LF);
        state.ADC_RB = 10.0*sin(0.01*state.Enc_LF);
        return true;
      }
*/
      int bytesRead =  _ReadComPort(m_PortHandle,(unsigned char *)(&Pkt),sizeof(SensorPacket));
      if( bytesRead && _CheckChksum((unsigned char *)(&Pkt),sizeof(SensorPacket)-2,Pkt.chksum))
	return bytesRead;
      else
	return 0;
    }


private:
    
    unsigned short int _CalcChksum(unsigned char* _data, int packDataSize)
    {
	unsigned short int sum = 0;
	for( int ii=0 ; ii<packDataSize ; ii++ )
		sum += _data[ii];
	return sum;
    }

    bool _CheckChksum(unsigned char* _data, int bytesReceived, unsigned short int RecChksum)
    {
	unsigned short int sum = 0;
	for( int ii=0 ; ii<bytesReceived ; ii++ ){
		sum += _data[ii];
	}
	if( RecChksum == sum ){
		return true;
	}else{
		printf("Wrong Checksum !!!   Calc: %d    Rec:%d\n",sum,RecChksum);
		return false;
	}
    }

    void _CloseComPort(ComPortHandle comPort)
    {
      close(comPort);
      m_bIsConnected = false;

    }

    int _ReadComPort(ComPortHandle comPort, unsigned char* bytes, int bytesToRead)
    {
      int bytesRead = read(comPort, bytes, bytesToRead);
      
//      printf("bitesRead was: %d\n",bytesRead);
//      for(int jj=0;jj<bytesRead;jj++)
//   	printf(" %d,",bytes[jj]);
//      printf("\n********************\n");

      // align
      int ii = 0;
      while( bytes[ii] != FTDI_PACKET_DELIMITER1 && bytes[ii+1] != FTDI_PACKET_DELIMITER2 ) {
          ii++;
      }
//      printf("  ii is: %d  ",ii);
      if( ii != 0 ) {
//          std::cout << "LOSING PACKET!" << std::endl;
          bytesRead = read(comPort, bytes, ii);
//          for(int jj=0;jj<bytesRead;jj++)
//              printf(" %d,",bytes[jj]);
//          printf("\n********************\n");
	return 0;
      }
      return bytesRead;

    }

    int _WriteComPort(ComPortHandle comPort, unsigned char* bytesToWrite, int size)
    {
        int bytesWritten = write(comPort, bytesToWrite, size);
        //printf("wrote %d bytes\n",bytesWritten);
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
    bool            m_bSimulateConnection;
};
