#ifndef __COMMPORT_H_
#define __COMMPORT_H_
/*
A serial port wrapper class
Originally developed at Astrobotic (https://www.astrobotic.com). Used with permission
*/

//#include <linux/types.h>
#ifdef __LINUX__
#include <linux/serial.h>
#endif

#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <sys/time.h>
/* 

Why isn't this named SerialPort, one might ask?
There is a class as part of the mixed/epuck codebase that uses that name
Muy difficult to track down why this constructor was being called with the other destructor,
which by the way, tries to delete a STL string and crashes Player


Other option would have been to disable epuck code - this seemed like the better option

*/

#define DEFAULT_SERIAL_TIMEOUT 1000    //in ms

#define DEBUG_MSG(msg, fmt) {printf("%s:%s:", __FILE__, __FUNCTION__); printf(msg,fmt); printf("\n");}
#define DEBUG_MSG0(msg) {printf("%s:%s:", __FILE__, __FUNCTION__); printf(msg); printf("\n");}

//#define DEBUG

class CommPort
{
 public:
  CommPort();
  ~CommPort();

  enum { PARITY_NONE=0, PARITY_EVEN=1, PARITY_ODD=2};
  enum {STOP_BITS_ONE=0, STOP_BITS_TWO=1};

    // Open the terminal
    // Returns 0 on success
    int OpenPort(const char* portName);

    // Close the terminal
    // Returns 0 on success
    int ClosePort();

   // Set the terminal speed
    // Valid values are 9600, 19200, 115200
    // Returns 0 on success

    int ChangeTermSpeed(int speed);
    ssize_t Write(uint8_t *data, ssize_t len);
    ssize_t Read(uint8_t *data, ssize_t maxlen, int timeout);
    ssize_t ReadLine(uint8_t *data, ssize_t maxlen, int timeout);
    void dumpBuffer(uint8_t* buffer, int length);
    uint8_t SetFlowControl();
    uint8_t ClearFlowControl();
    int setParity(uint8_t parity);
    int setStopBits(uint8_t stopBits);


    void setRS485(uint8_t enable);

 private:

    int serial_fd;
    // Name of serial device used to communicate 
    char *serial_name;
    
    int connect_rate; // The final connection rate that we settle on
    int serial_timeout; //how long to wait for responses, in ms

    int64_t GetTime();
    
};

#endif
