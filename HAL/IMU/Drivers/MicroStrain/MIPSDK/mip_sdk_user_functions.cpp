/////////////////////////////////////////////////////////////////////////////
//
//! @file    mip_sdk_user_functions.c
//! @author  Nathan Miller
//! @version 1.0
//
//! @description Target-Specific Functions Required by the MIP SDK
//
// External dependencies:
//
//  mip.h
//
//! @copyright 2011 Microstrain.
//
//!@section LICENSE
//!
//! THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING
//! CUSTOMERS WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER
//! FOR THEM TO SAVE TIME. AS A RESULT, MICROSTRAIN SHALL NOT BE HELD LIABLE
//! FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY
//! CLAIMS ARISING FROM THE CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY
//! CUSTOMERS OF THE CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH
//! THEIR PRODUCTS.
//
/////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//
//Include Files
//
////////////////////////////////////////////////////////////////////////////////


#include "mip_sdk_user_functions.h"

#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <stdint.h>
#include <iostream>
#include <sys/time.h>
#include <sys/ioctl.h>

std::string DeviceById(int id, std::string device_name)
{
    //search /dev/serial for microstrain devices in Linux
    //FILE* instream = popen("find /dev/serial -print 2> /dev/null | grep -i microstrain","r");

  // device should be something like /dev/serial/microstrain for linux or
  //                                 /dev/cu.usbmodem* for MacOS
  std::string cmd = "find " + device_name + " -print 2> /dev/null";
  FILE* instream = popen(cmd.c_str(), "r");

    if(!instream) {
        std::cerr << "Unable to open pipe." << std::endl;
        return 0;
    }

    int devfound;
    char devname[255];
    for(devfound=-1; devfound < id && fgets(devname,sizeof(devname), instream); ++devfound) {
    }

    if( devfound >= id ) {
        devname[strlen(devname)-1] = '\0';
        return std::string(devname);
    } /*else {
        //search /dev/cu.usbmodem1 for microstrain devices in MacOS
      std::cerr << "looking for /dev/cu.usbmodemfa131 " << std::endl;
        instream = popen("find /dev/cu.usbmodemfa131 -print 2> /dev/null","r");

        for(devfound=-1; devfound < id && fgets(devname,sizeof(devname), instream); ++devfound) {
        }

        if( devfound >= id ) {
            devname[strlen(devname)-1] = '\0';
            return std::string(devname);
        }
    }*/

    return std::string();
}

typedef int ComPortHandle;

int Purge(ComPortHandle comPortHandle){
  if (tcflush(comPortHandle,TCIOFLUSH)==-1){
    printf("flush failed\n");
    return 0;
  }
  return 1;
}

/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_sdk_port_open(void *port_handle, int port_num, int baudrate)
//
//! @section DESCRIPTION
//! Target-Specific communications port open.
//
//! @section DETAILS
//!
//! @param [out] void *port_handle - target-specific port handle pointer (user needs to allocate memory for this)
//! @param [in] int port_num       - port number (as recognized by the operating system.)
//! @param [in] int baudrate       - baudrate of the com port.
//
//! @retval MIP_USER_FUNCTION_ERROR  When there is a problem opening the port.\n
//! @retval MIP_USER_FUNCTION_OK     The open was successful.\n
//
//! @section NOTES
//!
//! The user should copy the \c mip_sdk_user_functions.c file to their project directory and\n
//! edit it as needed to support their target operating system.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_sdk_port_open(void **port_handle, int port_num, std::string device_name, int /*baudrate*/)
{
    const std::string comPortPath = DeviceById(port_num, device_name);
    if( comPortPath.empty() ) {
        std::cerr << "Device not connected" << std::endl;
        return MIP_USER_FUNCTION_ERROR;
    }

    ComPortHandle comPort = open(comPortPath.c_str(), O_RDWR | O_NOCTTY);

    if (comPort== -1) {
      printf("Unable to open com Port %s\n Errno = %i\n", comPortPath.c_str(), errno);
      return -1;
    }

    //Get the current options for the port...
    struct termios options;
    tcgetattr(comPort, &options);

    //set the baud rate to 115200
    int baudRate = B115200;
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
    options.c_cc[VMIN]  = 0;   // block reading until RX x characers. If x = 0, it is non-blocking.
    options.c_cc[VTIME] = 100;   // Inter-Character Timer -- i.e. timeout= x*.1 s

    //Set local mode and enable the receiver
    options.c_cflag |= (CLOCAL | CREAD);

    //Purge serial port buffers
    Purge(comPort);

    //Set the new options for the port...
    int status=tcsetattr(comPort, TCSANOW, &options);

    if (status != 0){ //For error message

      printf("Configuring comport failed\n");
      return status;

    }

    //Purge serial port buffers
    Purge(comPort);

    *(ComPortHandle**)(port_handle) = new ComPortHandle(comPort);

    //User must replace this code
    return MIP_USER_FUNCTION_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_sdk_port_close(void *port_handle)
//
//! @section DESCRIPTION
//! Target-Specific port close function
//
//! @section DETAILS
//!
//! @param [in] void *port_handle - target-specific port handle pointer (user needs to allocate memory for this)
//
//! @retval MIP_USER_FUNCTION_ERROR  When there is a problem closing the port.\n
//! @retval MIP_USER_FUNCTION_OK     The close was successful.\n
//
//! @section NOTES
//!
//! The user should copy the \c mip_sdk_user_functions.c file to their project directory and\n
//! edit it as needed to support their target operating system.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_sdk_port_close(void *port_handle)
{
    ComPortHandle comPort = *(ComPortHandle*)port_handle;
    close(comPort);
    return MIP_USER_FUNCTION_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_sdk_port_write(void *port_handle, u8 *buffer, u32 num_bytes, u32 *bytes_written, u32 timeout_ms)
//
//! @section DESCRIPTION
//! Target-Specific Port Write Function.
//
//! @section DETAILS
//!
//! @param [in]  void *port_handle  - target-specific port handle pointer (user needs to allocate memory for this)
//! @param [in]  u8 *buffer         - buffer containing num_bytes of data
//! @param [in]  u32 num_bytes      - the number of bytes to write to the port
//! @param [out] u32 *bytes_written - the number of bytes actually written to the port
//! @param [in]  u32 timeout_ms     - the write timeout
//
//! @retval MIP_USER_FUNCTION_ERROR  When there is a problem communicating with the port.\n
//! @retval MIP_USER_FUNCTION_OK     The write was successful.\n
//
//! @section NOTES
//!
//! The user should copy the \c mip_sdk_user_functions.c file to their project directory and\n
//! edit it as needed to support their target operating system.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_sdk_port_write(void *port_handle, u8 *buffer, u32 num_bytes, u32 *bytes_written, u32 /*timeout_ms*/)
{
    ComPortHandle comPort = *(ComPortHandle*)port_handle;
    *bytes_written = write(comPort, buffer, num_bytes);
    return MIP_USER_FUNCTION_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_sdk_port_read(void *port_handle, u8 *buffer, u32 num_bytes, u32 *bytes_read, u32 timeout_ms)
//
//! @section DESCRIPTION
//! Target-Specific Port Write Function.
//
//! @section DETAILS
//!
//! @param [in]  void *port_handle  - target-specific port handle pointer (user needs to allocate memory for this)
//! @param [in]  u8 *buffer         - buffer containing num_bytes of data
//! @param [in]  u32 num_bytes      - the number of bytes to write to the port
//! @param [out] u32 *bytes_read    - the number of bytes actually read from the device
//! @param [in]  u32 timeout_ms     - the read timeout
//
//! @retval MIP_USER_FUNCTION_ERROR  When there is a problem communicating with the port.\n
//! @retval MIP_USER_FUNCTION_OK     The read was successful.\n
//
//! @section NOTES
//!
//! The user should copy the \c mip_sdk_user_functions.c file to their project directory and\n
//! edit it as needed to support their target operating system.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_sdk_port_read(void *port_handle, u8 *buffer, u32 num_bytes, u32 *bytes_read, u32 /*timeout_ms*/)
{
    ComPortHandle comPort = *(ComPortHandle*)port_handle;

    ssize_t b_read = read(comPort, buffer, num_bytes);

    if(b_read < 0 )
        return MIP_USER_FUNCTION_ERROR;

    *bytes_read = (u32) b_read;
    
    return MIP_USER_FUNCTION_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u32 mip_sdk_port_read_count(void *port_handle)
//
//! @section DESCRIPTION
//! Target-Specific Function to Get the Number of Bytes Waiting on the Port.
//
//! @section DETAILS
//!
//! @param [in]  void *port_handle  - target-specific port handle pointer (user needs to allocate memory for this)
//
//! @returns  Number of bytes waiting on the port,\n
//!           0, if there is an error.
//
//! @section NOTES
//!
//! The user should copy the \c mip_sdk_user_functions.c file to their project directory and\n
//! edit it as needed to support their target operating system.
//!
//
/////////////////////////////////////////////////////////////////////////////

u32 mip_sdk_port_read_count(void *port_handle)
{
    ComPortHandle comPort = *(ComPortHandle*)port_handle;
    int bytes_available;
    ioctl(comPort, FIONREAD, &bytes_available);
    return bytes_available;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u32 mip_sdk_get_time_ms()
//
//! @section DESCRIPTION
//! Target-Specific Call to get the current time in milliseconds.
//
//! @section DETAILS
//!
//! @param [in]  void *port_handle  - target-specific port handle pointer (user needs to allocate memory for this)
//
//! @returns  Current time in milliseconds.
//
//! @section NOTES
//!
//!   1) This value should no roll-over in short periods of time (e.g. minutes)\n
//!   2) Most systems have a millisecond counter that rolls-over every 32 bits\n
//!      (e.g. 49.71 days roll-over period, with 1 millisecond LSB)\n
//!   3) An absolute reference is not required since this function is\n
//!      used for relative time-outs.\n
//!   4) The user should copy the \c mip_sdk_user_functions.c file to their project directory and\n
//!      edit it as needed to support their target operating system.
//!
//
/////////////////////////////////////////////////////////////////////////////

u32 mip_sdk_get_time_ms()
{
    struct timeval now;
    gettimeofday(&now, NULL);
    return ((now.tv_sec) * 1000 + now.tv_usec/1000);
}
