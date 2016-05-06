/* Basic serial port class abstraction
   Originally developed at Astrobotic (https://www.astrobotic.com). Used with permission
 */
#include "commport.h"
#define COMMPORT_MAX_READLINE_TIMEOUTS 3

#ifdef __MACH__
#define B921600 921600
#define B460800 460800

#endif

CommPort::CommPort()
{
  this->serial_name = NULL;
}

CommPort::~CommPort()
{
  ClosePort();
  if (this->serial_name)
    {
      delete [] this->serial_name;
    }
}

void CommPort::dumpBuffer(uint8_t* buffer, int length)
{
  int i;
  for (i=0; i<length; i++)
    {
      printf("0x%x ", buffer[i]);
    }
  printf("\n");
}

////////////////////////////////////////////////////////////////////////////////
// Open the terminal
// Returns 0 on success
int CommPort::OpenPort(const char* portName)
{
  this->serial_name = new char[strlen(portName)];

  strcpy(this->serial_name, portName);

  this->serial_fd = ::open(this->serial_name, O_RDWR | O_SYNC , S_IRUSR | S_IWUSR );

  if (this->serial_fd < 0)
  {
    DEBUG_MSG("unable to open serial port [%s]",
                  (char*) this->serial_name);
    return 1;
  }

  // set the serial port speed to 115200 or as specified
  //
  struct termios term;
  if( tcgetattr( this->serial_fd, &term ) < 0 )
    {
      DEBUG_MSG0("Unable to get serial port attributes");
      return 1;
    }

  cfmakeraw( &term );

  cfsetispeed( &term, B115200 );
  cfsetospeed( &term, B115200 );

  if( tcsetattr( this->serial_fd, TCSAFLUSH, &term ) < 0 )
    {
      DEBUG_MSG0("Unable to set serial port attributes");
      return 1;
    }

  // Make sure queue is empty
  //
  tcflush(this->serial_fd, TCIOFLUSH);

  return 0;
}
////////////////////////////////////////////////////////////////////////////////
// Close the terminal
// Returns 0 on success
//
int CommPort::ClosePort()
{
  ::close(this->serial_fd);
  return 0;
}
////////////////////////////////////////////////////////////////////////////////
// Set the terminal speed
// Returns 0 on success
//
int CommPort::ChangeTermSpeed(int speed)
{
  struct termios term;

  // First set the common terminal settings
  if( tcgetattr( this->serial_fd, &term ) < 0 )
    {
      DEBUG_MSG0("Unable to get device attributes");
      return 1;
    }

  cfmakeraw( &term );

  switch(speed)
  {
  case 1200:
    cfsetispeed( &term, B1200 );
    cfsetospeed( &term, B1200 );
    break;
  case 2400:
    cfsetispeed( &term, B2400 );
    cfsetospeed( &term, B2400 );
    break;
  case 9600:
    cfsetispeed( &term, B9600 );
    cfsetospeed( &term, B9600 );
    break;

  case 19200: 
    cfsetispeed( &term, B19200 );
    cfsetospeed( &term, B19200 );
    break;
    
  case 38400: 
    cfsetispeed( &term, B38400 );
    cfsetospeed( &term, B38400 );
    break;
    
  case 57600: 
    cfsetispeed( &term, B57600 );
    cfsetospeed( &term, B57600 );
    break;

  case 115200:
    cfsetispeed( &term, B115200 );
    cfsetospeed( &term, B115200 );
    break;
    
  case 230400:
    cfsetispeed( &term, B230400 );
    cfsetospeed( &term, B230400 );
    break;

    
  case 460800:
	cfsetispeed(&term, B460800);
	cfsetospeed(&term, B460800);
	break;
  case 921600:
	cfsetispeed(&term, B921600);
	cfsetospeed(&term, B921600);
	break;	
  default:
    DEBUG_MSG("unknown speed %d", speed);
    return 1;
  }
    
  if( tcsetattr( this->serial_fd, TCSAFLUSH, &term ) < 0 )
      {
	DEBUG_MSG0("Unable to set device attributes");
	return 1;
      }
  return 0;
}


////////////////////////////////////////////////////////////////////////////////
// Write a packet
//
ssize_t CommPort::Write(uint8_t *data, ssize_t len)
{
  // Make sure both input and output queues are empty
  //
  tcflush(this->serial_fd, TCIOFLUSH);


  ssize_t bytes = 0;
  bytes = ::write( this->serial_fd, data, len);

  // Make sure the queue is drained
  // Synchronous IO doesnt always work
  
#ifdef DEBUG
  dumpBuffer(data, len);  
#endif

  ::tcdrain(this->serial_fd);

  // Return the actual number of bytes sent
  //
  return bytes;
}
////////////////////////////////////////////////////////////////////////////////
// Get the time (in ms)
//
int64_t CommPort::GetTime()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  //GlobalTime->GetTime(&tv);
  return (int64_t) tv.tv_sec * 1000 + (int64_t) tv.tv_usec / 1000;
}



////////////////////////////////////////////////////////////////////////////////
// Read a packet from the port
// Set timeout to -1 to make this blocking, otherwise it will return in timeout ms.
// Returns the packet length (0 if timeout occurs)
//

ssize_t CommPort::Read(uint8_t *data, ssize_t maxlen, int timeout)
{
  int64_t start_time = GetTime();
  int64_t stop_time = start_time + timeout;

  int bytes = 0;

  // Read in the data, up to a maxlen max

  bytes = 0;

  while (bytes < maxlen)
  {
    if (timeout >= 0)
    {
      fd_set rfds, efds;
      FD_ZERO(&rfds);
      FD_SET(this->serial_fd, &rfds);
      FD_ZERO(&efds);
      FD_SET(this->serial_fd, &efds);
      int64_t delay = stop_time - GetTime();
      if (delay < 0) delay = 0;
      struct timeval tv;
      tv.tv_usec = (delay % 1000) * 1000;
      tv.tv_sec = delay / 1000;
      int retval = ::select(this->serial_fd + 1, &rfds, 0, &efds, &tv);
      if (retval < 0)
      {
        if (errno == EINTR)
	      continue;
	if (errno == EBADF)
	  return 0;

        DEBUG_MSG0("error on select (3)");
	perror("select:");
        return 0;
      }
      if (!retval)
      {
        //DEBUG_MSG0("timeout on select (3)");
        return 0;
      }
    }
    int retval = ::read(this->serial_fd, data + bytes, maxlen - bytes);
    if (retval < 0)
    {
      if (errno == EINTR)
	    continue;
      DEBUG_MSG0("error on read (3)");
      return 0;
    }
    if (!retval)
    {
      DEBUG_MSG0("eof on read (3)");
      return 0;
    }
    bytes += retval;
  }

  return bytes;
}

/* Read until a newline is picked up 
   Or until a set number of timeouts occur?
*/

ssize_t CommPort::ReadLine(uint8_t *data, ssize_t maxlen, int timeout)
{
  int totalBytes = 0;
  int bytesRead = 0;
  int attempts = 0;
  //Respect max buffer length
  while (totalBytes < maxlen)
    {
      bytesRead = this->Read(&data[totalBytes], 1, timeout);

      if (bytesRead == 0)
	{
	  attempts++;
	  if (attempts > COMMPORT_MAX_READLINE_TIMEOUTS)
	    {
	      printf("%s: Max ReadLine() attempts exceeded\n", __FILE__);
	      return 0;
	    }
	}
      totalBytes += bytesRead;

      //printf("CommPort::RL: Added %d bytes\n", bytesRead);
      //dumpBuffer(data, totalBytes);

      if (data[totalBytes-1] == '\x0d')
      	break;
      //printf("byte: %d val: 0x%x\n", totalBytes, data[totalBytes-1]);
    }
  return totalBytes;
}

uint8_t CommPort::SetFlowControl()
{
   struct termios term;
   struct termios new_term;
  // First set the common terminal settings
  if( tcgetattr( this->serial_fd, &term ) < 0 )
    {
      DEBUG_MSG0("Unable to get device attributes");
      return 1;
    }

  term.c_cflag |= CRTSCTS;
  tcsetattr( this->serial_fd, TCSANOW, &term );
  tcgetattr( this->serial_fd, &new_term );
  if ( memcmp(&term, &new_term,
	      sizeof(term )) != 0 )
    {
      /* do some error handling */
      DEBUG_MSG0("Unable to enable flow control");
      return 1;
    }
  return 0;
}

uint8_t CommPort::ClearFlowControl()
{
   struct termios term;
   struct termios new_term;
  // First set the common terminal settings
  if( tcgetattr( this->serial_fd, &term ) < 0 )
    {
      DEBUG_MSG0("Unable to get device attributes");
      return 1;
    }

  term.c_cflag &= ~CRTSCTS;
  tcsetattr( this->serial_fd, TCSANOW, &term );
  tcgetattr( this->serial_fd, &new_term );
  if ( memcmp(&term, &new_term,
	      sizeof(term )) != 0 )
    {
      /* do some error handling */
      DEBUG_MSG0("Unable to disable flow control");
      return 1;
    }
  return 0;
}

int CommPort::setParity(uint8_t parity)
{
  struct termios term;
  struct termios new_term;
  // First set the common terminal settings
  if( tcgetattr( this->serial_fd, &term ) < 0 )
    {
      DEBUG_MSG0("Unable to get device attributes");
      return 1;
    }

  switch (parity)
    {
    case PARITY_NONE:
      term.c_cflag &= ~PARENB; //disable parity
      break;
    case PARITY_EVEN:
      term.c_cflag |= PARENB; //enable parity
      term.c_cflag &= ~PARODD; //clear odd bit
      break;
    case PARITY_ODD:
      term.c_cflag |= PARENB; //enable parity
      term.c_cflag |= PARODD; //enable odd bit
      break;
    default:
      DEBUG_MSG0("Unknown parity request");
      break;
    }


  tcsetattr( this->serial_fd, TCSANOW, &term );
  tcgetattr( this->serial_fd, &new_term );
  if ( term.c_cflag != new_term.c_cflag)
    {
      /* do some error handling */
      DEBUG_MSG0("Unable to set parity");
      return 1;
    }
  return 0;
}

int CommPort::setStopBits(uint8_t stopBits)
{
  struct termios term;
  struct termios new_term;
  // First set the common terminal settings
  if( tcgetattr( this->serial_fd, &term ) < 0 )
    {
      DEBUG_MSG0("Unable to get device attributes");
      return 1;
    }

  switch (stopBits)
    {
    case STOP_BITS_ONE:
      term.c_cflag &= ~CSTOPB; 
      break;
    case STOP_BITS_TWO:
      term.c_cflag |= CSTOPB; 
      break;
    default:
      DEBUG_MSG0("Unknown stop bit request");
      break;
    }


  tcsetattr( this->serial_fd, TCSANOW, &term );
  tcgetattr( this->serial_fd, &new_term );
  if ( term.c_cflag != new_term.c_cflag)
    {
      /* do some error handling */
      DEBUG_MSG0("Unable to set stop bits");
      return 1;
    }
  return 0;
}

/* Nonzero to enable, 0 to disable */

void CommPort::setRS485(uint8_t enable)
{
  #ifdef __LINUX__
  //Taken from http://armbedded.eu/node/322

  struct serial_rs485 rs485conf;

  //Set port for 485 mode
  if(enable)
    {
      rs485conf.flags = 0;
      rs485conf.flags |= SER_RS485_ENABLED;
      rs485conf.delay_rts_before_send = 0x00000004;
      ioctl(serial_fd, TIOCSRS485, &rs485conf);
    }
  else
    {
      rs485conf.flags = 0;
      rs485conf.flags &= ~SER_RS485_ENABLED;
      ioctl(serial_fd, TIOCSRS485, &rs485conf);
    }

  #endif
  //Remove local echo (disable rx during tx)
  struct termios options;

  tcgetattr(serial_fd, &options);
  if(enable)
    options.c_cflag |= CREAD;
  else
    options.c_cflag &= ~CREAD;
  tcsetattr(serial_fd, TCSANOW, &options);
  fcntl(serial_fd, F_SETFL, FNDELAY);
}
