/* Gladiator IMU LM Series serial driver

Originally developed at Astrobotic (https://www.astrobotic.com). Used with permission

*/
#include "gladiator.h"

/* Rx constants */
#define RX_SYNC_BYTE (0x2a)

/*Tx constants */
#define TX_BYTE_PAUSE (0xd4)
#define TX_BYTE_SYNC (0x2b)
// CMD[0] field
#define TX_BYTE_OUTPUT_200 (0x02)
#define TX_BYTE_LOAD_COEF (0x05)
#define TX_BYTE_COEF_STORE (0x06)
#define TX_BYTE_OUTPUT_100 (0x0A)
#define TX_BYTE_RUN_NOW (0x0D)
// CMD[1] field: these values relate to the cmd[1] field in gladiator_tx_msg_t,
// when the cmd[0] field is set to TX_BYTE_LOAD_COEF.
#define COEF_FILTER_K (0x02)
#define COEF_OP_MODE (0x13)


#define MODE_IMU_MASK (0x00010000)
#define MODE_IMU_921K_BAUD (0x00020000)
#define MODE_IMU_100HZ (0x04 | MODE_IMU_MASK| MODE_IMU_921K_BAUD)
#define MODE_IMU_200HZ (0x01 | MODE_IMU_MASK| MODE_IMU_921K_BAUD)
#define MODE_IMU_500HZ (0x0C | MODE_IMU_MASK| MODE_IMU_921K_BAUD)
#define MODE_IMU_1000HZ (0x0E | MODE_IMU_MASK | MODE_IMU_921K_BAUD)

const uint8_t GladiatorIMU::msgSuccess[ 22 ] = {
  0x2b, // msg start character
  // the following is a null-terminated character string:"Dwnload Success"
  0x44, 0x77, 0x6e, 0x6c, 0x6f, 0x61, 0x64, 0x20,
  0x53, 0x75, 0x63, 0x63, 0x65, 0x73, 0x73, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x13 }; // checksum is last

// scale factors to convert raw IMU integer data to proper units

#define GYRO_TO_DEG_S (0.01f) // Gyro X,Y,Z
#define GYRO_TO_RAD_S (1.745329e-4f) // 0.01 * PI / 180.0
#define ACCL_TO_G (0.001f) // Accel X,Y,Z
#define ACCL_TO_M_S2 (9.81e-3f) // 0.001 * 9.81
#define TEMP_TO_DEG_C (0.01f) // from IMU channels


#define BUFFER_SIZE (1024)
#define SERIAL_TIMEOUT 1000

#define MSG_BUF_SIZE (128)
#define FLOAT_EPSILON 1e-6

GladiatorIMU::GladiatorIMU(const char* serPort)
{
  //Port needs to be set to RS485, 921600,8e2
  port = new CommPort();
  port->OpenPort(serPort);
  port->ChangeTermSpeed(921600);
  port->setRS485(1);
  port->setParity(CommPort::PARITY_EVEN);
  port->setStopBits(CommPort::STOP_BITS_TWO);

  //Zero out status bytes, etc
  revNumber[0] = revNumber[1] = 0;
  statusByte[0] = statusByte[1] = 0;
  gyroType = 0;
  accelScale = 0.0f;
  gyroScale = 0.0f;
}

GladiatorIMU:: ~GladiatorIMU()
{
  if (port)
    {
      port->ClosePort();
      delete port;
    }
}

/* Thread service routine */

void GladiatorIMU::run()
{
  printf("GladiatorIMU: Service thread started\n");

  void *newCmd;
  imu_data_t *data;
  while (1)
    {
      /* Check for commands */
      newCmd = GetCommand();
      if (newCmd)
	{
	  handleCommand((gladiator_cmd_t*) newCmd);
	}
      
      /* Pull a message */
      data = acquireData();

      /* Post some data */
      PutData(data);
    }
}

// sendMessage() : Sends a message/command to the IMU device. Assigns
// the fields within a properly formatted message and then computes
// a checksum and sends the message. Note that the IMU output is
// paused prior to sending a message.
//
// cmd[0], cmd[1], and payload need to be populated/set prior to call
//

void GladiatorIMU::sendMessage(gladiator_tx_msg_t *output )
{
  uint32_t i;
  uint8_t cksum;
  uint8_t *rawMessage;
  pauseOutput();
  rawMessage = (uint8_t*) output;
  output->sync = TX_BYTE_SYNC;
  output->cksum = 0; //a zero byte doesn't participate in the checksum

  cksum = 0;
  for( i=0; i < sizeof(gladiator_tx_msg_t); i++ )
    {
      cksum += rawMessage[i];
    }
  output->cksum = ~cksum + 1; //two's complement of sum
  port->Write((uint8_t*) output, sizeof(gladiator_tx_msg_t));

}

/* Load filter coefficients corresponding to given filter frequency:
// fVal : the new filter K value.
// 1.00f corresponds to 75Hz filter
// K=0.91f corresponds to 50Hz filter
// K=0.80f corresponds to 40Hz filter
// K=0.73f corresponds to 35Hz filter
// K=0.66f corresponds to 30Hz filter
// K=0.58f corresponds to 25Hz filter
// K=0.49f corresponds to 20Hz filter
// K=0.27f corresponds to 10Hz filter
*/

void GladiatorIMU::loadFilterK(float kVal)
{
  gladiator_tx_msg_t gMsg;
  gMsg.cmd[0] = TX_BYTE_LOAD_COEF;
  gMsg.cmd[1] = COEF_FILTER_K;
  gMsg.payload = *((uint32_t*)(&kVal)); //shove a float into a uint32_t 
  sendMessage(&gMsg); //first, load into active RAM
  
  usleep(10000); //sleep for 10ms

  //Save to onboard flash
  gMsg.cmd[0] = TX_BYTE_COEF_STORE;
  gMsg.cmd[1] = 1; //number of commands updated
  gMsg.payload = 0;
  sendMessage(&gMsg);

  usleep(10000); //sleep for 10ms

  //msgSuccess is returned - consume the bytes
  uint8_t rxBuffer[BUFFER_SIZE];
  uint8_t numRead = 0;
  numRead = port->Read(rxBuffer, sizeof(msgSuccess), SERIAL_TIMEOUT);
  if (numRead == sizeof(msgSuccess))
    printf("GladiatorIMU: Filter load successful, resyncing output\n");
  else
    {
      printf("GladiatorIMU: Filter load unsuccessful, received:\n");
      port->dumpBuffer(rxBuffer, numRead);
    }
}

//mode : either MODE_IMU_100HZ, MODE_IMU_200HZ or MODE_IMU_500HZ
void GladiatorIMU::setSampleRate(uint32_t mode)
{
  gladiator_tx_msg_t gMsg;
  gMsg.cmd[0] = TX_BYTE_LOAD_COEF;
  gMsg.cmd[1] = COEF_OP_MODE;
  gMsg.payload = mode;
  sendMessage(&gMsg); //first, load into active RAM
 
  usleep(10000); //sleep for 10ms

  //Save to onboard flash
  gMsg.cmd[0] = TX_BYTE_COEF_STORE;
  gMsg.cmd[1] = 1; //number of commands updated
  gMsg.payload = 0;
  sendMessage(&gMsg);

  //msgSuccess is returned - consume the bytes
  uint8_t rxBuffer[BUFFER_SIZE];
  //uint8_t numRead = 0;
  port->Read(rxBuffer, sizeof(msgSuccess), SERIAL_TIMEOUT);
  printf("GladiatorIMU: Change sample rate successful\n");
 
  //Apply it now
  gMsg.cmd[0] = TX_BYTE_RUN_NOW;
  gMsg.cmd[1] = 0;
  sendMessage(&gMsg);
}

// pauseOutput() : Call prior to sending a message to the IMU device.
// Since the IMU uses half-duplex communication, the IMU device
// needs shut off its output stream in order to properly receive
// an incoming command message. When the IMU receives a pause
// command, it will stop its output for several seconds. The
// command is repeated 10 times with a 1 millisecond spacing to
// ensure it is received while the IMU is in-between message
// output.
void GladiatorIMU::pauseOutput()
{
  int i;
  char cmd = TX_BYTE_PAUSE;
  // repeat several times to ensure success
  for( i=0; i < 10; i++ )
    {
      port->Write( (uint8_t* )&cmd, 1);
      usleep(1000);
    }
}

void GladiatorIMU::handleCommand(gladiator_cmd_t* cmd)
{ 
  uint8_t* payload;
  switch (cmd->command)
    {
    case CMD_LOADK:
      printf("GladiatorIMU: Loading filter K: %f\n", *((float*)&cmd->payload));
      loadFilterK(*((float*)&cmd->payload));
      break;
    case CMD_SETRATE:
      printf("GladiatorIMU: Setting rate %dHz\n", cmd->payload);
      switch (cmd->payload)
	{
	case 100:
	  setSampleRate(MODE_IMU_100HZ);
	  break;
	case 200:
	  setSampleRate(MODE_IMU_200HZ);
	  break;
	case 500:
	  setSampleRate(MODE_IMU_500HZ);
	  break;
	case 1000:
	  setSampleRate(MODE_IMU_1000HZ);
	  break;
	}

      break;
    case CMD_GETBOARD:
      printf("GladiatorIMU: Returning board number bytes\n");
      cmd->payload = 0;
      payload = (uint8_t*)&(cmd->payload);
      payload[0] = boardNumber[0];
      payload[1] = boardNumber[1];
      payload[2] = boardNumber[2];
      payload[3] = boardNumber[3];
      break;
    case CMD_GETSTATUS:
      printf("GladiatorIMU: Returning status bytes\n");
      cmd->payload = 0;
      payload = (uint8_t*)&(cmd->payload);
      payload[0] = statusByte[0];
      payload[1] = statusByte[1];
      payload[2] = filterNumber;
      payload[3] = 0;
      break;
    default:
      printf("GladiatorIMU: Unknown command: 0x%d\n", cmd->command);
      break;
    }
  
  cmd->signal();
}

imu_data_t* GladiatorIMU::acquireData()
{
  /* 
     First, pull a gladiator_rx_msg_t from the serial line, 
     with uSync ==  RX_SYNC_BYTE
  */
  int validMessage = 0;
  gladiator_rx_msg_t* fullMessage;
  imu_data_t *data;
  while (!validMessage)
    {

       fullMessage = getSingleMessage();
       if (fullMessage)
	 {
	   data = translate(fullMessage);
	   //We may have started with a message that doesn't include the scaling factors - loop around
	   if (!epsilonEqual(accelScale, 0, FLOAT_EPSILON))
	       validMessage = 1;
	 }
    }
  
  delete fullMessage;
  return data;
}

gladiator_rx_msg_t* GladiatorIMU::getSingleMessage()
{
  /* Overall plan: 
     Consume single bytes until the sync byte is read
     Then, consume sizeof(gladiator_rx_msg_t)-1 bytes
     Compare checksums
     Invalid -> return null
     valid -> return new msg

     Goal: Strike balance between syscall rate and reliability 
     Caller deletes storage
  */

  uint8_t foundSync = 0;
  uint8_t rxBuffer[BUFFER_SIZE];
  uint8_t numRead = 0;
  uint32_t i;
  uint8_t checkSum; //limit to 8 bits to make the scheme work 

  while (!foundSync)
    {
      numRead = port->Read(rxBuffer, 1, SERIAL_TIMEOUT);
      if (numRead == 0)
	{
	  printf("GladiatorIMU: Timeout on sync byte read\n");
	  return NULL;
	}
      if (rxBuffer[0] == RX_SYNC_BYTE)
	{
	  //Start of message found, continue
	  //printf("GladiatorIMU: Found sync\n");
	  foundSync = 1;
	}
      else
	{
	  //printf(" 0x%x", rxBuffer[0]);
	}
    }

  //Acquire the rest of the message
  numRead = port->Read(&rxBuffer[1], sizeof(gladiator_rx_msg_t)-1, SERIAL_TIMEOUT);

  //Compute checksum:
  checkSum = 0;
  for (i=0; i < sizeof(gladiator_rx_msg_t); i++)
    {
      checkSum += rxBuffer[i];
    }

  if (checkSum != 0)
    {
      printf("GladiatorIMU: Corrupt packet: computed checksum as 0x%x\n", checkSum);
      //return NULL;
    }
  
  gladiator_rx_msg_t *message = new gladiator_rx_msg_t;
  memcpy(message, rxBuffer, sizeof(gladiator_rx_msg_t));
  return message;
}

imu_data_t* GladiatorIMU::translate(gladiator_rx_msg_t* input)
{
  /* Translate device-specific message into the one specified by ICD */
 
  imu_data_t* output = new imu_data_t;
  #if _POSIX_TIMERS_ > 0
  //Systems that have clock_gettime
  struct timespec stamp;
  clock_gettime(CLOCK_REALTIME, &stamp);
  //On a 32-bit platform, struct timespec is a 8-byte structure; 64-bit is 16-byte
  //Promote as necessary
  output->tv_sec = stamp.tv_sec;
  output->tv_nsec = stamp.tv_nsec;
  #else
  //Mac branch:

  struct timeval tv;
  gettimeofday(&tv, 0);
  output->tv_sec = tv.tv_sec;
  output->tv_nsec = tv.tv_usec*1000;
  #endif
  
  //Once time is secure, deal with the status byte
  handleStatusByte(input);


  makeAccel(input->iAccX, &output->accel_x);
  makeAccel(input->iAccY, &output->accel_y);
  makeAccel(input->iAccZ, &output->accel_z);
  makeGyro(input->iGyrX, &output->gyro_x);
  makeGyro(input->iGyrY, &output->gyro_y);
  makeGyro(input->iGyrZ, &output->gyro_z);
  makeTemp(input->iTemp, &output->temp);
  return output;
}

void GladiatorIMU::handleStatusByte(gladiator_rx_msg_t *msg)
{
  //The status byte is used for a variety of things
  //First cut: is the msg counter 0 or 1:
  if ((msg->uCount == 0) && ((msg->uStatus & (1<<7)) == 0))
    {
      //Save the SW revision numbers
      revNumber[msg->uCount] = msg->uStatus & 0x3f;
    }
  else if ((msg->uCount == 0) && ((msg->uStatus & (1<<7)) != 0))
    {

      productCode = msg->uStatus & 0x3f;
    }
  else if ((msg->uCount == 1) && ((msg->uStatus & (1<<7)) == 0))
    {
      revNumber[msg->uCount] = msg->uStatus & 0x3f;
    }
  else if ((msg->uCount == 1) && ((msg->uStatus & (1<<7)) != 0))
    {
      releaseLevel = msg->uStatus & 0x3f;
    }
  else if (msg->uCount == 247)
    {
      filterNumber = msg->uStatus;
    }
  else if (msg->uCount >= 248 && msg->uCount < 252)
    {
      boardNumber[msg->uCount - 248] = msg->uStatus;
    }
  else if (msg->uCount >= 252)
    {
      serialNumber[msg->uCount - 252] = msg->uStatus;
    }
  else
    {
      //For other values of the msg counter, check bit six to determine flags (0) or scale factors(1) mode
      //According to the manual (IMU_USERGUIDE_USB_Rev_2013-06-28), this switches every other message
      if (msg->uStatus & 0x40)
	{
	  //printf("Count: %d\n", msg->uCount);
	  //Bit six set: scale factors
	  uint8_t accel_bits = msg->uStatus & 0x0e;
	  setAccelScale(accel_bits);
	  uint8_t gyro_bits = msg->uStatus & 0x31;
	  setGyroScale(gyro_bits);
	  //Save the value of the status byte:
	  statusByte[1] = msg->uStatus;
	}
      else
	statusByte[0] = msg->uStatus;
    }
}

void GladiatorIMU::makeAccel(int32_t src, int32_t* dest)
{
  //Given a uint32_t source accel (raw) from the unit, use its status byte to translate to units of g, then convert to 
  //ICD-defined units in *dest

  float accel;
  accel = src*accelScale; //accel is now in g
  accel *= 10000; //accel in 10000 g
  *dest = accel; //truncate 
}

void GladiatorIMU::makeGyro(int32_t src, int32_t* dest)
{
  //Given a uint32_t source gyro (raw) from the unit, use its status byte to translate to units of deg/s, then convert to 
  //ICD-defined units in *dest

  float gyro;
  gyro = src*gyroScale; //gyro is now in deg/s
  gyro *= 1000; //gyro in 1000 deg/s
  *dest = gyro; //truncate float
}

void GladiatorIMU::makeTemp(int16_t src, int16_t *dest)
{
   //Given a uint32_t source temp (raw) from the unit, use its status byte to translate to units of degrees C, then convert to 
  //ICD-defined units in *dest
  float temp;
  temp = src; //always in centideg, which is the ICD unit
  *dest = temp; //trunc the float
}

void GladiatorIMU::setAccelScale(uint8_t bits)
{

  switch (bits >> 1)
    {
    case 0x0:
      accelScale = 0.0010;
      break;
    case 0x1:
      accelScale = 0.0001;
      break;
    case 0x2:
      accelScale = 0.0001;
      break;
    case 0x3:
      accelScale = 0.0005;
      break;
    case 0x4:
      accelScale = 0.0005;
      break;
    case 0x5:
      accelScale = 0.0005;
      break;
    case 0x6:
      accelScale = 0.0010;
      break;
    case 0x7:
      accelScale = 0.0020;
      break;
    default:
      printf("GladiatorIMU: Got unknown accel scale bits: 0x%x\n", bits);
      break;
    }
}

void GladiatorIMU::setGyroScale(uint8_t bits)
{
  
  //I think the manual has an error on page 18: the gyro status bits are 5:4:0, not 5:4:1
  //bits input is already masked off as 0x31
  
    switch (bits)
    {
    case 0x00:
      gyroScale = 0.01;
      break;
    case 0x01:
      gyroScale = 0.01;
      break;
    case 0x10:
      gyroScale = 0.01;
      break;
    case 0x11:
      gyroScale = 0.015;
      break;
    case 0x12:
      gyroScale = 0.02;
      break;
    case 0x20:
      gyroScale = 0.01;
      break;
    case 0x21:
      gyroScale = 0.05;
      break;
    case 0x30:
      gyroScale = 0.01;
      break;
    case 0x31:
      gyroScale = 0.005;
      break;
    default:
      printf("GladiatorIMU: Got unknown gyro scale bits: 0x%x\n", bits);
      break;
    }

}

//From: http://stackoverflow.com/questions/3915138/comparing-floating-point-0
int GladiatorIMU::epsilonEqual( float a, float b, float epsilon )
{
    return fabsf( a - b ) < epsilon;
}


gladiator_cmd_t::gladiator_cmd_t(GladiatorIMU *host, GladiatorCommand m_req, uint32_t value) :
  ThreadedCommand(host)
{
  command = m_req;
  payload = value;
}

gladiator_cmd_t::~gladiator_cmd_t()
{

}
