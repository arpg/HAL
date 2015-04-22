#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <iostream>
#include <stdexcept>

#include <HAL/config.h>

#include "Reader.h"

#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/io/coded_stream.h>
#include <glog/logging.h>

namespace hal {

Reader& Reader::Instance( const std::string& filename, MessageType eType ) {
  static Reader m_Instance(filename);
  if( eType == Msg_Type_Camera ) {
    m_Instance.m_bReadCamera = true;
  }
  if( eType == Msg_Type_Encoder ) {
    m_Instance.m_bReadEncoder = true;
  }
  if( eType == Msg_Type_IMU ) {
    m_Instance.m_bReadIMU = true;
  }
  if( eType == Msg_Type_LIDAR ) {
    m_Instance.m_bReadLIDAR = true;
  }
  if( eType == Msg_Type_Posys ) {
    m_Instance.m_bReadPosys = true;
  }
  return m_Instance;
}

Reader::Reader(const std::string& filename) : m_bRunning(true),
                                              m_bShouldRun(false),
                                              m_bReadCamera(false),
                                              m_bReadEncoder(false),
                                              m_bReadIMU(false),
                                              m_bReadLIDAR(false),
                                              m_bReadPosys(false),
                                              m_nInitialImageID(0),
  m_nMaxBufferSize(10) {
  _BufferFromFile(filename);
}

bool Reader::_AmINext(MessageType eMsgType) {
  if( m_qMessageTypes.empty() ) {
    return false;
  }

  return m_qMessageTypes.front() == eMsgType;
}

Reader::~Reader() {
  StopBuffering();
}

void Reader::_ThreadFunc() {
  int fd = open(m_sFilename.c_str(), O_RDONLY);

  if(fd == -1) {
    std::cerr << "HAL: File '"<< m_sFilename
              << "' could not be opened. Does it exist?" << std::endl;
    return;
  }

  google::protobuf::io::FileInputStream raw_input(fd);
  raw_input.SetCloseOnDelete(true);

  ///-------------------- Read Magic Number
  char magic_number[4];

  {
    google::protobuf::io::CodedInputStream coded_input(&raw_input);
    coded_input.ReadRaw(magic_number, 4);
  }

  if( magic_number[0] != '%' || magic_number[1] != 'H' ||
      magic_number[2] != 'A' || magic_number[3] != 'L' ) {
    std::cerr << "HAL: File '"<< m_sFilename
              << "' not in expected format (wrong magic number)." << std::endl;
    return;
  }


  ///-------------------- Read Header Message
  uint32_t hdr_size_bytes;
  {
    google::protobuf::io::CodedInputStream coded_input(&raw_input);
    if( !coded_input.ReadVarint32(&hdr_size_bytes) ) {
      std::cerr << "HAL: Error while reading HEADER message size." << std::endl;
      return;
    }

    google::protobuf::io::CodedInputStream::Limit lim =
        coded_input.PushLimit(hdr_size_bytes);
    if( !m_Header.ParseFromCodedStream(&coded_input) ) {
      std::cerr << "HAL: Error while parsing from coded stream. "
                << "Has the HEADER Proto file definitions changed?"
                << std::endl;
      return;
    }
    coded_input.PopLimit(lim);
  }

  // check if version numbers match
  if( m_Header.version() != Messages_VERSION ) {
    std::cerr << "HAL: Log was recorded using a different "
              << "Messages version and it is unreadable!" << std::endl;
    return;
  }


  ///-------------------- Read Message Log
  size_t nImgID = 0;
  m_bRunning = true;

  while( m_bShouldRun ){
    google::protobuf::io::CodedInputStream coded_input(&raw_input);

    uint32_t msg_size_bytes;
    if( !coded_input.ReadVarint32(&msg_size_bytes) ) {
      // Probably end of stream.
      std::cerr << "HAL: Error while reading message size." << std::endl;
      break;
    }

    google::protobuf::io::CodedInputStream::Limit lim =
        coded_input.PushLimit(msg_size_bytes);
    std::unique_ptr<hal::Msg> pMsg(new hal::Msg);
//      This error message is inaccurate, so squelching it for now.
    if( !pMsg->ParseFromCodedStream(&coded_input) ) {
//      std::cerr << "HAL: Error while parsing from coded stream. "
//                << "Has the Proto file definitions changed?" << std::endl;
      break;
    }
    coded_input.PopLimit(lim);

    // Wait if buffer is full, then add to queue
    std::unique_lock<std::mutex> lock(m_QueueMutex);
    while(m_bShouldRun && m_qMessages.size() >= m_nMaxBufferSize){
      m_ConditionDequeued.wait_for(lock, std::chrono::milliseconds(10) );
    }

    if( nImgID < m_nInitialImageID ) {
      nImgID++;
      continue;
    }

    bool has_camera  = pMsg->has_camera();
    bool has_encoder = pMsg->has_encoder();
    bool has_imu     = pMsg->has_imu();
    bool has_lidar   = pMsg->has_lidar();
    bool has_pose    = pMsg->has_pose();

    int num_message_types =
        (has_camera + has_encoder + has_imu + has_lidar + has_pose);
    if (num_message_types == 0) {
      LOG(WARNING) << "Message with no known data types found";
    } else if (num_message_types > 1) {
      LOG(ERROR) << "Message with more than one data type found.";
    }

    MessageType msg_type;
    if (has_camera) {
      msg_type = Msg_Type_Camera;
    } else if (has_encoder) {
      msg_type = Msg_Type_Encoder;
    } else if (has_imu) {
      msg_type = Msg_Type_IMU;
    } else if (has_lidar) {
      msg_type = Msg_Type_LIDAR;
    } else if (has_pose) {
      msg_type = Msg_Type_Posys;
    }

    if ((has_camera  && m_bReadCamera)  ||
        (has_encoder && m_bReadEncoder) ||
        (has_imu     && m_bReadIMU)     ||
        (has_lidar   && m_bReadLIDAR)   ||
        (has_pose    && m_bReadPosys)) {
      m_qMessageTypes.push_back(msg_type);
      m_qMessages.push_back(std::move(pMsg));
      m_ConditionQueued.notify_one();
    }
  }

  m_bRunning = false;
}

std::unique_ptr<hal::Msg> Reader::ReadMessage() {
  // Wait if buffer is empty
  std::unique_lock<std::mutex> lock(m_QueueMutex);
  while( m_bRunning && m_qMessages.empty() ){
    m_ConditionQueued.wait_for(lock, std::chrono::milliseconds(10));
  }

  if(!m_qMessages.empty()) {
    std::unique_ptr<hal::Msg> pMessage = std::move(m_qMessages.front());
    m_qMessages.pop_front();
    m_qMessageTypes.pop_front();
    m_ConditionDequeued.notify_one();
    return pMessage;
  }else{
    return nullptr;
  }
}

std::unique_ptr<hal::CameraMsg> Reader::ReadCameraMsg(int id) {
  if( !m_bReadCamera ) {
    std::cerr << "warning: ReadCameraMsg was called but"
              << " ReadCamera variable is set to false! " << std::endl;
    return nullptr;
  }

  // Wait if buffer is empty
  std::unique_lock<std::mutex> lock(m_QueueMutex);
  while(m_bRunning && !_AmINext( Msg_Type_Camera ) ){
    m_ConditionQueued.wait_for(lock, std::chrono::milliseconds(10));
  }

  if(!m_bRunning || m_qMessages.empty()) {
    return nullptr;
  }

  std::unique_ptr<hal::CameraMsg> pCameraMsg(new hal::CameraMsg);
  pCameraMsg->Swap(m_qMessages.front()->mutable_camera());
  pCameraMsg->set_system_time(m_qMessages.front()->timestamp());

  m_qMessages.pop_front();
  m_qMessageTypes.pop_front();
  m_ConditionDequeued.notify_one();

  // message popped above might be some other id than the one wanted.
  // this might cause some frame dropping.
  if(id >= 0 && pCameraMsg->id() != id) {
    return nullptr;
  }

  return pCameraMsg;
}

std::unique_ptr<hal::EncoderMsg> Reader::ReadEncoderMsg() {
  if( !m_bReadEncoder ) {
    std::cerr << "warning: ReadEncoderMsg was called but"
              << "ReadEncoder variable is set to false! " << std::endl;
    return nullptr;
  }

  // Wait if buffer is empty
  std::unique_lock<std::mutex> lock(m_QueueMutex);
  while(m_bRunning && !_AmINext( Msg_Type_Encoder ) ){
    m_ConditionQueued.wait_for(lock, std::chrono::milliseconds(10));
  }

  if(!m_bRunning || m_qMessages.empty()) {
    return nullptr;
  }

  std::unique_ptr<hal::Msg> pMessage = std::move(m_qMessages.front());
  m_qMessages.pop_front();
  m_qMessageTypes.pop_front();
  m_ConditionDequeued.notify_one();

  std::unique_ptr<hal::EncoderMsg> pEncoderMsg( new hal::EncoderMsg );
  pEncoderMsg->Swap( pMessage->mutable_encoder() );
  return pEncoderMsg;
}

std::unique_ptr<hal::ImuMsg> Reader::ReadImuMsg() {
  if( !m_bReadIMU ) {
    std::cerr << "warning: ReadImuMsg was called but ReadIMU variable is set to false! " << std::endl;
    return nullptr;
  }

  // Wait if buffer is empty
  std::unique_lock<std::mutex> lock(m_QueueMutex);
  while(m_bRunning && !_AmINext( Msg_Type_IMU ) ){
    m_ConditionQueued.wait_for(lock, std::chrono::milliseconds(10));
  }

  if(!m_bRunning || m_qMessages.empty()) {
    return nullptr;
  }

  std::unique_ptr<hal::Msg> pMessage = std::move(m_qMessages.front());
  m_qMessages.pop_front();
  m_qMessageTypes.pop_front();
  m_ConditionDequeued.notify_one();

  std::unique_ptr<hal::ImuMsg> pImuMsg( new hal::ImuMsg );
  pImuMsg->Swap( pMessage->mutable_imu() );
  pImuMsg->set_system_time(pMessage->timestamp());
  return pImuMsg;
}

std::unique_ptr<hal::LidarMsg> Reader::ReadLidarMsg() {
  if( !m_bReadLIDAR ) {
    std::cerr << "warning: ReadLidarMsg was called but ReadLIDAR variable is set to false! " << std::endl;
    return nullptr;
  }

  // Wait if buffer is empty
  std::unique_lock<std::mutex> lock(m_QueueMutex);
  while(m_bRunning && !_AmINext( Msg_Type_LIDAR ) ){
    m_ConditionQueued.wait_for(lock, std::chrono::milliseconds(10));
  }

  if(!m_bRunning || m_qMessages.empty()) {
    return nullptr;
  }

  std::unique_ptr<hal::Msg> pMessage = std::move(m_qMessages.front());
  m_qMessages.pop_front();
  m_qMessageTypes.pop_front();
  m_ConditionDequeued.notify_one();

  std::unique_ptr<hal::LidarMsg> pLidarMsg( new hal::LidarMsg );
  pLidarMsg->Swap( pMessage->mutable_lidar() );
  return pLidarMsg;
}

std::unique_ptr<hal::PoseMsg> Reader::ReadPoseMsg() {
  if( !m_bReadPosys ) {
    std::cerr << "warning: ReadPoseMsg was called but ReadPose variable is set to false! " << std::endl;
    return nullptr;
  }

  // Wait if buffer is empty
  std::unique_lock<std::mutex> lock(m_QueueMutex);
  while(m_bRunning && !_AmINext( Msg_Type_Posys ) ){
    m_ConditionQueued.wait_for(lock, std::chrono::milliseconds(10));
  }

  if(!m_bRunning || m_qMessages.empty()) {
    return nullptr;
  }

  std::unique_ptr<hal::Msg> pMessage = std::move(m_qMessages.front());
  m_qMessages.pop_front();
  m_qMessageTypes.pop_front();
  m_ConditionDequeued.notify_one();

  std::unique_ptr<hal::PoseMsg> pPoseMsg( new hal::PoseMsg );
  pPoseMsg->Swap( pMessage->mutable_pose() );
  return pPoseMsg;
}

bool Reader::_BufferFromFile(const std::string& fileName) {
  m_sFilename = fileName;
  m_bShouldRun = true;
  m_ReadThread = std::thread( &Reader::_ThreadFunc, this );
  return true;
}

void Reader::StopBuffering() {
  if(m_bRunning)
  {
    m_bShouldRun = false;
    m_ConditionDequeued.notify_one();
  }

  if(m_ReadThread.joinable()) {
    m_ReadThread.join();
  }

  m_ConditionQueued.notify_all();
}

bool Reader::SetInitialImage(size_t nImgID) {
  if( m_sFilename.empty() ) {
    return false;
  }

  // kill reading thread if alive
  if( m_ReadThread.joinable() ) {
    m_bShouldRun = false;
    m_ReadThread.join();
    m_qMessages.clear();
    m_qMessageTypes.clear();
  }

  m_nInitialImageID = nImgID;
  m_bReadCamera = true;
  m_bShouldRun = true;
  m_ReadThread = std::thread( &Reader::_ThreadFunc, this );
  return true;
}

void Reader::EnableAll() {
  m_bReadCamera = true;
  m_bReadEncoder = true;
  m_bReadIMU = true;
  m_bReadLIDAR = true;
  m_bReadPosys = true;
}

void Reader::DisableAll() {
  m_bReadCamera = false;
  m_bReadEncoder = false;
  m_bReadIMU = false;
  m_bReadLIDAR = false;
  m_bReadPosys = false;
}

void Reader::Enable(MessageType type) {
  switch (type) {
    case Msg_Type_Camera:
      m_bReadCamera = true;
      break;
    case Msg_Type_Encoder:
      m_bReadEncoder = true;
      break;
    case Msg_Type_IMU:
      m_bReadIMU = true;
      break;
    case Msg_Type_LIDAR:
      m_bReadLIDAR = true;
      break;
    case Msg_Type_Posys:
      m_bReadPosys = true;
      break;
    default:
      LOG(FATAL) << "Incorrect message type given to Reader::Enable";
  }
}

void Reader::Disable(MessageType type) {
  switch (type) {
    case Msg_Type_Camera:
      m_bReadCamera = false;
      break;
    case Msg_Type_Encoder:
      m_bReadEncoder = false;
      break;
    case Msg_Type_IMU:
      m_bReadIMU = false;
      break;
    case Msg_Type_LIDAR:
      m_bReadLIDAR = false;
      break;
    case Msg_Type_Posys:
      m_bReadPosys = false;
      break;
    default:
      LOG(FATAL) << "Incorrect message type given to Reader::Disable";
  }
}

bool Reader::IsEnabled(MessageType type) const {
  switch (type) {
    case Msg_Type_Camera:
      return m_bReadCamera;
    case Msg_Type_Encoder:
      return m_bReadEncoder;
    case Msg_Type_IMU:
      return m_bReadIMU;
    case Msg_Type_LIDAR:
      return m_bReadLIDAR;
    case Msg_Type_Posys:
      return m_bReadPosys;
    default:
      LOG(FATAL) << "Incorrect message type given to Reader::IsEnabled";
  }

  return false;
}
}  // end namespace hal
