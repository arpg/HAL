#include "Reader.h"
#include <unistd.h>

#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/io/coded_stream.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <iostream>
#include <stdexcept>

namespace pb
{

Reader* Reader::m_pInstance = nullptr;

bool Reader::ReadCamera     = false;
bool Reader::ReadIMU        = false;

/////////////////////////////////////////////////////////////////////////////////////////
Reader* Reader::Instance( const std::string& filename )
{
    if(!m_pInstance) {
        m_pInstance = new Reader(filename);
    }
    return m_pInstance;
}

/////////////////////////////////////////////////////////////////////////////////////////
Reader::Reader(const std::string& filename) :
    m_bRunning(true),
    m_bShouldRun(false),
    m_nMaxBufferSize(10)
{
    _BufferFromFile(filename);
}

/////////////////////////////////////////////////////////////////////////////////////////
bool Reader::_AmINext(MessageType eMsgType)
{
    if( m_qMessageTypes.empty() ) {
        return false;
    }

    return m_qMessageTypes.front() == eMsgType;
}

/////////////////////////////////////////////////////////////////////////////////////////
Reader::~Reader()
{
    StopBuffering();
}

/////////////////////////////////////////////////////////////////////////////////////////
void Reader::_ThreadFunc()
{
    int fd = open(m_sFilename.c_str(), O_RDONLY);
    google::protobuf::io::FileInputStream raw_input(fd);
    raw_input.SetCloseOnDelete(true);

    uint32_t magic_number = 0;

    {
        google::protobuf::io::CodedInputStream coded_input(&raw_input);
        coded_input.ReadLittleEndian32(&magic_number);
    }

    if (magic_number != 1234) {
      std::cerr << "File not in expected format." << std::endl;
      return;
    }

    m_bRunning = true;

    while( m_bShouldRun ){
        google::protobuf::io::CodedInputStream coded_input(&raw_input);

        uint32_t msg_size_bytes;
        if( !coded_input.ReadVarint32(&msg_size_bytes) ) {
            // Probably end of stream.
            break;
        }

        google::protobuf::io::CodedInputStream::Limit lim =
                coded_input.PushLimit(msg_size_bytes);
        std::unique_ptr<pb::Msg> pMsg(new pb::Msg);
        if( !pMsg->ParseFromCodedStream(&coded_input) ) {
            break;
        }
        coded_input.PopLimit(lim);

        // Wait if buffer is full, then add to queue
        std::unique_lock<std::mutex> lock(m_QueueMutex);
        while(m_bShouldRun && m_qMessages.size() >= m_nMaxBufferSize){
            m_ConditionDequeued.wait_for(lock, std::chrono::milliseconds(10) );
        }
        if( (pMsg->has_camera() && ReadCamera)
            || (pMsg->has_imu() && ReadIMU) ) {

            if( pMsg->has_camera() ) {
                m_qMessageTypes.push_back( Msg_Type_Camera );
            }
            if( pMsg->has_imu() ) {
                m_qMessageTypes.push_back( Msg_Type_IMU );
            }

            m_qMessages.push_back(std::move(pMsg));

            m_ConditionQueued.notify_one();
        }
    }

    m_bRunning = false;
}

/////////////////////////////////////////////////////////////////////////////////////////
std::unique_ptr<pb::Msg> Reader::ReadMessage()
{
    // Wait if buffer is empty
    std::unique_lock<std::mutex> lock(m_QueueMutex);
    while(m_bRunning && m_qMessages.size() == 0 ){
        m_ConditionQueued.wait_for(lock, std::chrono::milliseconds(10));
    }

    if(m_qMessages.size()) {
        std::unique_ptr<pb::Msg> pMessage = std::move(m_qMessages.front());
        m_qMessages.pop_front();
        m_ConditionDequeued.notify_one();
        return pMessage;
    }else{
        return nullptr;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
std::unique_ptr<pb::CameraMsg> Reader::ReadCameraMsg()
{
    if( !ReadCamera ) {
        std::cerr << "warning: ReadCameraMsg was called but ReadCamera variable is set to false! " << std::endl;
        std::cerr << "This means the reader is NOT queueing camera messages. Please set Reader::ReadCamera to true if you wish to read camera messages." << std::endl;
        return nullptr;
    }

    // Wait if buffer is empty
    std::unique_lock<std::mutex> lock(m_QueueMutex);
    while(m_bRunning && !_AmINext( Msg_Type_Camera ) ){
        m_ConditionQueued.wait_for(lock, std::chrono::milliseconds(10));
    }

    if(!m_bRunning) {
        return nullptr;
    }

    std::unique_ptr<pb::Msg> pMessage = std::move(m_qMessages.front());
    m_qMessages.pop_front();
    m_qMessageTypes.pop_front();
    m_ConditionDequeued.notify_one();

    std::unique_ptr<pb::CameraMsg> pCameraMsg( new pb::CameraMsg );
    pCameraMsg->Swap( pMessage->mutable_camera() );
    return pCameraMsg;
}

/////////////////////////////////////////////////////////////////////////////////////////
std::unique_ptr<pb::ImuMsg> Reader::ReadImuMsg()
{
    if( !ReadIMU ) {
        std::cerr << "warning: ReadImuMsg was called but ReadIMU variable is set to false! " << std::endl;
        std::cerr << "This means the reader is NOT queueing IMU messages. Please set Reader::ReadIMU to true if you wish to read IMU messages." << std::endl;
        return nullptr;
    }

    // Wait if buffer is empty
    std::unique_lock<std::mutex> lock(m_QueueMutex);
    while(m_bRunning && !_AmINext( Msg_Type_IMU ) ){
        m_ConditionQueued.wait_for(lock, std::chrono::milliseconds(10));
    }

    if(!m_bRunning) {
        return nullptr;
    }

    std::unique_ptr<pb::Msg> pMessage = std::move(m_qMessages.front());
    m_qMessages.pop_front();
    m_qMessageTypes.pop_front();
    m_ConditionDequeued.notify_one();

    std::unique_ptr<pb::ImuMsg> pImuMsg( new pb::ImuMsg );
    pImuMsg->Swap( pMessage->mutable_imu() );
    return pImuMsg;
}

/////////////////////////////////////////////////////////////////////////////////////////
bool Reader::_BufferFromFile(const std::string& fileName)
{
    m_sFilename = fileName;
    m_bShouldRun = true;
    m_WriteThread = std::thread( &Reader::_ThreadFunc, this );
    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////
void Reader::StopBuffering()
{
    if(m_bRunning)
    {
        m_bShouldRun = false;
        m_ConditionDequeued.notify_one();
        m_WriteThread.join();
    }
    m_ConditionQueued.notify_all();
}

}
