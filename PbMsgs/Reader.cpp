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

/////////////////////////////////////////////////////////////////////////////////////////
Reader& Reader::GetInstance()
{
    static Reader s_instance("proto.log");
    return s_instance;
}

/////////////////////////////////////////////////////////////////////////////////////////
Reader::Reader(const std::string& filename) :
    m_bRunning(true),
    m_bShouldRun(false),
    m_nMaxBufferSize(10)
{
    BufferFromFile(filename);
}

/////////////////////////////////////////////////////////////////////////////////////////
Reader::~Reader()
{
    StopBuffering();
}

/////////////////////////////////////////////////////////////////////////////////////////
void Reader::ThreadFunc()
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
            std::cerr << "Failed to parse pMsg" << std::endl;
            break;
        }
        coded_input.PopLimit(lim);

        // Wait if buffer is full, then add to queue
        std::unique_lock<std::mutex> lock(m_QueueMutex);
        while(m_bShouldRun && m_qMessages.size() >= m_nMaxBufferSize){
            m_ConditionDequeued.wait_for(lock, std::chrono::milliseconds(10) );
        }        
        m_qMessages.push_back(std::move(pMsg));
        m_ConditionQueued.notify_one();
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
bool Reader::BufferFromFile(const std::string& fileName)
{
    m_sFilename = fileName;
    m_bShouldRun = true;
    m_WriteThread = std::thread( &Reader::ThreadFunc, this );
    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////
void Reader::StopBuffering()
{
    if(m_bRunning)
    {
        m_bShouldRun = false;
        m_ConditionQueued.notify_all();
    }
    m_WriteThread.join();    
}

}
