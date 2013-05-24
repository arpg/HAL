#include "Reader.h"
#include <unistd.h>

#include <google/protobuf/io/zero_copy_stream_impl.h>
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
    m_bRunning(false),
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
    
    int frames = 0;
    while(m_bShouldRun ){
        google::protobuf::io::CodedInputStream coded_input(&raw_input);

        int nCurrentSize;
        {
            std::lock_guard<std::mutex> lock(m_QueueMutex);
            nCurrentSize = m_qMessages.size();
        }

        if(nCurrentSize >= m_nMaxBufferSize){
            usleep(100);
            continue;
        }

        std::unique_ptr<pb::Msg> pMsg(new pb::Msg);
        
        uint32_t msg_size_bytes;
        if( !coded_input.ReadVarint32(&msg_size_bytes) ) {
            std::cerr << "Failed to parse msg_size_bytes" << std::endl;
            break;    
        }
        
        google::protobuf::io::CodedInputStream::Limit lim =
                coded_input.PushLimit(msg_size_bytes);        
        if( !pMsg->ParseFromCodedStream(&coded_input) ) {
            std::cerr << "Failed to parse pMsg #" << frames << std::endl;
            break;
        }
        coded_input.PopLimit(lim);
        frames++;

        std::lock_guard<std::mutex> lock(m_QueueMutex);
        m_qMessages.push_back(std::move(pMsg));
        m_QueueCondition.notify_one();
    }
    
    m_bRunning = false;    
}

/////////////////////////////////////////////////////////////////////////////////////////
std::unique_ptr<pb::Msg> Reader::ReadMessage()
{
    std::unique_lock<std::mutex> lock(m_QueueMutex);
    while(m_qMessages.size() == 0 ){
        m_QueueCondition.wait(lock);
        if(!m_bShouldRun) return nullptr;
    }
    std::unique_ptr<pb::Msg> pMessage = std::move(m_qMessages.front());
    m_qMessages.pop_front();
    
    return pMessage;
}

/////////////////////////////////////////////////////////////////////////////////////////
bool Reader::BufferFromFile(const std::string& fileName)
{
    StopBuffering();

    m_sFilename = fileName;
    m_bShouldRun = true;
    m_WriteThread = std::thread( &Reader::ThreadFunc, this );
    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////
void Reader::StopBuffering()
{
    if(m_bRunning) {
        m_bShouldRun = false;
        m_QueueCondition.notify_all();
        m_WriteThread.join();    
    }
    
}

}
