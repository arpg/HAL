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
    m_nMaxBufferSize(100)
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
    google::protobuf::io::CodedInputStream coded_input(&raw_input);
    raw_input.SetCloseOnDelete(true);

    raw_input.Skip(100);

    
    uint32_t magic_number = 0;
    coded_input.ReadLittleEndian32(&magic_number);
    if (magic_number != 1234) {
      std::cout << "File not in expected format." << std::endl;
      return;
    }
    
    std::cout << "ThreadFunc " << magic_number << std::endl;
    m_bRunning = true;
    
    while(m_bShouldRun ){

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
            std::cout << "Failed to parse msg_size_bytes" << std::endl;
//            return;            
        }
        raw_input.Skip(4);
        std::cout << "size: " << msg_size_bytes << std::endl;
        
        
//        if( !pMsg->ParseFromCodedStream(&coded_input) ) {
//        }
        
        if( !pMsg->ParseFromBoundedZeroCopyStream(&raw_input, msg_size_bytes) ) {
            std::cout << "Failed to parse pMsg" << std::endl;
//            return;
        }
        std::cout << "pMsg: has_camera: " << pMsg->has_camera() << std::endl;

        std::lock_guard<std::mutex> lock(m_QueueMutex);
        m_qMessages.push_back(std::move(pMsg));
        m_QueueCondition.notify_one();
    }
    
    std::cout << "exit ThreadFunc" << std::endl;    
    
    m_bRunning = false;    
}

/////////////////////////////////////////////////////////////////////////////////////////
std::unique_ptr<pb::Msg> Reader::ReadMessage()
{
    std::unique_lock<std::mutex> lock(m_QueueMutex);
    while(m_qMessages.size() == 0 ){
        m_QueueCondition.wait(lock);
    }
    std::cout << "----------------------------------------------" << std::endl;
    std::cout << m_qMessages.size() << std::endl;
    std::cout << "has_camera: " << m_qMessages.front()->has_camera() << std::endl;
    std::unique_ptr<pb::Msg> pMessage = std::move(m_qMessages.front());
    std::cout << "has_camera: " << pMessage->has_camera() << std::endl;
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
