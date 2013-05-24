#include <iostream>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include "Logger.h"
#include <functional>

#include <google/protobuf/io/zero_copy_stream_impl.h>

namespace pb
{

/////////////////////////////////////////////////////////////////////////////////////////
Logger& Logger::GetInstance()
{
    static Logger s_instance;
    return s_instance;
}

/////////////////////////////////////////////////////////////////////////////////////////
Logger::Logger() :
    m_sFilename("proto.log"),
    m_bShouldRun(false)
{
}

/////////////////////////////////////////////////////////////////////////////////////////
Logger::~Logger()
{
    StopLogging();
}

/////////////////////////////////////////////////////////////////////////////////////////
void Logger::ThreadFunc()
{
    int fd = open(m_sFilename.c_str(), O_RDWR | O_CREAT | O_TRUNC);
    
    if(fd==0) {
        std::cout << "Error opening file " << m_sFilename << std::endl;
        return;
    }
    
    google::protobuf::io::FileOutputStream raw_output(fd);
    raw_output.SetCloseOnDelete(true);
    google::protobuf::io::CodedOutputStream coded_output(&raw_output);
    
    const int magic_number = 1234;
    coded_output.WriteLittleEndian32(magic_number);
    
    std::unique_lock<std::mutex> lock(m_QueueMutex);

    // wait for first message
    while(m_bShouldRun && m_qMessages.size() == 0) {
        m_QueueCondition.wait(lock);
    }
    
    int frames = 0;
    while(m_bShouldRun){
        pb::Msg& msg = m_qMessages.front();
        
        const size_t size_bytes = msg.ByteSize();
        coded_output.WriteVarint32( size_bytes );
        
        if(!msg.SerializeToCodedStream(&coded_output)) {
            std::cout << "failed to serialize to coded stream" << std::endl;
        }
        m_qMessages.pop_front();
        frames++;
        
        while(m_bShouldRun && m_qMessages.size() == 0) {
            m_QueueCondition.wait(lock);
        }
        
    }
    
    std::cout << "-ThreadFunc(). Wrote " << frames << " frames." << std::endl;
}

/////////////////////////////////////////////////////////////////////////////////////////
void Logger::LogMessage(const pb::Msg &message)
{
    if(message.has_timestamp() == false){
        std::cout << "Attempted to log a message without a timestamp.";
    }
    if(!m_WriteThread.joinable()) {
        LogToFile(m_sFilename);
    }
    
    std::lock_guard<std::mutex> lock(m_QueueMutex);
    m_qMessages.push_back(message);
    m_QueueCondition.notify_one();
}

/////////////////////////////////////////////////////////////////////////////////////////
void Logger::LogToFile(const std::string& filename)
{
    StopLogging();
    
    m_sFilename = filename;
    m_bShouldRun = true;
    m_WriteThread = std::thread( &Logger::ThreadFunc, this );
}

/////////////////////////////////////////////////////////////////////////////////////////
std::string Logger::LogToFile(const std::string& sLogDir,
                                   const std::string& sPrefix )
{
    StopLogging();

    std::string sFileDir;
    int nCount = 0;
    while(1) {
        std::stringstream wss;
        wss << sLogDir << sPrefix << "_log" << nCount << ".log";
        std::ifstream ifile(wss.str());
        if(!ifile){
            sFileDir = wss.str();
            break;
        }
        nCount++;
    }

    LogToFile(sFileDir);
    return sFileDir;
}

/////////////////////////////////////////////////////////////////////////////////////////
void Logger::StopLogging()
{
    std::cout << "StopLogging()" << std::endl;
    
    if(m_WriteThread.joinable()) {
        std::cout << "StopLogging() join" << std::endl;
        m_bShouldRun = false;
        m_QueueCondition.notify_all();
        m_WriteThread.join();
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
bool Logger::IsLogging()
{
    return m_WriteThread.joinable();  
}

}
