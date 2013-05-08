#include "Logger.h"

Logger *g_pLoggerInstance = NULL;

/////////////////////////////////////////////////////////////////////////////////////////
Logger *Logger::GetInstance()
{
    if(g_pLoggerInstance == NULL){
        g_pLoggerInstance = new Logger();
    }
    return g_pLoggerInstance;
}

/////////////////////////////////////////////////////////////////////////////////////////
Logger::Logger() : m_bClosing(false)
{
}

/////////////////////////////////////////////////////////////////////////////////////////
Logger::~Logger()
{
    m_bClosing = true;
    m_QueueCondition.notify_all();

    m_WriteThread.join();
    CloseLogFile();
}

/////////////////////////////////////////////////////////////////////////////////////////
void Logger::ThreadFunc()
{
    std::unique_lock<std::mutex> lock(m_QueueMutex);
    while(1){
        //first wait for something to be added to the queue
        m_QueueCondition.wait(lock);

        while(m_qMessages.size() > 0 ){
            pb::Msg& msg = m_qMessages.back();
            int byteSize = msg.ByteSize();

            m_File << byteSize ; //write the size
            char* array = new char[byteSize];
            msg.SerializeToArray((void*)array, byteSize);
            m_File.write(array, byteSize);
            delete[] array;

            m_qMessages.pop_back();
        }

        if(m_bClosing) {
            break;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
std::string Logger::OpenNewLogFile(const std::string& sLogDir,
                                   const std::string& sPrefix /* = "" */ )
{
    CloseLogFile();

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

    OpenLogFile(sFileDir);
    return sFileDir;
}

/////////////////////////////////////////////////////////////////////////////////////////
void Logger::LogMessage(const pb::Msg &message)
{
    if(message.has_timestamp() == false){
        std::cout << "Attempted to log a message without a timestamp.";
    }
    std::lock_guard<std::mutex> lock(m_QueueMutex);
    m_qMessages.push_front(message);
    m_QueueCondition.notify_one();
}

/////////////////////////////////////////////////////////////////////////////////////////
void Logger::OpenLogFile(const std::string& fileName)
{
    CloseLogFile();
    m_File.open(fileName);
}

/////////////////////////////////////////////////////////////////////////////////////////
void Logger::CloseLogFile()
{
    if(m_File.is_open()){
        m_File.close();
    }
}
