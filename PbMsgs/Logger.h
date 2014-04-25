#pragma once

#include <thread>
#include <list>
#include <mutex>
#include <fstream>
#include <sstream>
#include <condition_variable>
#include <PbMsgs/Header.pb.h>
#include <PbMsgs/Messages.pb.h>

namespace pb
{

class Logger
{
public:
    static Logger& GetInstance();

    Logger();
    ~Logger();

    std::string LogToFile(const std::string &sLogDir, const std::string &sPrefix);
    void LogToFile(const std::string &fileName);
    void StopLogging();
    bool IsLogging();
    void SetMaxBufferSize( unsigned int nBufferSize );
    size_t buffer_size() const;

    bool LogMessage(const pb::Msg& message);

private:
    void ThreadFunc();

private:
    std::list<pb::Msg>          m_qMessages;
    std::mutex                  m_QueueMutex;
    std::condition_variable     m_QueueCondition;
    std::string                 m_sFilename;
    bool                        m_bShouldRun;
    unsigned int                m_nMaxBufferSize;
    std::thread                 m_WriteThread;
};

} /* namespace */
