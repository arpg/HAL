#ifndef LOGGER_H
#define LOGGER_H

#include <thread>
#include <list>
#include <mutex>
#include <fstream>
#include <sstream>
#include <condition_variable>
#include "Messages/Messages.pb.h"

class Logger
{
public:
    static Logger *GetInstance();
    ~Logger();

    std::string OpenNewLogFile(const std::string &sLogDir, const std::string &sPrefix);
    void OpenLogFile(const std::string &fileName);
    void CloseLogFile();
    void ThreadFunc();
    void LogMessage(const pb::Msg& message);

private:
    Logger();
    std::thread m_WriteThread;
    std::list<pb::Msg> m_qMessages;
    std::mutex m_QueueMutex;
    std::condition_variable m_QueueCondition;
    std::ofstream m_File;
    bool m_bClosing;
};

#endif // LOGGER_H
