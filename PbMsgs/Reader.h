#ifndef READER_H
#define READER_H

#include <thread>
#include <list>
#include <mutex>
#include <fstream>
#include <sstream>
#include <condition_variable>
#include <PbMsgs/Messages.pb.h>
#include <memory>

namespace pb
{
class Reader
{
public:
    static Reader& GetInstance();

    Reader();
    ~Reader();

    void OpenLogFile(const std::string &fileName);
    void CloseLogFile();
    void ThreadFunc();
    void ReadMessage(std::unique_ptr<pb::Msg> pMessage);

    /// Getters and setters for max buffer size
    void SetMaxBufferSize(const int nNumMessages) { m_nMaxBufferSize = nNumMessages; }
    int& GetMaxBufferSize() { return m_nMaxBufferSize; }
    int GetMaxBufferSize() const { return m_nMaxBufferSize; }

private:
    std::list<std::unique_ptr<pb::Msg> > m_qMessages;
    std::mutex m_QueueMutex;
    std::condition_variable m_QueueCondition;
    std::ifstream m_File;
    bool m_bClosing;
    std::thread m_WriteThread;
    int m_nMaxBufferSize;
};
}

#endif // READER_H
