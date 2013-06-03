#ifndef READER_H
#define READER_H

#include <fstream>

#include <thread>
#include <mutex>
#include <condition_variable>

#include <list>
#include <memory>

#include <PbMsgs/Messages.pb.h>

namespace pb
{
class Reader
{
public:
    static Reader& GetInstance();

    Reader(const std::string& filename);
    ~Reader();

    bool BufferFromFile(const std::string &fileName);
    void StopBuffering();
    std::unique_ptr<pb::Msg> ReadMessage();

    /// Getters and setters for max buffer size
    void SetMaxBufferSize(const int nNumMessages) { m_nMaxBufferSize = nNumMessages; }
    size_t& GetMaxBufferSize() { return m_nMaxBufferSize; }
    size_t GetMaxBufferSize() const { return m_nMaxBufferSize; }

private:
    void ThreadFunc();

    std::string m_sFilename;
    bool m_bRunning;
    bool m_bShouldRun;
    std::list<std::unique_ptr<pb::Msg> > m_qMessages;
    std::mutex m_QueueMutex;
    std::condition_variable m_ConditionQueued;
    std::condition_variable m_ConditionDequeued;
    std::thread m_WriteThread;
    size_t m_nMaxBufferSize;
};
}

#endif // READER_H
