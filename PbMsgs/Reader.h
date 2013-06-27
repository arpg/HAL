#pragma once

#include <fstream>

#include <thread>
#include <mutex>
#include <condition_variable>

#include <list>
#include <memory>

#include <PbMsgs/Messages.pb.h>

namespace pb
{

enum MessageType {
    Msg_Type_Camera,
    Msg_Type_IMU
};

class Reader
{
public:
    static Reader* Instance(const std::string& filename);

    ~Reader();

    /// Reads message regardless of type. This allows the user to handle the message list directly.
    /// This will block if no messages are in the queue.
    std::unique_ptr<pb::Msg> ReadMessage();

    /// Reads the next camera message from the message queue. If the next message is NOT a camera message,
    /// or the message queue is empty, the function will block. Mostly used for camera specific driver implementations.
    /// The "ReadCamera" static variable must be set to true if the reader is to queue camera messages.
    std::unique_ptr<pb::CameraMsg> ReadCameraMsg();

    /// Reads the next IMU message from the message queue. If the next message is NOT an IMU message,
    /// or the message queue is empty, the function will block. Mostly used for IMU specific driver implementations.
    /// The "ReadIMU" static variable must be set to true if the reader is to queue IMU messages.
    std::unique_ptr<pb::ImuMsg> ReadImuMsg();

    /// Stops the buffering thread. Should be called by driver implementations, usually in their destructors.
    void StopBuffering();

    /// Getters and setters for max buffer size
    void SetMaxBufferSize(const int nNumMessages) { m_nMaxBufferSize = nNumMessages; }
    size_t GetMaxBufferSize() const { return m_nMaxBufferSize; }
private:
    Reader(const std::string& filename);
    bool _AmINext( MessageType eMsgType );
    bool _BufferFromFile(const std::string &fileName);

    void _ThreadFunc();

public:
    static bool                             ReadCamera;
    static bool                             ReadIMU;

private:
    static Reader*                          m_pInstance;
    std::string                             m_sFilename;
    bool                                    m_bRunning;
    bool                                    m_bShouldRun;
    std::list<std::unique_ptr<pb::Msg> >    m_qMessages;
    std::list<MessageType >                 m_qMessageTypes;
    std::mutex                              m_QueueMutex;
    std::condition_variable                 m_ConditionQueued;
    std::condition_variable                 m_ConditionDequeued;
    std::thread                             m_WriteThread;
    size_t                                  m_nMaxBufferSize;
};

} /* namespace */
