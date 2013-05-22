#include "Reader.h"
#include <unistd.h>
namespace pb
{

/////////////////////////////////////////////////////////////////////////////////////////
Reader& Reader::GetInstance()
{
    static Reader s_instance;
    return s_instance;
}

/////////////////////////////////////////////////////////////////////////////////////////
Reader::Reader() :
    m_bClosing(false),
    m_WriteThread( &Reader::ThreadFunc, this ),
    m_nMaxBufferSize(100)
{
}

/////////////////////////////////////////////////////////////////////////////////////////
Reader::~Reader()
{
    m_bClosing = true;
    m_QueueCondition.notify_all();

    m_WriteThread.join();
    CloseLogFile();
}

/////////////////////////////////////////////////////////////////////////////////////////
void Reader::ThreadFunc()
{
    while(!m_bClosing){
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
        pMsg->ParseFromIstream(&m_File);

        std::lock_guard<std::mutex> lock(m_QueueMutex);
        m_qMessages.push_back(std::move(pMsg));
        m_QueueCondition.notify_one();
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
void Reader::ReadMessage(std::unique_ptr<pb::Msg> pMessage)
{
    std::unique_lock<std::mutex> lock(m_QueueMutex);
    while(m_qMessages.size() == 0 ){
        m_QueueCondition.wait(lock);
    }

    pMessage = std::move(m_qMessages.front());
    m_qMessages.pop_front();

//    if(message.has_timestamp() == false){
//        std::cout << "Attempted to log a message without a timestamp.";
//    }
//    std::lock_guard<std::mutex> lock(m_QueueMutex);
//    m_qMessages.push_front(message);
//    m_QueueCondition.notify_one();
}

/////////////////////////////////////////////////////////////////////////////////////////
void Reader::OpenLogFile(const std::string& fileName)
{
    CloseLogFile();
    m_File.open(fileName);
}

/////////////////////////////////////////////////////////////////////////////////////////
void Reader::CloseLogFile()
{
    if(m_File.is_open()){
        m_File.close();
    }
}

}
