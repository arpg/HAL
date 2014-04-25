#pragma once

#include <atomic>
#include <fstream>
#include <list>
#include <mutex>
#include <sstream>
#include <thread>
#include <condition_variable>
#include <PbMsgs/Header.pb.h>
#include <PbMsgs/Messages.pb.h>

namespace pb {

class Logger {
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
  size_t messages_written() const;

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
  std::atomic<size_t>         m_nMessagesWritten;
};

} /* namespace */
