#pragma once

#include <atomic>
#include <fstream>
#include <list>
#include <mutex>
#include <sstream>
#include <thread>
#include <condition_variable>
#include <HAL/Header.pb.h>
#include <HAL/Messages.pb.h>

namespace hal {

class Logger {
 public:
  static Logger& GetInstance();

  Logger();
  ~Logger();

  /** Create a new log in the given directory of the format sPrefix_<count>.
   *
   * <count> is increased by one for every new log file.
   */
  std::string LogToFile(const std::string &sLogDir, const std::string &sPrefix);

  /** Write a log to this specific file, overwriting any previous file. */
  void LogToFile(const std::string &fileName);
  void StopLogging();
  bool IsLogging();
  void SetMaxBufferSize( unsigned int nBufferSize );
  size_t buffer_size() const;
  size_t messages_written() const;

  bool LogMessage(const hal::Msg& message);

 private:
  void ThreadFunc();

 private:
  std::list<hal::Msg>          m_qMessages;
  std::mutex                  m_QueueMutex;
  std::condition_variable     m_QueueCondition;
  std::string                 m_sFilename;
  bool                        m_bShouldRun;
  unsigned int                m_nMaxBufferSize;
  std::thread                 m_WriteThread;
  std::atomic<size_t>         m_nMessagesWritten;
};

} /* namespace */
