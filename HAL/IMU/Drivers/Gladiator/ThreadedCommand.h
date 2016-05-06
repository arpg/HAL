#ifndef __THREADEDCOMMAND_H_
#define __THREADEDCOMMAND_H_

#include "ThreadedObject.h"
/*
  Originally developed at Astrobotic (https://www.astrobotic.com). Used with permission
  Generic command to be used with the ThreadedObjects.
  Override and populate with device-specific fields
*/

class ThreadedCommand
{
 public:
  ThreadedCommand(ThreadedObject *m_parent);
  ~ThreadedCommand();
  void signal();
  void submit();
  void wait();
 private:
  pthread_mutex_t cmdMutex;
  pthread_cond_t responseValid;
  ThreadedObject *parent;
};

#endif
