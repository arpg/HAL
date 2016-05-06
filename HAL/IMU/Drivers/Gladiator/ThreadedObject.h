#ifndef __THREADEDOBJ_H_
#define __THREADEDOBJ_H_
/*
Self-contained threaded object built around pthreads
Originally developed at Astrobotic (https://www.astrobotic.com). Used with permission
 */

#include <iostream>
#include <deque>
#include <stdint.h>
#include <sys/time.h>
#include <pthread.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

using namespace std;

class ThreadedObject
{
 public: 
  ThreadedObject();
  virtual ~ThreadedObject();
  pthread_t start();
  void cancel();
  void join();
  void PutCommand(void* command);

  /*
    GetData() doesn't implement a means of making sure that a 
    particular client got a piece of data
    If you expect to use multiple clients, don't use GetData()
    A correct implementation would use some client ID bitmap, e.g.
    similar to an FD_SET where in O(1) time one could check.
    However, that still wouldn't solve the problem of a slow client
    backing up the flow for others. 
  */

  void* GetData();          //polling, nonblocking

  void* WaitData();      //sleeps on a broadcast condvar if necessary


 protected:
  virtual void run()=0;
  void* GetCommand();
  void PutData(void* data);
  int64_t GetTime();
 private:
  deque<void*> cmdQueue;
  deque<void*> dataQueue;
  pthread_mutex_t cmdMutex;
  pthread_mutex_t dataMutex;
  pthread_cond_t  dataPresent;
  pthread_t myThread;

  friend void *redirectRun(void *);

};

#endif
