
#include "ThreadedObject.h"
/*
Self-contained threaded object built around pthreads
Originally developed at Astrobotic (https://www.astrobotic.com). Used with permission
 */


void *redirectRun(void *m)
{
  ThreadedObject *me=(ThreadedObject *)m;
  me->run();
  return NULL;
}

ThreadedObject::ThreadedObject()
{
  pthread_mutex_init(&cmdMutex, NULL);
  pthread_mutex_init(&dataMutex, NULL);
  pthread_cond_init(&dataPresent, NULL);
}

ThreadedObject::~ThreadedObject()
{
  pthread_mutex_destroy(&dataMutex);
  pthread_mutex_destroy(&cmdMutex);
  pthread_cond_destroy(&dataPresent);
}

void ThreadedObject::join()
{
  void *ret;
  pthread_join(myThread,&ret);
}

void ThreadedObject::cancel()
{
  pthread_cancel(myThread);
}

pthread_t ThreadedObject::start()
{
  if (pthread_create( &myThread, NULL,redirectRun,(void *)(this)))
    cerr<<"ThreadedObject::start could not start thread"<<endl;
  return myThread;
}

void ThreadedObject::PutCommand(void* command)
{
  pthread_mutex_lock(&cmdMutex);
  cmdQueue.push_back(command);
  pthread_mutex_unlock(&cmdMutex);
}

/* Nonblocking option to check for data */
/* DOES NOT RESPECT MULTIPLE CLIENTS */

void* ThreadedObject::GetData()
{
  void* retVal;

  //cout << "Attempting to get data" << endl;
  pthread_mutex_lock(&dataMutex);
  if (dataQueue.empty())
    retVal = NULL;
  else
    {
      retVal = dataQueue.front();
      dataQueue.pop_front();
    }
  pthread_mutex_unlock(&dataMutex);
  return retVal;
}

/* The condition variable is set by the data production facility */
/* Copy the item into the provided void* */
void* ThreadedObject::WaitData()
{ 
  void *item;
  pthread_mutex_lock(&dataMutex);

  //If the queue is empty, wait for something to appear
  if (dataQueue.size() == 0)
    pthread_cond_wait(&dataPresent, &dataMutex);
  
  item = dataQueue.front();
  dataQueue.pop_front();
  pthread_mutex_unlock(&dataMutex);
  return item;
}

void* ThreadedObject::GetCommand()
{
  void* retVal;
  pthread_mutex_lock(&cmdMutex);
  if (cmdQueue.empty())
    retVal = NULL;
  else
    {
      retVal = cmdQueue.front();
      cmdQueue.pop_front();
    }
  pthread_mutex_unlock(&cmdMutex);
  return retVal;
}

void ThreadedObject::PutData(void* data)
{
  pthread_mutex_lock(&dataMutex);

  dataQueue.push_back(data);  
  pthread_cond_signal(&dataPresent);
 
  pthread_mutex_unlock(&dataMutex);
}

// Get the time (in ms)
//
int64_t ThreadedObject::GetTime()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);

  return (int64_t) tv.tv_sec * 1000 + (int64_t) tv.tv_usec / 1000;
}
