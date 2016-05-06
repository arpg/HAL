#include "ThreadedCommand.h"

/* Generic command for use with the ThreadedObject
   Designed to allow a device to have a response to the command 
   Originally developed at Astrobotic (https://www.astrobotic.com). Used with permission
*/

ThreadedCommand::ThreadedCommand(ThreadedObject* m_parent)
{
  parent = m_parent;
  pthread_cond_init(&responseValid, NULL);
  //printf("Created cmdMutex 0x%x\n", (void*)&cmdMutex);
  pthread_mutex_init(&cmdMutex, NULL);
}

ThreadedCommand::~ThreadedCommand()
{
  //printf("Destroying mutex 0x%x\n", &cmdMutex);
  pthread_cond_destroy(&responseValid);
  pthread_mutex_destroy(&cmdMutex);
}

void ThreadedCommand::submit()
{
  pthread_mutex_lock(&cmdMutex); //avoid a deadlock where the command is acted on before we even finish submitting it
  //printf("Locked cmdMutex 0x%x\n", (void*)&cmdMutex);
  parent->PutCommand(this);
  //printf("Waiting on signal 0x%x\n", (void*)&cmdMutex);
  pthread_cond_wait(&responseValid, &cmdMutex);

  pthread_mutex_unlock(&cmdMutex);
  //printf("Unlocked cmdMutex 0x%x\n", (void*)&cmdMutex);
}

/* Called from context of device service thread to indicate command handled */
void ThreadedCommand::signal()
{
  //printf("Waiting on cmdMutex 0x%x\n", (void*)&cmdMutex);
  pthread_mutex_lock(&cmdMutex);
  //printf("Signalling on cmdMutex 0x%x\n", (void*)&cmdMutex);
  pthread_cond_signal(&responseValid);
  pthread_mutex_unlock(&cmdMutex);
}

void ThreadedCommand::wait()
{
  pthread_mutex_lock(&cmdMutex);
  pthread_cond_wait(&responseValid, &cmdMutex);
  pthread_mutex_unlock(&cmdMutex);
}
