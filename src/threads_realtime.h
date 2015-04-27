///////////////////////////////////////////////////////////////////////////////
///// Wrapper library for thread management (creation and syncronization) ///// 
///// using low level operating system native methods (Real-Time Version) /////
///////////////////////////////////////////////////////////////////////////////

#ifndef THREADS_H
#define THREADS_H

#include <utility.h>

#include <stdint.h>

#include "debug.h"


///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                      THREADS HANDLING 									                    /////
///////////////////////////////////////////////////////////////////////////////////////////////////////

const CmtThreadFunctionID INVALID_THREAD_HANDLE = -1;
#define INFINITE CMT_WAIT_FOREVER

// Returns unique identifier of the calling thread
#define THREAD_ID CmtGetCurrentThreadID()

// Controls the thread opening mode. JOINABLE if you want the thread to only end and free its resources
// when calling Thread_WaitExit on it. DETACHED if you want it to do that by itself.
enum { DETACHED, JOINABLE };

// Aliases for platform abstraction
typedef CmtThreadFunctionID Thread_Handle;

// Exclusive thread pool
CmtThreadPoolHandle threadPool = NULL;

// Setup new thread to run the given method asyncronously
Thread_Handle Thread_Start( void* (*function)( void* ), void* args, int mode )
{
  CmtThreadFunctionID threadID = -1;
  static int status;
  
  static char errorBuffer[ CMT_MAX_MESSAGE_BUF_SIZE ];
  
  if( threadPool == NULL ) CmtNewThreadPool( 50, &threadPool );
  
  CmtThreadFunctionID* ref_threadID = ( mode == JOINABLE ) ? &threadID : NULL; 
  
  if( (status = CmtScheduleThreadPoolFunction( threadPool, (int (CVICALLBACK *) (void*)) function, args, ref_threadID )) < 0 )
  {
    CmtGetErrorMessage( status, errorBuffer );
    ERROR_PRINT( "error starting new thread: %s", errorBuffer );
    return INVALID_THREAD_HANDLE;
  }
  
  DEBUG_PRINT( "created thread %x successfully", ( mode == JOINABLE ) ? threadID : 0 );

  return threadID;
}

static int CVICALLBACK ClearThreadPool( void* data )
{
  if( threadPool != NULL )
  {
    CmtDiscardThreadPool( threadPool );
    threadPool = NULL;
  }
  
  return 0;
}

// Exit the calling thread, returning the given value
void Thread_Exit( uint32_t exitCode )
{
  DEBUG_PRINT( "thread exiting with code: %u", exitCode );

  CmtScheduleThreadPoolFunction( DEFAULT_THREAD_POOL_HANDLE, ClearThreadPool, NULL, NULL );
  
  CmtExitThreadPoolThread( (int) exitCode );
}

// Wait for the thread of the given manipulator to exit and return its exiting value
uint32_t Thread_WaitExit( Thread_Handle handle, unsigned int milliseconds )
{
  static uint32_t exitCode = 0;
  static int exitStatus = 0;
  
  static char errorBuffer[ CMT_MAX_MESSAGE_BUF_SIZE ]; 

  if( handle != INVALID_THREAD_HANDLE )
  {
    DEBUG_PRINT( "waiting thread %x", handle );

    if( (exitStatus = CmtWaitForThreadPoolFunctionCompletionEx( threadPool, handle, 0, milliseconds )) < 0 )
    {
      CmtGetErrorMessage( exitStatus, errorBuffer );
  	  ERROR_PRINT( "error waiting for thread end: %s", errorBuffer );
    }
    else
    {
      DEBUG_PRINT( "thread %x returned !", handle );

      if( (exitStatus = CmtGetThreadPoolFunctionAttribute( threadPool, handle, ATTR_TP_FUNCTION_RETURN_VALUE , (void*) &exitCode )) < 0 )
      {
        CmtGetErrorMessage( exitStatus, errorBuffer );
        ERROR_PRINT( "error getting function return value: %s:", errorBuffer );
      }
    }
  
    CmtReleaseThreadPoolFunctionID( threadPool, handle );
  }

  return exitCode;
}

// Returns number of running threads (method for encapsulation purposes)
size_t Thread_GetActiveThreadsNumber()
{
  static size_t threadsNumber;
  
  CmtGetThreadPoolAttribute( threadPool, ATTR_TP_NUM_ACTIVE_THREADS, &threadsNumber );
  
  return threadsNumber;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                     THREAD LOCK (MUTEX)									                    /////
///////////////////////////////////////////////////////////////////////////////////////////////////////

typedef CmtThreadLockHandle ThreadLock;

// Request new unique mutex for using in thread syncronization
ThreadLock ThreadLock_Create()
{
  CmtThreadLockHandle new_lock;
  CmtNewLock( NULL, 0, &new_lock );
  return new_lock;
}

// Properly discard mutex variable
extern inline void ThreadLock_Discard( CmtThreadLockHandle lock ) { CmtDiscardLock( lock ); }

// Mutex aquisition and release
extern inline void ThreadLock_Aquire( CmtThreadLockHandle lock ) { CmtGetLock( lock ); }
extern inline void ThreadLock_Release( CmtThreadLockHandle lock ) { CmtReleaseLock( lock ); }


///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                       THREAD SEMAPHORE      						                    /////
///////////////////////////////////////////////////////////////////////////////////////////////////////

typedef CmtTSQHandle Semaphore;

extern inline void Semaphore_Increment( Semaphore* );

Semaphore* Semaphore_Create( size_t startCount, size_t maxCount )
{
  CmtTSQHandle* sem = (CmtTSQHandle*) malloc( sizeof(CmtTSQHandle) );
  
  CmtNewTSQ( (int) maxCount, sizeof(uint8_t), 0, sem );
  
  for( size_t i = 0; i < startCount; i++ )
    Semaphore_Increment( sem );
  
  return sem;
}

extern inline void Semaphore_Discard( Semaphore* sem )
{
  CmtDiscardTSQ( *sem );
}

extern inline void Semaphore_Increment( Semaphore* sem )
{
  static uint8_t signal = 1;
  
  CmtWriteTSQData( *sem, &signal, 1, TSQ_INFINITE_TIMEOUT, NULL );  
}

extern inline void Semaphore_Decrement( Semaphore* sem )
{
  static uint8_t signal;
  
  CmtReadTSQData( *sem, &signal, 1, TSQ_INFINITE_TIMEOUT, 0 );
}

#endif /* THREADS_H */


#ifndef THREADS_DATA_QUEUE_H
#define THREADS_DATA_QUEUE_H

///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                     THREAD SAFE QUEUE                                       /////
///////////////////////////////////////////////////////////////////////////////////////////////////////

typedef CmtTSQHandle DataQueue;

DataQueue* DataQueue_Init( size_t maxLength, size_t itemSize )
{
  DataQueue* queue = (DataQueue*) malloc( sizeof(DataQueue) );
  
  if( CmtNewTSQ( (int) maxLength, itemSize, 0, queue ) < 0 )
    return NULL;
  
  return queue;
}

extern inline void DataQueue_End( DataQueue* queue )
{
  if( queue != NULL )
    CmtDiscardTSQ( *queue );
}

extern inline size_t DataQueue_GetItemsCount( DataQueue* queue )
{
  static size_t count;
  
  if( queue != NULL )
  {
    if( CmtGetTSQAttribute( *queue, ATTR_TSQ_ITEMS_IN_QUEUE, (void*) &count ) == 0 )
      return count;
  }

  return 0;
}

enum QueueReadMode { QUEUE_READ_WAIT = TSQ_INFINITE_TIMEOUT, QUEUE_READ_NOWAIT = 0 };
extern inline int DataQueue_Pop( DataQueue* queue, void* buffer, enum QueueReadMode mode )
{
  if( queue == NULL ) return -1;
  
  return CmtReadTSQData( *queue, buffer, 1, (int) mode, 0 );
}

enum QueueWriteMode { QUEUE_APPEND_WAIT = 0, QUEUE_APPEND_OVERWRITE = OPT_TSQ_AUTO_FLUSH_EXACT, QUEUE_FLUSH = OPT_TSQ_AUTO_FLUSH_ALL };
extern inline int DataQueue_Push( DataQueue* queue, void* buffer, enum QueueWriteMode mode )
{
  if( queue == NULL ) return -1;
  
  if( CmtSetTSQAttribute( *queue, ATTR_TSQ_QUEUE_OPTIONS, (int) mode ) == 0 )
    return -1;
  
  return CmtWriteTSQData( *queue, buffer, 1, TSQ_INFINITE_TIMEOUT, NULL );
}

#endif /* THREADS_DATA_QUEUE_H */ 
