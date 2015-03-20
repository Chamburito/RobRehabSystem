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

#define INFINITE CMT_WAIT_FOREVER

// Returns unique identifier of the calling thread
#define THREAD_ID CmtGetCurrentThreadID()

// Controls the thread opening mode. JOINABLE if you want the thread to only end and free its resources
// when calling thread_wait_exit on it. DETACHED if you want it to do that by itself.
enum { DETACHED, JOINABLE };

// Aliases for platform abstraction
typedef CmtThreadFunctionID Thread_Handle;

// Number of running threads
static size_t n_threads = 0;

// Setup new thread to run the given method asyncronously
Thread_Handle thread_start( void* (*function)( void* ), void* args, int mode )
{
  static CmtThreadFunctionID thread_id;
  static int status;
  
  static char error_buffer[ CMT_MAX_MESSAGE_BUF_SIZE ];
  
  if( (status = CmtScheduleThreadPoolFunction( DEFAULT_THREAD_POOL_HANDLE, (int (CVICALLBACK *) (void*)) function, args, &thread_id )) < 0 )
  {
    CmtGetErrorMessage( status, error_buffer );
    ERROR_PRINT( "error starting new thread: %s\n", error_buffer );
    return 0;
  }
  
  DEBUG_PRINT( "created thread %x successfully\n", thread_id );

  n_threads++;
  
  if( mode == DETACHED ) 
	  CmtReleaseThreadPoolFunctionID( DEFAULT_THREAD_POOL_HANDLE, thread_id );

  return thread_id;
}

// Exit the calling thread, returning the given value
void thread_exit( uint32_t exit_code )
{
  n_threads--;

  DEBUG_PRINT( "thread exiting with code: %u\n", exit_code );

  CmtExitThreadPoolThread( (int) exit_code );
}

// Wait for the thread of the given manipulator to exit and return its exiting value
uint32_t thread_wait_exit( Thread_Handle handle, unsigned int milisseconds )
{
  static uint32_t exit_code = 0;
  static int exit_status = 0;
  
  static char error_buffer[ CMT_MAX_MESSAGE_BUF_SIZE ]; 

  DEBUG_PRINT( "waiting thread %x\n", handle );

  if( (exit_status = CmtWaitForThreadPoolFunctionCompletionEx( DEFAULT_THREAD_POOL_HANDLE, handle, 0, milisseconds )) < 0 )
  {
    CmtGetErrorMessage( exit_status, error_buffer );
	  ERROR_PRINT( "error waiting for thread end: %s\n", error_buffer );
  }
  else
  {
    DEBUG_PRINT( "thread %x returned !\n", handle );

    if( (exit_status = CmtGetThreadPoolFunctionAttribute( DEFAULT_THREAD_POOL_HANDLE, handle, ATTR_TP_FUNCTION_RETURN_VALUE , (void*) &exit_code )) < 0 )
    {
      CmtGetErrorMessage( exit_status, error_buffer );
      ERROR_PRINT( "error getting function return value: %s:\n", error_buffer );
    }
  }
  
  CmtReleaseThreadPoolFunctionID( DEFAULT_THREAD_POOL_HANDLE, handle );

  return exit_code;
}

// Returns number of running threads (method for encapsulation purposes)
size_t threads_get_active_count()
{
  return n_threads;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                     THREAD LOCK (MUTEX)									                    /////
///////////////////////////////////////////////////////////////////////////////////////////////////////

typedef CmtThreadLockHandle Thread_Lock;

// Request new unique mutex for using in thread syncronization
Thread_Lock thread_lock_create()
{
  CmtThreadLockHandle new_lock;
  CmtNewLock( NULL, 0, &new_lock );
  return new_lock;
}

// Properly discard mutex variable
extern inline void thread_lock_discard( CmtThreadLockHandle lock ) { CmtDiscardLock( lock ); }

// Mutex aquisition and release
extern inline void thread_lock_aquire( CmtThreadLockHandle lock ) { CmtGetLock( lock ); }
extern inline void thread_lock_release( CmtThreadLockHandle lock ) { CmtReleaseLock( lock ); }


///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                       THREAD SEMAPHORE      						                    /////
///////////////////////////////////////////////////////////////////////////////////////////////////////

typedef CmtTSQHandle Semaphore;

extern inline void semaphore_increment( Semaphore );

Semaphore semaphore_create( size_t start_count, size_t max_count )
{
  Semaphore sem;
  
  CmtNewTSQ( (int) max_count, sizeof(uint8_t), 0, &sem );
  
  for( size_t i = 0; i < start_count; i++ )
    semaphore_increment( sem );
  
  return sem;
}

extern inline void semaphore_discard( Semaphore sem )
{
  CmtDiscardTSQ( sem );
}

extern inline void semaphore_increment( Semaphore sem )
{
  static uint8_t signal = 1;
  
  CmtWriteTSQData( sem, &signal, 1, TSQ_INFINITE_TIMEOUT, NULL );  
}

extern inline void semaphore_decrement( Semaphore sem )
{
  static uint8_t signal;
  
  CmtReadTSQData( sem, &signal, 1, TSQ_INFINITE_TIMEOUT, 0 );
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                     THREAD SAFE QUEUE										                    /////
///////////////////////////////////////////////////////////////////////////////////////////////////////

typedef CmtTSQHandle Data_Queue;

Data_Queue* data_queue_init( size_t max_length, size_t element_size )
{
  Data_Queue* queue = (Data_Queue*) malloc( sizeof(Data_Queue) );
  
  CmtNewTSQ( (int) max_length, element_size, 0, queue );
  
  return queue;
}

extern inline void data_queue_end( Data_Queue* queue )
{
  CmtDiscardTSQ( *queue );
}

extern inline size_t data_queue_get_count( Data_Queue* queue )
{
  size_t count;
  
  CmtGetTSQAttribute( *queue, ATTR_TSQ_ITEMS_IN_QUEUE, (void*) &count );
  
  return count;
}

void* data_queue_read( Data_Queue* queue, void* buffer )
{
  CmtReadTSQData( *queue, buffer, 1, TSQ_INFINITE_TIMEOUT, 0 );
  
  return buffer;
}

extern inline void data_queue_write( Data_Queue* queue, void* buffer, uint8_t replace )
{
  if( replace == 1 ) CmtSetTSQAttribute( *queue, ATTR_TSQ_QUEUE_OPTIONS, OPT_TSQ_AUTO_FLUSH_EXACT ); // Replace old data behaviour
  else CmtSetTSQAttribute( *queue, ATTR_TSQ_QUEUE_OPTIONS, 0 );
  
  CmtWriteTSQData( *queue, buffer, 1, TSQ_INFINITE_TIMEOUT, NULL );
}

#endif /* THREADS_H */ 
