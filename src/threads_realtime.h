///////////////////////////////////////////////////////////////////////////////
///// Wrapper library for thread management (creation and syncronization) ///// 
///// using low level operating system native methods (Real-Time Version) /////
///////////////////////////////////////////////////////////////////////////////

#ifndef THREADS_H
#define THREADS_H

#include <utility.h>

#define INFINITE CMT_WAIT_FOREVER

// Returns unique identifier of the calling thread
#define THREAD_ID CmtGetCurrentThreadID()

// Controls the thread opening mode. JOINABLE if you want the thread to only end and free its resources
// when calling wait_thread_end on it. DETACHED if you want it to do that by itself.
enum { DETACHED, JOINABLE };

// Aliases for platform abstraction
typedef CmtThreadFunctionID Thread_Handle;
typedef CmtThreadLockHandle Thread_Lock;

// List of created mutexes
static CmtThreadLockHandle* lock_list;
static size_t n_locks = 0;

// List of manipulators for the created (joinable) threads
static CmtThreadFunctionID* handle_list;
static size_t n_handles = 0;

// Number of running threads
static size_t n_threads = 0;

// Request new unique mutex for using in thread syncronization
Thread_Lock get_new_thread_lock()
{
  lock_list = (CmtThreadLockHandle*) realloc( lock_list, ( n_locks + 1 ) * sizeof(CmtThreadLockHandle) );
  CmtNewLock( NULL, 0, &lock_list[ n_locks ] );
  return lock_list[ n_locks++ ];
}

// Mutex aquisition and release
extern inline LOCK_THREAD( CmtThreadLockHandle lock ) { CmtGetLock( lock ); }
extern inline UNLOCK_THREAD( CmtThreadLockHandle lock ) { CmtReleaseLock( lock ); }

// Setup new thread to run the given method asyncronously
Thread_Handle run_thread( void* (*function)( void* ), void* args, int mode )
{
  static CmtThreadFunctionID thread_id;
  static int status;
  
  if( (status = CmtScheduleThreadPoolFunction( DEFAULT_THREAD_POOL_HANDLE, (int (CVICALLBACK *) (void*)) function, args, &thread_id )) < 0 )
  {
    fprintf( stderr, "%s: %s\n", __func__, CmtGetErrorMessage( status ) );
    return 0;
  }
  
  #ifdef DEBUG_2
  printf( "%s: created thread %x successfully\n", __func__, handle_list[ n_handles ] );
  #endif

  n_threads++;
  
  if( mode == JOINABLE )
  {
	handle_list = (HANDLE*) realloc( handle_list, ( n_handles + 1 ) * sizeof(HANDLE) );
	handle_list[ n_handles ] = thread_id;
	n_handles++;
  }
  else if( mode == DETACHED ) 
	CmtReleaseThreadPoolFunctionID( DEFAULT_THREAD_POOL_HANDLE, thread_id );

  return thread_id;
}

// Exit the calling thread, returning the given value
void exit_thread( uint32_t exit_code )
{
  n_threads--;

  #ifdef DEBUG_2
  printf( "%s: thread exiting with code: %u\n", __func__, exit_code );
  #endif

  CmtExitThreadPoolThread( (int) exit_code );
}

// Wait for the thread of the given manipulator to exit and return its exiting value
uint32_t wait_thread_end( Thread_Handle handle, unsigned int milisseconds )
{
  static uint32_t exit_code = 0;
  static int exit_status = 0;

  #ifdef DEBUG_1
  printf( "%s: waiting thread %x\n", __func__, handle );
  #endif

  if( (exit_status = CmtWaitForThreadPoolFunctionCompletionEx( DEFAULT_THREAD_POOL_HANDLE, handle, 0, timeout )) < 0 )
  {
	fprintf( stderr, "%s: %s\n", __func__, CmtGetErrorMessage( exit_status ) );
	return 0;
  }

  #ifdef DEBUG_1
  printf( "%s: thread %x returned !\n", __func__, handle );
  #endif

  if( CmtGetThreadPoolFunctionAttribute( DEFAULT_THREAD_POOL_HANDLE, handle, ATTR_TP_FUNCTION_RETURN_VALUE , (void*) &exit_code ) < 0 )
  {
    #ifdef DEBUG_1
    printf( "%s: %s\n", __func__, CmtGetErrorMessage( exit_status ) );
    #endif

    return exit_code;
  }
  else
    return 0;
}

// Returns number of running threads (method for encapsulation purposes)
size_t get_threads_number()
{
  return n_threads;
}

// Handle the destruction of the remaining threads and mutexes when the program quits
void end_threading()
{
  for( size_t lock_id = 0; lock_id < n_locks; lock_id++ )
    CmtDiscardLock( lock_list[ lock_id ] );
  
  for( thread_id = 0; thread_id < n_handles; thread_id++ )
    CmtReleaseThreadPoolFunctionID( DEFAULT_THREAD_POOL_HANDLE, thread_id );
  
  return;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                     THREAD SAFE QUEUE										  /////
///////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct Data_Queue CmtTSQHandle;

Data_Queue* data_queue_init( size_t max_length, size_t element_size )
{
  Data_Queue* queue = (Data_Queue*) malloc( sizeof(Data_Queue) );
  
  CmtNewTSQ( (int) max_length, element_size, OPT_TSQ_AUTO_FLUSH_EXACT, queue );
  
  return queue;
}

extern inline void data_queue_end( Data_queue* queue )
{
  CmtDiscardTSQ( *queue );
}

extern inline size_t data_queue_count( Data_Queue* queue )
{
  size_t count;
  
  CmtGetTSQAttribute( *queue, ATTR_TSQ_ITEMS_IN_QUEUE, (void*) &count );
  
  return count;
}

void* data_queue_read( Data_Queue* queue, void* buffer )
{
  if( data_queue_count( queue ) > 0 )
    CmtReadTSQData( *queue, buffer, 1, 0, 0 );
  
  return buffer;
}

extern inline void data_queue_write( Data_Queue* queue, void* buffer )
{
  CmtWriteTSQData( *queue, buffer, 1, TSQ_INFINITE_TIMEOUT, NULL );
}

#endif /* THREADS_H */ 
