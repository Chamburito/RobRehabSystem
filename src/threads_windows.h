///////////////////////////////////////////////////////////////////////////////
///// Wrapper library for thread management (creation and syncronization) ///// 
///// using low level operating system native methods (Windows Version)   /////
///////////////////////////////////////////////////////////////////////////////

#ifndef THREADS_H
#define THREADS_H

#ifdef __cplusplus
extern "C"{
#endif

#ifdef _CVI_
  #include <ansi_c.h>
#else
  #include <Windows.h>
  #include <process.h>
  #include <stdio.h>
  #include <stdint.h>
#endif

	
///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                      THREADS HANDLING 									                    /////
///////////////////////////////////////////////////////////////////////////////////////////////////////

// Returns unique identifier of the calling thread
#define THREAD_ID GetCurrentThreadId()

// Controls the thread opening mode. JOINABLE if you want the thread to only end and free its resources
// when calling thread_wait_exit on it. DETACHED if you want it to do that by itself.
enum { DETACHED, JOINABLE };

// Aliases for platform abstraction
typedef HANDLE Thread_Handle;

// Number of running threads
static size_t n_threads = 0;

// Setup new thread to run the given method asyncronously
Thread_Handle thread_start( void* (*function)( void* ), void* args, int mode )
{
  static HANDLE handle;
  static unsigned int thread_id;
  
  if( (handle = (HANDLE) _beginthreadex( NULL, 0, (unsigned int (__stdcall *) (void*)) function, args, 0, &thread_id )) == NULL )
  {
    ERROR_PRINT( "_beginthreadex: error creating new thread: code: %x\n", GetLastError() );
    return 0;
  }
  
  DEBUG_PRINT( "created thread %x successfully\n", handle );

  n_threads++;
  
  if( mode == DETACHED ) CloseHandle( handle );

  return handle;
}

// Exit the calling thread, returning the given value
extern inline void thread_exit( uint32_t exit_code )
{
  n_threads--;

  DEBUG_PRINT( "thread exiting with code: %u\n", exit_code );

  _endthreadex( (DWORD) exit_code );
}

// Wait for the thread of the given manipulator to exit and return its exiting value
uint32_t thread_wait_exit( Thread_Handle handle, unsigned int milisseconds )
{
  static DWORD exit_code = 0;
  static DWORD exit_status = WAIT_OBJECT_0;

  DEBUG_PRINT( "waiting thread %x\n", handle );

  exit_status = WaitForSingleObject( handle, (DWORD) milisseconds );
  
  if( exit_status == WAIT_FAILED )
    ERROR_PRINT( "WaitForSingleObject: error waiting for thread %x end: code: %x\n", handle, GetLastError() );
  else if( exit_status == WAIT_TIMEOUT )
    ERROR_PRINT( "WaitForSingleObject: wait for thread %x timed out\n", handle );
  else
  {
    DEBUG_PRINT( "thread %x returned\n", handle );
    
    if( GetExitCodeThread( handle, &exit_code ) != 0 )
      DEBUG_PRINT( "thread exit code: %u\n", exit_code ); 
  }
  
  if( CloseHandle( handle ) == 0 )
    ERROR_PRINT( "CloseHandle: error trying to close thread handle %x: code: %x", handle, GetLastError() );

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

typedef CRITICAL_SECTION* Thread_Lock;

// Request new unique mutex for using in thread syncronization
Thread_Lock thread_lock_create()
{
  CRITICAL_SECTION* lock = malloc( sizeof(CRITICAL_SECTION) );
  InitializeCriticalSection( lock );
  return lock;
}

extern inline void thread_lock_discard( CRITICAL_SECTION* lock )
{
  DeleteCriticalSection( lock );
  free( lock );
}

// Mutex aquisition and release
extern inline void thread_lock_aquire( CRITICAL_SECTION* lock ) { EnterCriticalSection( lock ); }
extern inline void thread_lock_release( CRITICAL_SECTION* lock ) { LeaveCriticalSection( lock ); }


///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                       THREAD SEMAPHORE      						                    /////
///////////////////////////////////////////////////////////////////////////////////////////////////////

typedef HANDLE Semaphore;

Semaphore semaphore_create( size_t start_count, size_t max_count )
{
  Semaphore sem = CreateSemaphore( NULL, start_count, max_count, NULL );
  
  return sem;
}

extern inline void semaphore_discard( Semaphore sem )
{
  CloseHandle( sem );
}

extern inline void semaphore_increment( Semaphore sem )
{
  ReleaseSemaphore( sem, 1, NULL );  
}

extern inline void semaphore_decrement( Semaphore* sem )
{
  WaitForSingleObject( sem, INFINITE );
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                     THREAD SAFE QUEUE										                    /////
///////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct _Data_Queue
{
  void** cache;
  size_t first, last, max_length;
  size_t element_size;
  Thread_Lock access_lock;
  Semaphore count_lock;
}
Data_Queue;

Data_Queue* data_queue_init( size_t max_length, size_t element_size )
{
  Data_Queue* queue = (Data_Queue*) malloc( sizeof(Data_Queue) );
  
  queue->max_length = max_length;
  queue->element_size = element_size;
  queue->cache = (void**) calloc( queue->max_length, sizeof(void*) );
  for( size_t i = 0; i < queue->max_length; i++ )
	queue->cache[ i ] = (void*) malloc( element_size );
  
  queue->first = queue->last = 0;
  
  queue->access_lock = thread_lock_create();
  queue->count_lock = semaphore_create( 0, queue->max_length );
  
  return queue;
}

void data_queue_end( Data_queue* queue )
{
  if( queue != NULL )
  {
  	for( size_t i = 0; i < queue->max_length; i++ )
  	  free( queue->cache[ i ] );
  	free( queue->cache );
    
    thread_lock_discard( queue->access_lock );
    semaphore_discard( queue->count_lock );
	
  	free( queue );
  	queue = NULL;
  }
}

extern inline size_t data_queue_count( Data_Queue* queue )
{
  return ( queue->last - queue->first );
}

void* data_queue_read( Data_Queue* queue, void* buffer )
{
  static void* data_out = NULL;
	
  semaphore_decrement( queue->count_lock );
  
  thread_lock_aquire( queue->access_lock );
  
  data_out = queue->cache[ queue->first % queue->max_length ]; // Always keep access index between 0 and MAX_DATA
  buffer = memcpy( buffer, data_out, queue->element_size );
  queue->first++;
  
  thread_lock_release( queue->access_lock );
  
  return buffer;
}

enum { WAIT = 0, REPLACE = 1 };

void data_queue_write( Data_Queue* queue, void* buffer, uint8_t replace )
{
  static void* data_in = NULL;
  
  if( replace == 0 ) semaphore_increment( queue->count_lock ); // Replace old data behaviour
  
  thread_lock_aquire( queue->access_lock );
  
  data_in = queue->cache[ queue->last % queue->max_length ]; // Always keep access index between 0 and MAX_DATA
  memcpy( data_in, buffer, queue->element_size );
  if( data_queue_count( queue ) == queue->max_length ) queue->first++;
  queue->last++;
  
  thread_lock_release( queue->access_lock );
}

#ifdef __cplusplus
}
#endif

#endif /* THREADS_H */ 
