///////////////////////////////////////////////////////////////////////////////
///// Wrapper library for thread management (creation and syncronization) ///// 
///// using low level operating system native methods (Windows Version)   /////
///////////////////////////////////////////////////////////////////////////////

#ifndef THREADS_H
#define THREADS_H

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
    ERROR_PRINT( "_beginthreadex: failed creating new thread with function %p", function );
    return 0;
  }
  
  DEBUG_PRINT( "created thread %x successfully", handle );

  n_threads++;
  
  if( mode == DETACHED ) CloseHandle( handle );

  return handle;
}

// Exit the calling thread, returning the given value
extern inline void thread_exit( uint32_t exit_code )
{
  n_threads--;

  DEBUG_PRINT( "thread %x exiting with code: %u", GetCurrentThreadId(), exit_code );

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
/////                                       THREAD SEMAPHORE                                      /////
///////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct {
  HANDLE counter;
  size_t value;
}
Semaphore;

Semaphore* Semaphore_Create( size_t start_count, size_t max_count )
{
  Semaphore* sem = (Semaphore*) malloc( sizeof(Semaphore) );
  sem->counter = CreateSemaphore( NULL, start_count, max_count, NULL );
  sem->value = start_count;
  
  return sem;
}

extern inline void Semaphore_Discard( Semaphore* sem )
{
  CloseHandle( sem->counter );
  free( sem );
}

extern inline void Semaphore_Increment( Semaphore* sem )
{
  ReleaseSemaphore( sem->counter, 1, &(sem->value) );
  sem->value++;  
}

extern inline void Semaphore_Decrement( Semaphore* sem )
{
  WaitForSingleObject( sem->counter, INFINITE );
  sem->value--;
}

extern inline size_t Semaphore_GetValue( Semaphore* sem )
{
  return sem->value;
}


#endif /* THREADS_H */ 
