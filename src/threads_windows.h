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

const HANDLE INVALID_THREAD_HANDLE = NULL;

// Returns unique identifier of the calling thread
#define THREAD_ID GetCurrentThreadId()

// Controls the thread opening mode. THREAD_JOINABLE if you want the thread to only end and free its resources
// when calling Thread_WaitExit on it. THREAD_DETACHED if you want it to do that by itself.
enum { THREAD_DETACHED, THREAD_JOINABLE };

// Aliases for platform abstraction
typedef HANDLE Thread_Handle;

// Number of running threads
static size_t threadsNumber = 0;

// Setup new thread to run the given method asyncronously
Thread_Handle Thread_Start( void* (*function)( void* ), void* args, int mode )
{
  static HANDLE handle;
  static unsigned int thread_id;
  
  if( (handle = (HANDLE) _beginthreadex( NULL, 0, (unsigned int (__stdcall *) (void*)) function, args, 0, &thread_id )) == INVALID_HANDLE )
  {
    ERROR_PRINT( "_beginthreadex: failed creating new thread with function %p", function );
    return INVALID_HANDLE;
  }
  
  DEBUG_PRINT( "created thread %x successfully", handle );

  threadsNumber++;
  
  if( mode == THREAD_DETACHED ) CloseHandle( handle );

  return handle;
}

// Exit the calling thread, returning the given value
extern inline void Thread_Exit( uint32_t exitCode )
{
  threadsNumber--;

  DEBUG_PRINT( "thread %x exiting with code: %u", GetCurrentThreadId(), exitCode );

  _endthreadex( (DWORD) exitCode );
}

// Wait for the thread of the given manipulator to exit and return its exiting value
uint32_t Thread_WaitExit( Thread_Handle handle, unsigned int milliseconds )
{
  static DWORD exitCode = 0;
  static DWORD exitStatus = WAIT_OBJECT_0;

  if( handle != INVALID_THREAD_HANDLE )
  {
    DEBUG_PRINT( "waiting thread %x\n", handle );

    exitStatus = WaitForSingleObject( handle, (DWORD) milliseconds );
  
    if( exitStatus == WAIT_FAILED )
      ERROR_PRINT( "WaitForSingleObject: error waiting for thread %x end: code: %x\n", handle, GetLastError() );
    else if( exitStatus == WAIT_TIMEOUT )
      ERROR_PRINT( "WaitForSingleObject: wait for thread %x timed out\n", handle );
    else
    {
      DEBUG_PRINT( "thread %x returned\n", handle );
    
      if( GetExitCodeThread( handle, &exitCode ) != 0 )
        DEBUG_PRINT( "thread exit code: %u\n", exitCode ); 
    }
  
    if( CloseHandle( handle ) == 0 )
      ERROR_PRINT( "CloseHandle: error trying to close thread handle %x: code: %x", handle, GetLastError() );
  }

  return exitCode;
}

// Returns number of running threads (method for encapsulation purposes)
size_t Thread_GetActiveThreadsNumber()
{
  return threadsNumber;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                     THREAD LOCK (MUTEX)									                    /////
///////////////////////////////////////////////////////////////////////////////////////////////////////

typedef CRITICAL_SECTION* ThreadLock;

// Request new unique mutex for using in thread syncronization
ThreadLock ThreadLock_Create()
{
  CRITICAL_SECTION* lock = malloc( sizeof(CRITICAL_SECTION) );
  InitializeCriticalSection( lock );
  return lock;
}

extern inline void ThreadLock_Discard( CRITICAL_SECTION* lock )
{
  DeleteCriticalSection( lock );
  free( lock );
}

// Mutex aquisition and release
extern inline void ThreadLock_Aquire( CRITICAL_SECTION* lock ) { EnterCriticalSection( lock ); }
extern inline void ThreadLock_Release( CRITICAL_SECTION* lock ) { LeaveCriticalSection( lock ); }


///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                       THREAD SEMAPHORE                                      /////
///////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct {
  HANDLE counter;
  size_t value;
}
Semaphore;

Semaphore* Semaphore_Create( size_t startCount, size_t maxCount )
{
  Semaphore* sem = (Semaphore*) malloc( sizeof(Semaphore) );
  sem->counter = CreateSemaphore( NULL, startCount, maxCount, NULL );
  sem->value = startCount;
  
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
