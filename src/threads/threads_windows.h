///////////////////////////////////////////////////////////////////////////////
///// Wrapper library for thread management (creation and syncronization) ///// 
///// using low level operating system native methods (Windows Version)   /////
///////////////////////////////////////////////////////////////////////////////

#ifndef THREADS_H
#define THREADS_H

#include "debug/debug.h"
#include "interface.h"

#include <stdint.h>
#include <Windows.h>
//#include <process.h>
	
///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                      THREADS HANDLING 									                    /////
///////////////////////////////////////////////////////////////////////////////////////////////////////

const HANDLE INVALID_THREAD_HANDLE = NULL;

// Returns unique identifier of the calling thread
#define THREAD_ID GetCurrentThreadId()

// Controls the thread opening mode. THREAD_JOINABLE if you want the thread to only end and free its resources
// when calling Threading_WaitExit on it. THREAD_DETACHED if you want it to do that by itself.
enum { THREAD_DETACHED, THREAD_JOINABLE };

// Aliases for platform abstraction
typedef HANDLE Thread;

typedef void* (*AsyncFunction)( void* );

#define THREAD_FUNCTIONS( namespace, function_init ) \
        function_init( Thread, namespace, StartThread, AsyncFunction, void*, int ) \
        function_init( uint32_t, namespace, WaitExit, Thread, unsigned int )

INIT_NAMESPACE_INTERFACE( Threading, THREAD_FUNCTIONS )

// Setup new thread to run the given method asyncronously
Thread Threading_StartThread( AsyncFunction function, void* args, int mode )
{
  static HANDLE handle;
  static unsigned int threadID;
  
  if( (handle = CreateThread( NULL, 0, (LPTHREAD_START_ROUTINE) function, args, 0, &threadID )) == INVALID_THREAD_HANDLE )
  {
    ERROR_PRINT( "CreateThread: failed creating new thread with function %p", function );
    return INVALID_THREAD_HANDLE;
  }
  
  DEBUG_PRINT( "created thread %x successfully", handle );
  
  if( mode == THREAD_DETACHED ) CloseHandle( handle );

  return handle;
}

// Wait for the thread of the given manipulator to exit and return its exiting value
uint32_t Threading_WaitExit( Thread handle, unsigned int milliseconds )
{
  static DWORD exitCode = 0;
  static DWORD exitStatus = WAIT_OBJECT_0;

  if( handle != INVALID_THREAD_HANDLE )
  {
    DEBUG_PRINT( "waiting thread %x", handle );

    exitStatus = WaitForSingleObject( handle, (DWORD) milliseconds );
  
    if( exitStatus == WAIT_FAILED )
      ERROR_PRINT( "WaitForSingleObject: error waiting for thread %x end: code: %x", handle, GetLastError() );
    else if( exitStatus == WAIT_TIMEOUT )
      ERROR_PRINT( "WaitForSingleObject: wait for thread %x timed out", handle );
    else
    {
      DEBUG_PRINT( "thread %x returned", handle );
    
      if( GetExitCodeThread( handle, &exitCode ) != 0 )
        DEBUG_PRINT( "thread exit code: %u", exitCode ); 
    }
  
    if( CloseHandle( handle ) == 0 )
      ERROR_PRINT( "CloseHandle: error trying to close thread handle %x: code: %x", handle, GetLastError() );
  }

  return exitCode;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                     THREAD LOCK (MUTEX)									                    /////
///////////////////////////////////////////////////////////////////////////////////////////////////////

typedef CRITICAL_SECTION* ThreadLock;

#define THREAD_LOCK_FUNCTIONS( namespace, function_init ) \
        function_init( ThreadLock, namespace, Create, void ) \
        function_init( void, namespace, Discard, ThreadLock ) \
        function_init( void, namespace, Aquire, ThreadLock ) \
        function_init( void, namespace, Release, ThreadLock )

INIT_NAMESPACE_INTERFACE( ThreadLocks, THREAD_LOCK_FUNCTIONS )

// Request new unique mutex for using in thread syncronization
inline ThreadLock ThreadLocks_Create()
{
  CRITICAL_SECTION* lock = malloc( sizeof(CRITICAL_SECTION) );
  InitializeCriticalSection( lock );
  return lock;
}

inline void ThreadLocks_Discard( ThreadLock lock )
{
  DeleteCriticalSection( lock );
  free( lock );
}

// Mutex aquisition and release
inline void ThreadLocks_Aquire( ThreadLock lock ) { EnterCriticalSection( lock ); }
inline void ThreadLocks_Release( ThreadLock lock ) { LeaveCriticalSection( lock ); }


///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                       THREAD SEMAPHORE                                      /////
///////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct {
  HANDLE counter;
  size_t count, maxCount;
}
SemaphoreData;

typedef SemaphoreData* Semaphore;

#define SEMAPHORE_FUNCTIONS( namespace, function_init ) \
        function_init( Semaphore, namespace, Create, size_t, size_t ) \
        function_init( void, namespace, Discard, Semaphore ) \
        function_init( void, namespace, Increment, Semaphore ) \
        function_init( void, namespace, Decrement, Semaphore ) \
        function_init( size_t, namespace, GetCount, Semaphore ) \
        function_init( void, namespace, SetCount, Semaphore, size_t )

INIT_NAMESPACE_INTERFACE( Semaphores, SEMAPHORE_FUNCTIONS )

inline Semaphore Semaphores_Create( size_t startCount, size_t maxCount )
{
  Semaphore sem = (Semaphore) malloc( sizeof(SemaphoreData) );
  sem->counter = CreateSemaphore( NULL, startCount, maxCount, NULL );
  sem->count = startCount;
  sem->maxCount = maxCount;
  
  return sem;
}

inline void Semaphores_Discard( Semaphore sem )
{
  CloseHandle( sem->counter );
  free( sem );
}

inline void Semaphores_Increment( Semaphore sem )
{
  ReleaseSemaphore( sem->counter, 1, &(sem->count) );
  sem->count++;  
}

inline void Semaphores_Decrement( Semaphore sem )
{
  WaitForSingleObject( sem->counter, INFINITE );
  sem->count--;
}

inline size_t Semaphores_GetCount( Semaphore sem )
{
  if( sem == NULL ) return 0;
  
  return sem->count;
}

inline void Semaphores_SetCount( Semaphore sem, size_t count )
{
  if( sem == NULL ) return;
  
  if( count > sem->maxCount ) count = sem->maxCount;

  size_t countDelta = (size_t) abs( (int) count - (int) sem->count );
  if( count > sem->count )
  {
    for( size_t i = 0; i < countDelta; i++ )
      Semaphores_Increment( sem );
  }  
  else if( count < sem->count )
  {
    for( size_t i = 0; i < countDelta; i++ )
      Semaphores_Decrement( sem ); 
  }
}


#endif /* THREADS_H */ 
