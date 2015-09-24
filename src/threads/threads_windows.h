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

#define NAMESPACE Threading

#define NAMESPACE_FUNCTIONS( FUNCTION_INIT, namespace ) \
        FUNCTION_INIT( Thread, namespace, StartThread, AsyncFunction, void*, int ) \
        FUNCTION_INIT( uint32_t, namespace, WaitExit, Thread, unsigned int )

NAMESPACE_FUNCTIONS( INIT_NAMESPACE_FILE, NAMESPACE )

const struct { NAMESPACE_FUNCTIONS( INIT_NAMESPACE_POINTER, NAMESPACE ) } NAMESPACE = { NAMESPACE_FUNCTIONS( INIT_NAMESPACE_STRUCT, NAMESPACE ) };

#undef NAMESPACE_FUNCTIONS
#undef NAMESPACE

// Setup new thread to run the given method asyncronously
Thread Threading_StartThread( AsyncFunction function, void* args, int mode )
{
  static HANDLE handle;
  static unsigned int threadID;
  
  if( (handle = CreateThread( NULL, 0, (LPTHREAD_START_ROUTINE) function, args, 0, &threadID )) == INVALID_THREAD_HANDLE )
  {
    ERROR_PRINT( "_beginthreadex: failed creating new thread with function %p", function );
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


///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                     THREAD LOCK (MUTEX)									                    /////
///////////////////////////////////////////////////////////////////////////////////////////////////////

typedef CRITICAL_SECTION* ThreadLock;

#define NAMESPACE ThreadLocks

#define NAMESPACE_FUNCTIONS( FUNCTION_INIT, namespace ) \
        FUNCTION_INIT( ThreadLock, namespace, Create, void ) \
        FUNCTION_INIT( void, namespace, Discard, ThreadLock ) \
        FUNCTION_INIT( void, namespace, Aquire, ThreadLock ) \
        FUNCTION_INIT( void, namespace, Release, ThreadLock )

NAMESPACE_FUNCTIONS( INIT_NAMESPACE_FILE, NAMESPACE )

const struct { NAMESPACE_FUNCTIONS( INIT_NAMESPACE_POINTER, NAMESPACE ) } NAMESPACE = { NAMESPACE_FUNCTIONS( INIT_NAMESPACE_STRUCT, NAMESPACE ) };

#undef NAMESPACE_FUNCTIONS
#undef NAMESPACE

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
  size_t value;
}
SemaphoreData;

typedef SemaphoreData* Semaphore;

#define NAMESPACE Semaphores

#define NAMESPACE_FUNCTIONS( FUNCTION_INIT, namespace ) \
        FUNCTION_INIT( Semaphore, namespace, Create, size_t, size_t ) \
        FUNCTION_INIT( void, namespace, Discard, Semaphore ) \
        FUNCTION_INIT( void, namespace, Increment, Semaphore ) \
        FUNCTION_INIT( void, namespace, Decrement, Semaphore ) \
        FUNCTION_INIT( size_t, namespace, GetValue, Semaphore )

NAMESPACE_FUNCTIONS( INIT_NAMESPACE_FILE, NAMESPACE )

const struct { NAMESPACE_FUNCTIONS( INIT_NAMESPACE_POINTER, NAMESPACE ) } NAMESPACE = { NAMESPACE_FUNCTIONS( INIT_NAMESPACE_STRUCT, NAMESPACE ) };

#undef NAMESPACE_FUNCTIONS
#undef NAMESPACE

inline Semaphore Semaphores_Create( size_t startCount, size_t maxCount )
{
  Semaphore sem = (Semaphore) malloc( sizeof(SemaphoreData) );
  sem->counter = CreateSemaphore( NULL, startCount, maxCount, NULL );
  sem->value = startCount;
  
  return sem;
}

inline void Semaphores_Discard( Semaphore sem )
{
  CloseHandle( sem->counter );
  free( sem );
}

inline void Semaphores_Increment( Semaphore sem )
{
  ReleaseSemaphore( sem->counter, 1, &(sem->value) );
  sem->value++;  
}

inline void Semaphores_Decrement( Semaphore sem )
{
  WaitForSingleObject( sem->counter, INFINITE );
  sem->value--;
}

inline size_t Semaphores_GetValue( Semaphore sem )
{
  return sem->value;
}


#endif /* THREADS_H */ 
