///////////////////////////////////////////////////////////////////////////////
///// Wrapper library for thread management (creation and syncronization) ///// 
///// using low level operating system native methods (Windows Version)   /////
///////////////////////////////////////////////////////////////////////////////

#ifndef THREADS_H
#define THREADS_H

#ifdef __cplusplus
extern "C"{
#endif

#include <Windows.h>
#include <process.h>
#include <stdio.h>

// Mutex aquisition and release
#define LOCK_THREAD( lock ) TryEnterCriticalSection( lock )
#define UNLOCK_THREAD( lock ) LeaveCriticalSection( lock )

// Returns unique identifier of the calling thread
#define THREAD_ID GetCurrentThreadId()

// Controls the thread opening mode. JOINABLE if you want the thread to only end and free its resources
// when calling wait_thread_end on it. DETACHED if you want it to do that by itself.
enum { DETACHED, JOINABLE };


// Aliases for platform abstraction
typedef HANDLE Thread_Handle;
typedef CRITICAL_SECTION* Thread_Lock;

// List of created mutexes
static CRITICAL_SECTION* lock_list;
static size_t n_locks = 0;

// List of manipulators for the created (joinable) threads
static HANDLE* handle_list;
static size_t n_handles = 0;

// Number of running threads
static size_t n_threads = 0;

// Request new unique mutex for using in thread syncronization
Thread_Lock get_new_thread_lock()
{
  lock_list = (CRITICAL_SECTION*) realloc( lock_list, ( n_locks + 1 ) * sizeof(CRITICAL_SECTION) );
  InitializeCriticalSection( &lock_list[ n_locks ] );
  return &lock_list[ n_locks++ ];
}

// Setup new thread to run the given method asyncronously
Thread_Handle run_thread( void* (*function)( void* ), void* args, int mode )
{
  static HANDLE handle;
  static unsigned int thread_id;
  
  if( (handle = (HANDLE) _beginthreadex( NULL, 0, (unsigned int (__stdcall *) (void*)) function, args, 0, &thread_id )) == NULL )
  {
    fprintf( stderr, "run_thread: _beginthreadex: error creating new thread: code: %x\n", GetLastError() );
    return 0;
  }
  
  #ifdef DEBUG_2
  printf( "run_thread: created thread %x successfully\n", handle_list[ n_handles ] );
  #endif

  n_threads++;
  
  if( mode == JOINABLE )
  {
	handle_list = (HANDLE*) realloc( handle_list, ( n_handles + 1 ) * sizeof(HANDLE) );
	handle_list[ n_handles ] = handle;
	n_handles++;
  }
  else if( mode == DETACHED ) 
	CloseHandle( handle );

  return handle;
}

// Exit the calling thread, returning the given value
void exit_thread( uint32_t exit_code )
{
  n_threads--;

  #ifdef DEBUG_2
  printf( "exit_thread: thread exiting with code: %u\n", exit_code );
  #endif

  _endthreadex( (DWORD) exit_code );
}

// Wait for the thread of the given manipulator to exit and return its exiting value
uint32_t wait_thread_end( Thread_Handle handle, unsigned int milisseconds )
{
  static DWORD exit_code = 0;
  static DWORD exit_status = WAIT_OBJECT_0;

  #ifdef DEBUG_1
  printf( "wait_thread_end: waiting thread %x\n", handle );
  #endif

  exit_status = WaitForSingleObject( handle, INFINITE /*(DWORD) milisseconds*/ );
  if( exit_status == WAIT_FAILED )
  {
    fprintf( stderr, "wait_thread_end: WaitForSingleObject: error waiting for thread end: code: %x\n", GetLastError() );
    return 0;
  }
  else if( exit_status == WAIT_TIMEOUT )
  {
    fprintf( stderr, "wait_thread_end: WaitForSingleObject: wait timed out\n" );
    return 0;
  }

  #ifdef DEBUG_1
  printf( "wait_thread_end: thread returned\n" );
  #endif

  if( GetExitCodeThread( handle, &exit_code ) != 0 )
  {
    #ifdef DEBUG_1
    printf( "wait_thread_end: thread exit code: %u\n", exit_code );
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
  size_t id;
  
  for( id = 0; id < n_locks; id++ )
    DeleteCriticalSection( &(lock_list[ id ]) );
  
  /*for( id = 0; id < n_handles; id++ )
  {
    if( CloseHandle( handle_list[ id ] ) == 0 )
		fprintf( stderr, "end_threading: CloseHandle: error trying to close handle: code: %x", GetLastError() );
  }*/
  
  return;
}

#ifdef __cplusplus
}
#endif

#endif /* THREADS_H */ 
