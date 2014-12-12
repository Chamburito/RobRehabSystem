#ifndef THREADS_H
#define THREADS_H

#ifdef __cplusplus
extern "C"{
#endif

#include <Windows.h>
#include <process.h>
#include <stdio.h>

#define LOCK_THREAD( lock ) TryEnterCriticalSection( lock )
#define UNLOCK_THREAD( lock ) LeaveCriticalSection( lock )
#define THREAD_ID GetCurrentThreadId()

typedef HANDLE Thread_Handle;
typedef CRITICAL_SECTION* Thread_Lock;

static CRITICAL_SECTION* lock_list;
static size_t n_locks = 0;

static HANDLE* handle_list;
static size_t n_handles = 0;

static size_t n_threads = 0;

Thread_Lock get_new_thread_lock()
{
  lock_list = (CRITICAL_SECTION*) realloc( lock_list, ( n_locks + 1 ) * sizeof(CRITICAL_SECTION) );
  InitializeCriticalSection( &lock_list[ n_locks ] );
  return &lock_list[ n_locks++ ];
}

Thread_Handle run_thread( void* (*function)( void* ), void* args )
{
  static unsigned int thread_id;
  
  handle_list = (HANDLE*) realloc( handle_list, ( n_handles + 1 ) * sizeof(HANDLE) );
  
  if( (handle_list[ n_handles ] = (HANDLE) _beginthreadex( NULL, 0, (unsigned int (__stdcall *) (void*)) function, args, 0, &thread_id )) == NULL )
  {
    fprintf( stderr, "run_thread: _beginthreadex: error creating new thread: code: %x", GetLastError() );
    return 0;
  }
  
  n_threads++;
  
  return handle_list[ n_handles++ ];
}

void exit_thread( uint32_t exit_code )
{
  n_threads--;
  _endthreadex( (DWORD) exit_code );
}

// Aguarda o encerramento de uma thread e retorna seu código de saída
uint32_t wait_thread_end( Thread_Handle handle )
{
  static DWORD exit_code;

  if( WaitForSingleObject( handle, INFINITE ) == WAIT_FAILED )
  {
    perror( "wait_thread_end: WaitForSingleObject: error waiting for thread end:" );
    return 0;
  }

  if( GetExitCodeThread( handle, &exit_code ) != 0 )
  {
    printf( "wait_thread_end: thread exit code: %u\n", exit_code );
    return exit_code;
  }
  else
    return 0;
}

size_t get_threads_number()
{
  return n_threads;
}

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
