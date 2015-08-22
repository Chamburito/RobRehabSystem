///////////////////////////////////////////////////////////////////////////////
///// Wrapper library for thread management (creation and syncronization) ///// 
///// using low level operating system native methods (Posix Version)     /////
///////////////////////////////////////////////////////////////////////////////

#ifndef THREADS_H
#define THREADS_H

#include <pthread.h>
#include <semaphore.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <errno.h>
#include <malloc.h>

#include "klib/khash.h"

#include "debug.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                      THREADS HANDLING                                       /////
///////////////////////////////////////////////////////////////////////////////////////////////////////

const int INVALID_THREAD_HANDLE = -1;
#define INFINITE 0xffffffff
  
// Returns unique identifier of the calling thread
#define THREAD_ID pthread_self()

// Controls the thread opening mode. JOINABLE if you want the thread to only end and free its resources
// when calling wait_thread_end on it. DETACHED if you want it to do that by itself.
enum { THREAD_DETACHED, THREAD_JOINABLE };

typedef struct _Controller
{
  pthread_t handle;
  pthread_cond_t condition;
  pthread_mutex_t lock;
  void* result;
} Controller;

// Aliases for platform abstraction
typedef int Thread_Handle;

// Number of running threads
KHASH_MAP_INIT_INT( Thread, pthread_t )
static khash_t( Thread )* threadsList = NULL;

// Setup new thread to run the given method asyncronously
Thread_Handle Thread_Start( void* (*function)( void* ), void* args, int mode )
{
  pthread_t handle;
  
  if( pthread_create( &handle, NULL, function, args ) != 0 )
  {
    ERROR_PRINT( "pthread_create: failed creating new thread with function %p", function );
    return INVALID_THREAD_HANDLE;
  }

  if( threadsList == NULL ) threadsList = kh_init( Thread );
  
  int insertionStatus;
  khint_t threadID = kh_put( Thread, threadsList, *((int*) &handle), &insertionStatus );
  if( insertionStatus != 0 ) return INVALID_THREAD_HANDLE;
  
  DEBUG_PRINT( "created thread %d successfully\n", threadID );
  
  if( mode == THREAD_DETACHED ) pthread_detach( handle );
  
  kh_value( threadsList, threadID ) = handle;

  return (int) threadID;
}

// Exit the calling thread, returning the given value
void Thread_Exit( uint32_t exit_code )
{
  static uint32_t exit_code_storage;
  exit_code_storage = exit_code;
  
  for( khint_t threadID = kh_begin( threadsList ); threadID != kh_end( threadsList ); threadID++ )
  {
    if( pthread_equal( pthread_self(), kh_value( threadsList, threadID ) ) )
    {
      kh_del( Thread, threadsList, threadID );
      
      DEBUG_PRINT( "thread %d exiting with code: %u", (int) threadID, exit_code );
      
      if( kh_size( threadsList ) == 0 )
      {
        kh_destroy( Thread, threadsList );
        threadsList = NULL;
      }
      
      break;
    }
  }

  pthread_exit( &exit_code_storage );
}

// Waiter function to be called asyncronously
static void* Waiter( void *args )
{
  Controller* controller = (Controller*) args;
    
  pthread_join( controller->handle, &(controller->result) );
  pthread_mutex_lock( &(controller->lock) );
  pthread_mutex_unlock( &(controller->lock) );
  pthread_cond_signal( &(controller->condition) );

  pthread_exit( NULL );
  return 0;
}

// Wait for the thread of the given manipulator to exit and return its exiting value
uint32_t Thread_WaitExit( Thread_Handle handle, unsigned int milisseconds )
{
  static struct timespec timeout;
  pthread_t controlHandle;
  Controller controlArgs = { handle };
  int controlResult;

  if( handle != INVALID_THREAD_HANDLE )
  {
    clock_gettime( CLOCK_REALTIME, &timeout );

    if( timeout.tv_sec + ( milisseconds / 1000 ) < INFINITE )
    {
      timeout.tv_sec += (time_t) ( milisseconds / 1000 );
      timeout.tv_nsec += (long) 1000000 * ( milisseconds % 1000 );
    }
  
    pthread_mutex_init( &(controlArgs.lock), 0 );
    pthread_cond_init( &(controlArgs.condition), 0 );
    pthread_mutex_lock( &(controlArgs.lock) );

    if( pthread_create( &controlHandle, NULL, Waiter, (void*) &controlArgs ) != 0 )
    {
      perror( "wait_thread_end: pthread_create: error creating waiter thread:" );
      return 0;
    }
  
    DEBUG_PRINT( "waiting thread %x exit", handle );
  
    do controlResult = pthread_cond_timedwait( &(controlArgs.condition), &(controlArgs.lock), &timeout );
    while( controlArgs.result != NULL && controlResult != ETIMEDOUT );

    pthread_cancel( controlHandle );
    pthread_join( controlHandle, NULL );

    pthread_cond_destroy( &(controlArgs.condition) );
    pthread_mutex_destroy( &(controlArgs.lock) );
  
    DEBUG_PRINT( "waiter for thread %x exited", handle );
  
    if( controlArgs.result != NULL )
    {
      DEBUG_PRINT( "thread %x exit code: %u", handle, *((uint32_t*) (controlArgs.result)) );
    
      return *((uint32_t*) (controlArgs.result));
    }
    else 
      DEBUG_PRINT( "waiting for thread %x timed out", handle );
  }

  return 0;
}

// Returns number of running threads (method for encapsulation purposes)
size_t Thread_GetActiveThreadsNumber()
{
  return kh_size( threadsList );
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                     THREAD LOCK (MUTEX)                                     /////
///////////////////////////////////////////////////////////////////////////////////////////////////////

typedef pthread_mutex_t* ThreadLock;

// Request new unique mutex for using in thread syncronization
ThreadLock ThreadLock_Create()
{
  pthread_mutex_t* newLock = (pthread_mutex_t*) malloc( sizeof(pthread_mutex_t) );
  pthread_mutex_init( newLock, NULL );
  return newLock;
}

extern inline void ThreadLock_Discard( pthread_mutex_t* lock )
{
  pthread_mutex_destroy( lock );
  free( lock );
}

// Mutex aquisition and release
extern inline void ThreadLock_Aquire( pthread_mutex_t* lock ) { pthread_mutex_lock( lock ); }
extern inline void ThreadLock_Release( pthread_mutex_t* lock ) { pthread_mutex_unlock( lock ); }


///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                       THREAD SEMAPHORE                                      /////
///////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct 
{
  sem_t upCounter;
  sem_t downCounter;
}
Semaphore;

Semaphore* Semaphore_Create( size_t start_count, size_t max_count )
{
  Semaphore* sem = (Semaphore*) malloc( sizeof(Semaphore) );
  sem_init( &(sem->upCounter), 0, max_count - start_count );
  sem_init( &(sem->downCounter), 0, start_count );
  
  return sem;
}

extern inline void Semaphore_Discard( Semaphore* sem )
{
  sem_destroy( &(sem->upCounter) );
  sem_destroy( &(sem->downCounter) );
  free( sem );
}

extern inline void Semaphore_Increment( Semaphore* sem )
{
  sem_wait( &(sem->upCounter) );
  sem_post( &(sem->downCounter) );  
}

extern inline void Semaphore_Decrement( Semaphore* sem )
{
  sem_wait( &(sem->downCounter) );
  sem_post( &(sem->upCounter) ); 
}

extern inline size_t Semaphore_GetValue( Semaphore* sem )
{
  int countValue;
  sem_getvalue( &(sem->downCounter), &countValue );
  
  return (size_t) countValue;
}


#endif /* THREADS_H */
