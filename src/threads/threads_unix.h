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

#include "debug/debug.h"
#include "interfaces.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                      THREADS HANDLING                                       /////
///////////////////////////////////////////////////////////////////////////////////////////////////////

#define THREAD_INVALID_HANDLE NULL
#define INFINITE 0xFFFFFFFF
  
// Returns unique identifier of the calling thread
#define THREAD_ID pthread_self()

// Controls the thread opening mode. JOINABLE if you want the thread to only end and free its resources
// when calling wait_thread_end on it. DETACHED if you want it to do that by itself.
enum { THREAD_DETACHED, THREAD_JOINABLE };

typedef struct _ThreadController
{
  pthread_t handle;
  pthread_cond_t condition;
  pthread_mutex_t lock;
  void* result;
} ThreadController;

// Aliases for platform abstraction
typedef pthread_t* Thread;

typedef void* (*AsyncFunction)( void* );

#define THREAD_FUNCTIONS( namespace, function_init ) \
        function_init( Thread, namespace, StartThread, AsyncFunction, void*, int ) \
        function_init( uint32_t, namespace, WaitExit, Thread, unsigned int )

INIT_NAMESPACE_INTERFACE( Threading, THREAD_FUNCTIONS )

// Setup new thread to run the given method asyncronously
Thread Threading_StartThread( void* (*function)( void* ), void* args, int mode )
{
  pthread_t* handle = (pthread_t*) malloc( sizeof(pthread_t) );
  
  if( pthread_create( handle, NULL, function, args ) != 0 )
  {
    ERROR_PRINT( "pthread_create: failed creating new thread with function %p", function );
    return THREAD_INVALID_HANDLE;
  }
  
  DEBUG_PRINT( "created thread %p successfully", handle );
  
  if( mode == THREAD_DETACHED ) pthread_detach( *handle );

  return handle;
}

// Waiter function to be called asyncronously
static void* Waiter( void *args )
{
  ThreadController* controller = (ThreadController*) args;
    
  pthread_join( controller->handle, &(controller->result) );
  pthread_mutex_lock( &(controller->lock) );
  pthread_mutex_unlock( &(controller->lock) );
  pthread_cond_signal( &(controller->condition) );

  pthread_exit( NULL );
  return 0;
}

// Wait for the thread of the given manipulator to exit and return its exiting value
uint32_t Threading_WaitExit( Thread handle, unsigned int milisseconds )
{
  static struct timespec timeout;
  pthread_t controlHandle;
  ThreadController controlArgs = { *handle };
  int controlResult;

  if( handle != THREAD_INVALID_HANDLE )
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
  
    DEBUG_PRINT( "waiting thread %p exit", handle );
  
    do controlResult = pthread_cond_timedwait( &(controlArgs.condition), &(controlArgs.lock), &timeout );
    while( controlArgs.result != NULL && controlResult != ETIMEDOUT );

    pthread_cancel( controlHandle );
    pthread_join( controlHandle, NULL );

    pthread_cond_destroy( &(controlArgs.condition) );
    pthread_mutex_destroy( &(controlArgs.lock) );
  
    DEBUG_PRINT( "waiter for thread %p exited", handle );
  
    if( controlArgs.result != NULL )
    {
      DEBUG_PRINT( "thread %p exit code: %u", handle, *((uint32_t*) (controlArgs.result)) );
    
      return *((uint32_t*) (controlArgs.result));
    }
    else 
      DEBUG_PRINT( "waiting for thread %p timed out", handle );
    
    free( handle );
  }

  return 0;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                     THREAD LOCK (MUTEX)                                     /////
///////////////////////////////////////////////////////////////////////////////////////////////////////

typedef pthread_mutex_t* ThreadLock;

#define THREAD_LOCK_FUNCTIONS( namespace, function_init ) \
        function_init( ThreadLock, namespace, Create, void ) \
        function_init( void, namespace, Discard, ThreadLock ) \
        function_init( void, namespace, Aquire, ThreadLock ) \
        function_init( void, namespace, Release, ThreadLock )

INIT_NAMESPACE_INTERFACE( ThreadLocks, THREAD_LOCK_FUNCTIONS )

// Request new unique mutex for using in thread syncronization
ThreadLock ThreadLocks_Create()
{
  pthread_mutex_t* newLock = (pthread_mutex_t*) malloc( sizeof(pthread_mutex_t) );
  pthread_mutex_init( newLock, NULL );
  return newLock;
}

extern inline void ThreadLocks_Discard( pthread_mutex_t* lock )
{
  pthread_mutex_destroy( lock );
  free( lock );
}

// Mutex aquisition and release
extern inline void ThreadLocks_Aquire( pthread_mutex_t* lock ) { pthread_mutex_lock( lock ); }
extern inline void ThreadLocks_Release( pthread_mutex_t* lock ) { pthread_mutex_unlock( lock ); }


///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                       THREAD SEMAPHORE                                      /////
///////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct 
{
  sem_t upCounter;
  sem_t downCounter;
  size_t maxCount;
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

Semaphore Semaphores_Create( size_t startCount, size_t maxCount )
{
  Semaphore sem = (Semaphore) malloc( sizeof(SemaphoreData) );
  sem_init( &(sem->upCounter), 0, maxCount - startCount );
  sem_init( &(sem->downCounter), 0, startCount );
  
  sem->maxCount = maxCount;
  
  return sem;
}

inline void Semaphores_Discard( Semaphore sem )
{
  sem_destroy( &(sem->upCounter) );
  sem_destroy( &(sem->downCounter) );
  free( sem );
}

inline void Semaphores_Increment( Semaphore sem )
{
  sem_wait( &(sem->upCounter) );
  sem_post( &(sem->downCounter) );  
}

inline void Semaphores_Decrement( Semaphore sem )
{
  sem_wait( &(sem->downCounter) );
  sem_post( &(sem->upCounter) ); 
}

inline size_t Semaphores_GetCount( Semaphore sem )
{
  int countValue;
  sem_getvalue( &(sem->downCounter), &countValue );
  
  return (size_t) countValue;
}

inline void Semaphores_SetCount( Semaphore sem, size_t count )
{
  if( count <= sem->maxCount )
  {
    size_t currentCount = Semaphores_GetCount( sem );
    
    if( currentCount > count )
    {
      for( size_t i = count; i < currentCount; i++ )
        Semaphores_Decrement( sem );
    }
    else if( currentCount < count )
    {
      for( size_t i = currentCount; i < count; i++ )
        Semaphores_Increment( sem );
    }
  }
}


#endif /* THREADS_H */
