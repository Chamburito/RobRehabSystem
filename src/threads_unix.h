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

#define INFINITE 0xffffffff

///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                      THREADS HANDLING                                       /////
///////////////////////////////////////////////////////////////////////////////////////////////////////
  
// Returns unique identifier of the calling thread
#define THREAD_ID pthread_self()

// Controls the thread opening mode. JOINABLE if you want the thread to only end and free its resources
// when calling wait_thread_end on it. DETACHED if you want it to do that by itself.
enum { DETACHED, JOINABLE };

typedef struct _Controller
{
  pthread_t handle;
  pthread_cond_t condition;
  pthread_mutex_t lock;
  void* result;
} Controller;

// Aliases for platform abstraction
typedef pthread_t Thread_Handle;

// Number of running threads
static size_t threadsNumber = 0;

// Setup new thread to run the given method asyncronously
Thread_Handle Thread_Start( void* (*function)( void* ), void* args, int mode )
{
  static pthread_t handle;
  
  if( pthread_create( &handle, NULL, function, args ) != 0 )
  {
    ERROR_PRINT( "pthread_create: failed creating new thread with function %p", function );
    return 0;
  }

  DEBUG_PRINT( "created thread %x successfully\n", handle );

  threadsNumber++;
  
  if( mode == DETACHED ) pthread_detach( handle );

  return handle;
}

// Exit the calling thread, returning the given value
void Thread_Exit( uint32_t exit_code )
{
  static uint32_t exit_code_storage;
  exit_code_storage = exit_code;
  threadsNumber--;

  DEBUG_PRINT( "thread %x exiting with code: %u", pthread_self(), exit_code );

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
  pthread_t control_handle;
  Controller control_args = { handle };
  int control_result;

  clock_gettime( CLOCK_REALTIME, &timeout );

  if( timeout.tv_sec + ( milisseconds / 1000 ) < INFINITE )
  {
    timeout.tv_sec += (time_t) ( milisseconds / 1000 );
    timeout.tv_nsec += (long) 1000000 * ( milisseconds % 1000 );
  }
  
  pthread_mutex_init( &(control_args.lock), 0 );
  pthread_cond_init( &(control_args.condition), 0 );
  pthread_mutex_lock( &(control_args.lock) );

  if( pthread_create( &control_handle, NULL, Waiter, (void*) &control_args ) != 0 )
  {
    perror( "wait_thread_end: pthread_create: error creating waiter thread:" );
    return 0;
  }
  
  DEBUG_PRINT( "waiting thread %x exit", handle );
  
  do control_result = pthread_cond_timedwait( &(control_args.condition), &(control_args.lock), &timeout );
  while( control_args.result != NULL && control_result != ETIMEDOUT );

  pthread_cancel( control_handle );
  pthread_join( control_handle, NULL );

  pthread_cond_destroy( &(control_args.condition) );
  pthread_mutex_destroy( &(control_args.lock) );
  
  DEBUG_PRINT( "waiter for thread %x exited", handle );
  
  if( control_args.result != NULL )
  {
    DEBUG_PRINT( "thread %x exit code: %u", handle, *((uint32_t*) (control_args.result)) );
    
    return *((uint32_t*) (control_args.result));
  }
  else 
    DEBUG_PRINT( "waiting for thread %x timed out", handle );
  
  return 0;
}

// Returns number of running threads (method for encapsulation purposes)
size_t Thread_GetActiveThreadsNumber()
{
  return threadsNumber;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                     THREAD LOCK (MUTEX)                                     /////
///////////////////////////////////////////////////////////////////////////////////////////////////////

typedef pthread_mutex_t* ThreadLock;

// Request new unique mutex for using in thread syncronization
ThreadLock ThreadLock_Create()
{
  pthread_mutex_t* lock = (pthread_mutex_t*) malloc( sizeof(pthread_mutex_t) );
  pthread_mutex_init( lock, NULL );
  return lock;
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
