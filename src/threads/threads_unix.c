////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  Copyright (c) 2016 Leonardo José Consoni                                  //
//                                                                            //
//  This file is part of RobRehabSystem.                                      //
//                                                                            //
//  RobRehabSystem is free software: you can redistribute it and/or modify    //
//  it under the terms of the GNU Lesser General Public License as published  //
//  by the Free Software Foundation, either version 3 of the License, or      //
//  (at your option) any later version.                                       //
//                                                                            //
//  RobRehabSystem is distributed in the hope that it will be useful,         //
//  but WITHOUT ANY WARRANTY; without even the implied warranty of            //
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the              //
//  GNU Lesser General Public License for more details.                       //
//                                                                            //
//  You should have received a copy of the GNU Lesser General Public License  //
//  along with RobRehabSystem. If not, see <http://www.gnu.org/licenses/>.    //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
///// Wrapper library for thread management (creation and syncronization) ///// 
///// using low level operating system native methods (Posix Version)     /////
///////////////////////////////////////////////////////////////////////////////

#include <pthread.h>
#include <semaphore.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <errno.h>
#include <malloc.h>

#include "debug/sync_debug.h"

#include "threads/threading.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                      THREADS HANDLING                                       /////
///////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct _ThreadController
{
  pthread_t handle;
  pthread_cond_t condition;
  pthread_mutex_t lock;
  void* result;
} ThreadController;

DEFINE_NAMESPACE_INTERFACE( Threading, THREAD_INTERFACE )

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

  return (Thread) handle;
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
  ThreadController controlArgs = { .handle = *((pthread_t*) handle) };
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

unsigned long Threading_GetCurrentThreadID()
{
  return (unsigned long) pthread_self();
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                     THREAD LOCK (MUTEX)                                     /////
///////////////////////////////////////////////////////////////////////////////////////////////////////

DEFINE_NAMESPACE_INTERFACE( ThreadLocks, THREAD_LOCK_INTERFACE )

// Request new unique mutex for using in thread syncronization
ThreadLock ThreadLocks_Create()
{
  pthread_mutex_t* newLock = (pthread_mutex_t*) malloc( sizeof(pthread_mutex_t) );
  pthread_mutex_init( newLock, NULL );
  return (ThreadLock) newLock;
}

void ThreadLocks_Discard( ThreadLock lock )
{
  pthread_mutex_destroy( (pthread_mutex_t*) lock );
  free( lock );
}

// Mutex aquisition and release
void ThreadLocks_Aquire( ThreadLock lock ) { pthread_mutex_lock( (pthread_mutex_t*) lock ); }
void ThreadLocks_Release( ThreadLock lock ) { pthread_mutex_unlock( (pthread_mutex_t*) lock ); }


///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                       THREAD SEMAPHORE                                      /////
///////////////////////////////////////////////////////////////////////////////////////////////////////

struct _SemaphoreData
{
  sem_t upCounter;
  sem_t downCounter;
  size_t maxCount;
};

DEFINE_NAMESPACE_INTERFACE( Semaphores, SEMAPHORE_INTERFACE )

Semaphore Semaphores_Create( size_t startCount, size_t maxCount )
{
  Semaphore sem = (Semaphore) malloc( sizeof(SemaphoreData) );
  sem_init( &(sem->upCounter), 0, maxCount - startCount );
  sem_init( &(sem->downCounter), 0, startCount );
  
  sem->maxCount = maxCount;
  
  return sem;
}

void Semaphores_Discard( Semaphore sem )
{
  sem_destroy( &(sem->upCounter) );
  sem_destroy( &(sem->downCounter) );
  free( sem );
}

void Semaphores_Increment( Semaphore sem )
{
  sem_wait( &(sem->upCounter) );
  sem_post( &(sem->downCounter) );  
}

void Semaphores_Decrement( Semaphore sem )
{
  sem_wait( &(sem->downCounter) );
  sem_post( &(sem->upCounter) ); 
}

size_t Semaphores_GetCount( Semaphore sem )
{
  int countValue;
  sem_getvalue( &(sem->downCounter), &countValue );
  
  return (size_t) countValue;
}

void Semaphores_SetCount( Semaphore sem, size_t count )
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
