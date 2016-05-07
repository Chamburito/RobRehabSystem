///////////////////////////////////////////////////////////////////////////////
///// Wrapper library for thread management (creation and syncronization) ///// 
///// using low level operating system native methods                     /////
///////////////////////////////////////////////////////////////////////////////

#ifndef THREADING_H
#define THREADING_H

#include "interfaces.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                      THREADS HANDLING                                       /////
///////////////////////////////////////////////////////////////////////////////////////////////////////

#define THREAD_INVALID_HANDLE NULL
#ifndef INFINITE
  #define INFINITE 0xFFFFFFFF
#endif

// Controls the thread opening mode. JOINABLE if you want the thread to only end and free its resources
// when calling wait_thread_end on it. DETACHED if you want it to do that by itself.
enum { THREAD_DETACHED, THREAD_JOINABLE };

// Aliases for platform abstraction
typedef void* Thread;
typedef void* (*AsyncFunction)( void* );

#define THREAD_INTERFACE( Namespace, INIT_FUNCTION ) \
        INIT_FUNCTION( Thread, Namespace, StartThread, AsyncFunction, void*, int ) \
        INIT_FUNCTION( uint32_t, Namespace, WaitExit, Thread, unsigned int ) \
        INIT_FUNCTION( unsigned long, Namespace, GetCurrentThreadID, void )

DECLARE_NAMESPACE_INTERFACE( Threading, THREAD_INTERFACE )

// Returns unique identifier of the calling thread
#define THREAD_ID Threading.GetCurrentThreadID()


///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                     THREAD LOCK (MUTEX)                                     /////
///////////////////////////////////////////////////////////////////////////////////////////////////////

typedef void* ThreadLock;

#define THREAD_LOCK_INTERFACE( Namespace, INIT_FUNCTION ) \
        INIT_FUNCTION( ThreadLock, Namespace, Create, void ) \
        INIT_FUNCTION( void, Namespace, Discard, ThreadLock ) \
        INIT_FUNCTION( void, Namespace, Aquire, ThreadLock ) \
        INIT_FUNCTION( void, Namespace, Release, ThreadLock )

DECLARE_NAMESPACE_INTERFACE( ThreadLocks, THREAD_LOCK_INTERFACE )


///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                       THREAD SEMAPHORE                                      /////
///////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct _SemaphoreData SemaphoreData;
typedef SemaphoreData* Semaphore;

#define SEMAPHORE_INTERFACE( Namespace, INIT_FUNCTION ) \
        INIT_FUNCTION( Semaphore, Namespace, Create, size_t, size_t ) \
        INIT_FUNCTION( void, Namespace, Discard, Semaphore ) \
        INIT_FUNCTION( void, Namespace, Increment, Semaphore ) \
        INIT_FUNCTION( void, Namespace, Decrement, Semaphore ) \
        INIT_FUNCTION( size_t, Namespace, GetCount, Semaphore ) \
        INIT_FUNCTION( void, Namespace, SetCount, Semaphore, size_t )

DECLARE_NAMESPACE_INTERFACE( Semaphores, SEMAPHORE_INTERFACE )


#endif /* THREADING_H */
