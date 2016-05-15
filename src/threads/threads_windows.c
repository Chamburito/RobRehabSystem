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
///// using low level operating system native methods (Windows Version)   /////
///////////////////////////////////////////////////////////////////////////////

#include "debug/sync_debug.h"

#include <stdint.h>
#include <Windows.h>
//#include <process.h>

#include "threads/threading.h"
	
///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                      THREADS HANDLING 									                    /////
///////////////////////////////////////////////////////////////////////////////////////////////////////

DEFINE_NAMESPACE_INTERFACE( Threading, THREAD_INTERFACE )

// Setup new thread to run the given method asyncronously
Thread Threading_StartThread( AsyncFunction function, void* args, int mode )
{
  static HANDLE handle;
  static unsigned int threadID;
  
  if( (handle = CreateThread( NULL, 0, (LPTHREAD_START_ROUTINE) function, args, 0, &threadID )) == THREAD_INVALID_HANDLE )
  {
    ERROR_PRINT( "CreateThread: failed creating new thread with function %p", function );
    return THREAD_INVALID_HANDLE;
  }
  
  DEBUG_PRINT( "created thread %x successfully", handle );
  
  if( mode == THREAD_DETACHED ) CloseHandle( handle );

  return (Thread) handle;
}

// Wait for the thread of the given manipulator to exit and return its exiting value
uint32_t Threading_WaitExit( Thread handle, unsigned int milliseconds )
{
  static DWORD exitCode = 0;
  static DWORD exitStatus = WAIT_OBJECT_0;

  if( (HANDLE) handle != THREAD_INVALID_HANDLE )
  {
    DEBUG_PRINT( "waiting thread %x", (HANDLE) handle );

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
  
    if( CloseHandle( (HANDLE) handle ) == 0 )
      ERROR_PRINT( "CloseHandle: error trying to close thread handle %x: code: %x", (HANDLE) handle, GetLastError() );
  }

  return exitCode;
}

unsigned long Threading_GetCurrentThreadID()
{
  return GetCurrentThreadId();
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                     THREAD LOCK (MUTEX)                                     /////
///////////////////////////////////////////////////////////////////////////////////////////////////////

DEFINE_NAMESPACE_INTERFACE( ThreadLocks, THREAD_LOCK_INTERFACE )

// Request new unique mutex for using in thread syncronization
ThreadLock ThreadLocks_Create()
{
  CRITICAL_SECTION* lock = malloc( sizeof(CRITICAL_SECTION) );
  InitializeCriticalSection( lock );
  return (ThreadLock) lock;
}

void ThreadLocks_Discard( ThreadLock lock )
{
  DeleteCriticalSection( (CRITICAL_SECTION*) lock );
  free( lock );
}

// Mutex aquisition and release
void ThreadLocks_Aquire( ThreadLock lock ) { EnterCriticalSection( (CRITICAL_SECTION*) lock ); }
void ThreadLocks_Release( ThreadLock lock ) { LeaveCriticalSection( (CRITICAL_SECTION*) lock ); }


///////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                       THREAD SEMAPHORE                                      /////
///////////////////////////////////////////////////////////////////////////////////////////////////////

struct _SemaphoreData
{
  HANDLE counter;
  size_t count, maxCount;
};

DEFINE_NAMESPACE_INTERFACE( Semaphores, SEMAPHORE_INTERFACE )

inline Semaphore Semaphores_Create( size_t startCount, size_t maxCount )
{
  Semaphore sem = (Semaphore) malloc( sizeof(SemaphoreData) );
  sem->counter = CreateSemaphore( NULL, startCount, maxCount, NULL );
  sem->count = startCount;
  sem->maxCount = maxCount;
  
  return sem;
}

void Semaphores_Discard( Semaphore sem )
{
  CloseHandle( sem->counter );
  free( sem );
}

void Semaphores_Increment( Semaphore sem )
{
  ReleaseSemaphore( sem->counter, 1, &(sem->count) );
  sem->count++;  
}

void Semaphores_Decrement( Semaphore sem )
{
  WaitForSingleObject( sem->counter, INFINITE );
  sem->count--;
}

size_t Semaphores_GetCount( Semaphore sem )
{
  if( sem == NULL ) return 0;
  
  return sem->count;
}

void Semaphores_SetCount( Semaphore sem, size_t count )
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
