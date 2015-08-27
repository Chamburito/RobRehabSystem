#ifndef SHARED_MEMORY_H
#define SHARED_MEMORY_H

#include <sys/mman.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/stat.h>

#include "../../async_debug.h"

#define SHM_READ S_IRUSR
#define SHM_WRITE S_IWUSR

void* CreateByName( const char*, size_t, int );
void* CreateByKey( int, size_t, int );
void Destroy( void* );

const struct 
{ 
  void* (*CreateByName)( const char*, size_t, int );
  void* (*CreateByKey)( int, size_t, int );
  void (*Destroy)( void* );
} 
SharedObject = { CreateByName, CreateByKey, Destroy };

void* CreateByName( const char* mappingName , size_t objectSize, int flags )
{
  struct FILE* mappedFile = fopen( mappingName, "r+" );
  fclose( mappedFile );
  
  key_t sharedKey = ftok( mappingName, 1 );
  
  if( sharedKey == -1 )
  {
    perror( "Failed to create shared memory segment" );
    return (void*) -1;
  }
  
  return CreateByKey( sharedKey, objectSize, flags );
}

void* CreateByKey( int key, size_t objectSize, int flags )
{
  int sharedMemoryID = shmget( (key_t) key, objectSize, IPC_CREAT | flags );
  
  DEBUG_PRINT( "Got shared memory area ID %d", sharedMemoryID );
  
  if( sharedMemoryID == -1 )
  {
    perror( "Failed to create shared memory segment" );
    return (void*) -1;
  }
  
  void* newSharedObject = shmat( sharedMemoryID, NULL, 0 );
  
  DEBUG_PRINT( "Binded object address %p to shared memory area", newSharedObject );
  
  if( newSharedObject == (void*) -1 ) perror( "Failed to bind object" );
  
  return newSharedObject;
}

void Destroy( void* sharedObject )
{
  shmdt( sharedObject );
}


#endif // SHARED_MEMORY_H
