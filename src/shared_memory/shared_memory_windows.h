#ifndef SHARED_MEMORY_H
#define SHARED_MEMORY_H

#include <sys/mman.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/stat.h>

#include "debug/async_debug.h"

#define SHM_READ GENERIC_READ
#define SHM_WRITE GENERIC_WRITE

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

void* CreateByName( const char* mappingName, size_t objectSize, int flags )
{
  struct FILE* mappedFile = fopen( mappingName, "r+" );
  fclose( mappedFile );
  
  
  
  return CreateByKey( sharedKey, objectSize, flags );
}

void* CreateByKey( int key, size_t objectSize, int flags )
{
  
  
  return newSharedObject;
}

void Destroy( void* sharedObject )
{
  
}


#endif // SHARED_MEMORY_H
