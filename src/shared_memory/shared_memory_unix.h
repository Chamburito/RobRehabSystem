#ifndef SHARED_MEMORY_H
#define SHARED_MEMORY_H

#include <sys/mman.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/stat.h>

#include "klib/khash.h"

#include "debug/async_debug.h"

#define SHM_READ S_IRUSR
#define SHM_WRITE S_IWUSR
#define SHM_READ_WRITE ( S_IRUSR | S_IWUSR )

KHASH_MAP_INIT_STR( SO, void* )
khash_t( SO )* sharedObjectsList = NULL;

void* CreateObject( const char*, size_t, int );
void DestroyObject( void* );

const struct 
{ 
  void* (*CreateObject)( const char*, size_t, int );
  void (*DestroyObject)( void* );
} 
SharedObjects = { CreateObject, DestroyObject };

void* CreateObject( const char* mappingName , size_t objectSize, int flags )
{
  FILE* mappedFile = fopen( mappingName, "r+" );
  if( mappedFile == NULL )
  {
    if( (mappedFile = fopen( mappingName, "w+" )) == NULL )
    {
      perror( "Failed to open memory mapped file" );
      return (void*) -1;
    }
  }
  
  fclose( mappedFile );
  
  key_t sharedKey = ftok( mappingName, 1 );
  if( sharedKey == -1 )
  {
    perror( "Failed to aquire shared memory key" );
    return (void*) -1;
  }
  
  int sharedMemoryID = shmget( sharedKey, objectSize, IPC_CREAT | flags );
  if( sharedMemoryID == -1 )
  {
    perror( "Failed to create shared memory segment" );
    return (void*) -1;
  }
  
  DEBUG_PRINT( "Got shared memory area ID %d", sharedMemoryID );
  
  void* newSharedObject = shmat( sharedMemoryID, NULL, 0 );
  if( newSharedObject == (void*) -1 ) 
  {
    perror( "Failed to bind object" );
    return (void*) -1;
  }
  
  DEBUG_PRINT( "Binded object address %p to shared memory area", newSharedObject );
  
  if( sharedObjectsList == NULL ) sharedObjectsList = kh_init( SO );
  
  int insertionStatus;
  khint_t newSharedMemoryID = kh_put( SO, sharedObjectsList, mappingName, &insertionStatus );
  
  kh_value( sharedObjectsList, newSharedMemoryID ) = newSharedObject;
  
  return newSharedObject;
}

void DestroyObject( void* sharedObject )
{
  for( khint_t sharedObjectID = 0; sharedObjectID != kh_end( sharedObjectsList ); sharedObjectID++ )
  {
    if( !kh_exist( sharedObjectsList, sharedObjectID ) ) continue;
    
    if( kh_value( sharedObjectsList, sharedObjectID ) == sharedObject )
    {
      shmdt( sharedObject );
      (void) remove( kh_key( sharedObjectsList, sharedObjectID ) );
      kh_del( SO, sharedObjectsList, sharedObjectID );
      
      if( kh_size( sharedObjectsList ) == 0 )
      {
        kh_destroy( SO, sharedObjectsList );
        sharedObjectsList = NULL;
      }
      
      break;
    }
  }
}


#endif // SHARED_MEMORY_H
