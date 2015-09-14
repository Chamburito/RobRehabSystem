/*! \file shared_memory_unix.h
    \brief Memory sharing functions using the POSIX (shm*) API.
*/

#ifndef SHARED_MEMORY_H
#define SHARED_MEMORY_H

#include <sys/mman.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/stat.h>

#include "klib/khash.h"

#include "debug/async_debug.h"

// Access permissions
#define SHM_READ S_IRUSR
#define SHM_WRITE S_IWUSR
#define SHM_READ_WRITE ( S_IRUSR | S_IWUSR )

/**
 * A hash table 
 * Stores pointers to the created shared memory areas
 */
KHASH_MAP_INIT_STR( SOStr, void* )
khash_t( SOStr )* sharedObjectsList = NULL;

/** 
 * Function definition
 * Shared memory high level functions
 */
static void* CreateObject( const char*, size_t, int );
static void DestroyObject( void* );

/** 
 * A function pointer structure
 * Struct exported as our shared memory interface 
 */
const struct 
{ 
  void* (*CreateObject)( const char*, size_t, int );
  void (*DestroyObject)( void* );
} 
SharedObjects = { CreateObject, DestroyObject };

/**
 * Creates shared memory area and returns its pointer
 * @param mappingName name of the shared memory area (mapped file)
 * @param objectSize size in bytes of the shared memory area
 * @param flags bitfield containing access permissions ( read-only, write-only or read-write )
 * @return generic (void*) pointer to the created memory area (returns (void*) -1 when fails)
 */
static void* CreateObject( const char* mappingName , size_t objectSize, int flags )
{
  // Shared memory is mapped to a file. So we create a new file.
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
  
  // Generates exclusive key from the file name (same name generate same key)
  key_t sharedKey = ftok( mappingName, 1 );
  if( sharedKey == -1 )
  {
    perror( "Failed to aquire shared memory key" );
    return (void*) -1;
  }
  
  // Reserves shared memory area and returns a file descriptor to it
  int sharedMemoryID = shmget( sharedKey, objectSize, IPC_CREAT | flags );
  if( sharedMemoryID == -1 )
  {
    perror( "Failed to create shared memory segment" );
    return (void*) -1;
  }
  
  DEBUG_PRINT( "Got shared memory area ID %d", sharedMemoryID );
  
  // Maps created shared memory area to program address (pointer)
  void* newSharedObject = shmat( sharedMemoryID, NULL, 0 );
  if( newSharedObject == (void*) -1 ) 
  {
    perror( "Failed to bind object" );
    return (void*) -1;
  }
  
  DEBUG_PRINT( "Binded object address %p to shared memory area", newSharedObject );
  
  // 
  if( sharedObjectsList == NULL ) sharedObjectsList = kh_init( SOStr );
  
  int insertionStatus;
  khint_t newSharedMemoryID = kh_put( SOStr, sharedObjectsList, mappingName, &insertionStatus );
  
  kh_value( sharedObjectsList, newSharedMemoryID ) = newSharedObject;
  
  return newSharedObject;
}

/**
 * Discards shared memory area and remove its pointer from the hash table
 * @param sharedObject pointer to the shared memory area
 */
static void DestroyObject( void* sharedObject )
{
  for( khint_t sharedObjectID = 0; sharedObjectID != kh_end( sharedObjectsList ); sharedObjectID++ )
  {
    if( !kh_exist( sharedObjectsList, sharedObjectID ) ) continue;
    
    if( kh_value( sharedObjectsList, sharedObjectID ) == sharedObject )
    {
      shmdt( sharedObject );
      (void) remove( kh_key( sharedObjectsList, sharedObjectID ) );
      kh_del( SOStr, sharedObjectsList, sharedObjectID );
      
      if( kh_size( sharedObjectsList ) == 0 )
      {
        kh_destroy( SOStr, sharedObjectsList );
        sharedObjectsList = NULL;
      }
      
      break;
    }
  }
}


#endif // SHARED_MEMORY_H
