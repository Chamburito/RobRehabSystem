#include <sys/mman.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/stat.h>

#include "klib/khash.h"

#include "debug/async_debug.h"

#include "shared_memory/shared_memory.h"

/// A hash table 
/// Stores pointers to the created shared memory areas
KHASH_MAP_INIT_STR( SOStr, void* )
khash_t( SOStr )* sharedObjectsList = NULL;


DEFINE_NAMESPACE_INTERFACE( SharedObjects, SHARED_MEMORY_INTERFACE )


void* SharedObjects_CreateObject( const char* mappingName , size_t objectSize, uint8_t flags )
{
  char mappingFilePath[ SHARED_OBJECT_PATH_MAX_LENGTH ];
  
  sprintf( mappingFilePath, "/dev/shm/%s", mappingName );
  
  DEBUG_PRINT( "trying to open shared memory object %s", mappingFilePath );
  
  // Shared memory is mapped to a file. So we create a new file.
  FILE* mappedFile = fopen( mappingFilePath, "r+" );
  if( mappedFile == NULL )
  {
    if( (mappedFile = fopen( mappingFilePath, "w+" )) == NULL )
    {
      perror( "Failed to open memory mapped file" );
      return (void*) -1;
    }
  }
  fclose( mappedFile );
  
  // Generates exclusive key from the file name (same name generate same key)
  key_t sharedKey = ftok( mappingFilePath, 1 );
  if( sharedKey == -1 )
  {
    perror( "Failed to aquire shared memory key" );
    return (void*) -1;
  }
  
  // Reserves shared memory area and returns a file descriptor to it
  int accessFlags = 0;
  if( flags & SHM_READ ) accessFlags |= S_IRUSR;
  if( flags & SHM_WRITE ) accessFlags |= S_IWUSR;
  int sharedMemoryID = shmget( sharedKey, objectSize, IPC_CREAT | /*accessFlags*/ 0660 );
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

void SharedObjects_DestroyObject( void* sharedObject )
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
