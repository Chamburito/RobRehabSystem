#include <windows.h>

#include "klib/khash.h"

#include "debug/async_debug.h"

#include "shared_memory/shared_memory.h"

typedef struct _SharedObject
{
  HANDLE handle;
  void* data;
}
SharedObject;

KHASH_MAP_INIT_STR( SO, SharedObject )
khash_t( SO )* sharedObjectsList = NULL;


DEFINE_NAMESPACE_INTERFACE( SharedObjects, SHARED_MEMORY_INTERFACE )


void* SharedObjects_CreateObject( const char* mappingName, size_t objectSize, uint8_t flags )
{
  int accessFlag = FILE_MAP_ALL_ACCESS;
  if( flags == SHM_READ ) accessFlag = FILE_MAP_READ;
  if( flags == SHM_WRITE ) accessFlag = FILE_MAP_WRITE;
  HANDLE mappedFile = OpenFileMapping( accessFlag, FALSE, mappingName );
  if( mappedFile == NULL )
  {
    mappedFile = CreateFileMapping( INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0, objectSize, mappingName );
    if( mappedFile == NULL )
    {
      return (void*) -1;
    }
  }
  
  if( sharedObjectsList == NULL ) sharedObjectsList = kh_init( SO );
  
  int insertionStatus;
  khint_t newSharedMemoryID = kh_put( SO, sharedObjectsList, mappingName, &insertionStatus );
  
  kh_value( sharedObjectsList, newSharedMemoryID ).handle = mappedFile;
  kh_value( sharedObjectsList, newSharedMemoryID ).data = MapViewOfFile( mappedFile, flags, 0, 0, 0 );
  
  return kh_value( sharedObjectsList, newSharedMemoryID ).data;
}

void SharedObjects_DestroyObject( void* sharedObjectData )
{
  for( khint_t sharedObjectID = 0; sharedObjectID != kh_end( sharedObjectsList ); sharedObjectID++ )
  {
    if( !kh_exist( sharedObjectsList, sharedObjectID ) ) continue;
    
    if( kh_value( sharedObjectsList, sharedObjectID ).data == sharedObjectData )
    {
      UnmapViewOfFile( sharedObjectData );
      CloseHandle( kh_value( sharedObjectsList, sharedObjectID ).handle );
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
