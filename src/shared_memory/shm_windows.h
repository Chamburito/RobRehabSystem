#ifndef SHARED_MEMORY_H
#define SHARED_MEMORY_H

#include <windows.h>

#include "klib/khash.h"

#include "debug/async_debug.h"

#define SHM_READ FILE_MAP_READ
#define SHM_WRITE FILE_MAP_WRITE
#define SHM_READ_WRITE FILE_MAP_ALL_ACCESS

typedef struct _SharedObject
{
  HANDLE handle;
  void* data;
}
SharedObject;

KHASH_MAP_INIT_STR( SO, SharedObject )
khash_t( SO )* sharedObjectsList = NULL;

void* CreateObject( const char*, size_t, int );
void DestroyObject( void* );

const struct 
{ 
  void* (*CreateObject)( const char*, size_t, int );
  void (*DestroyObject)( void* );
} 
SharedObjects = { CreateObject, DestroyObject };

void* CreateObject( const char* mappingName, size_t objectSize, int flags )
{
  HANDLE mappedFile = OpenFileMapping( flags, FALSE, mappingName );
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

void DestroyObject( void* sharedObjectData )
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


#endif // SHARED_MEMORY_H
