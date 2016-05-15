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
