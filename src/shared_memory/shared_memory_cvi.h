#ifndef SHARED_MEMORY_H
#define SHARED_MEMORY_H

#include "asynctmr.h"
#include <cvinetv.h>
#include <cvirte.h>		
#include <userint.h>

#include "klib/khash.h"

#include "debug/async_debug.h"

#define SHM_READ 0x0f
#define SHM_WRITE 0xf0
#define SHM_READ_WRITE ( SHM_READ | SHM_WRITE )

typedef struct _SharedObject
{
  CNVSubscriber reader;
  CNVWriter writer;
  int timerID;
  CNVData networkData;
  void* data;
  size_t dataSize;
  void* oldData;
}
SharedObject;

KHASH_MAP_INIT_STR( SO, SharedObject )
khash_t( SO )* sharedObjectsList = NULL;

#define SHARED_MEMORY_FUNCTIONS( namespace, function_init ) \
        function_init( void*, namespace, CreateObject, const char*, size_t, int ) \
        function_init( void, namespace, DestroyObject, void* )

INIT_NAMESPACE_INTERFACE( SharedObjects, SHARED_MEMORY_FUNCTIONS )

void CVICALLBACK UpdateDataIn( void*, CNVData, void* );
int CVICALLBACK UpdateDataOut( int, int, int, void*, int, int );

void* SharedObjects_CreateObject( const char* mappingName, size_t objectSize, int flags )
{
  DEBUG_PRINT( "trying to create shared object %s with %u bytes (mode: %d)", mappingName, objectSize, flags );
  
  if( sharedObjectsList != NULL )
  {
    khint_t sharedObjectID = kh_get( SO, sharedObjectsList, mappingName );
    if( sharedObjectID != kh_end( sharedObjectsList ) )
    {
      DEBUG_PRINT( "shared object %s already exists", mappingName );
      return kh_value( sharedObjectsList, sharedObjectID ).data;
    }
  }
  
  if( sharedObjectsList == NULL ) sharedObjectsList = kh_init( SO ); 
  
  int insertionStatus;
  khint_t newSharedObjectID = kh_put( SO, sharedObjectsList, mappingName, &insertionStatus );

  SharedObject* newSharedObject = &(kh_value( sharedObjectsList, newSharedObjectID ));
  
  memset( newSharedObject, 0, sizeof(SharedObject) );
  
  newSharedObject->data = malloc( objectSize );
  memset( newSharedObject->data, 0, objectSize );
  newSharedObject->oldData = malloc( objectSize );
  memset( newSharedObject->oldData, 0, objectSize );
  
  newSharedObject->dataSize = objectSize;
  CNVCreateArrayDataValue( &(newSharedObject->networkData), CNVUInt8, newSharedObject->data, 1, &(newSharedObject->dataSize) );
  
  char* variablePathName = (char*) calloc( strlen( "\\\\localhost\\system\\" ) + strlen( mappingName ) + 1, sizeof(char) );
  sprintf( variablePathName, "\\\\localhost\\system\\%s", mappingName );
  
  if( ( flags & SHM_READ ) )
  {
    int status = CNVCreateSubscriber( variablePathName, UpdateDataIn, NULL, newSharedObject, 10000, 0, &(newSharedObject->reader) ); 
    if( status != 0 )
    {
      DEBUG_PRINT( "%s", CNVGetErrorDescription( status ) );
      free( variablePathName );
      return NULL;
    }
    
    DEBUG_PRINT( "created subscriber %s with pointer %p", variablePathName, newSharedObject->reader );
  }
  
  if( ( flags & SHM_WRITE ) )
  {
    int status = CNVCreateWriter( variablePathName, NULL, NULL, 10000, 0, &(newSharedObject->writer) );
    if( status != 0 )
    {
      DEBUG_PRINT( "%s", CNVGetErrorDescription( status ) );
      free( variablePathName );
      return NULL;
    }
    
    DEBUG_PRINT( "created writer %s with pointer %p", variablePathName, newSharedObject->writer );
    
    //if( (newSharedObject->timerID = NewAsyncTimer( 0.001, -1, 1, UpdateDataOut, newSharedObject )) < 0 )
    if( (newSharedObject->timerID = NewAsyncTimerWithPriority( 0.001, -1, 1, UpdateDataOut, newSharedObject, 0 )) < 0 )
    {
      DEBUG_PRINT( "error creating async timer: code: %d", newSharedObject->timerID );
      free( variablePathName );
      return NULL;
    }
    
    DEBUG_PRINT( "created timer %d", newSharedObject->timerID );
  }
  
  free( variablePathName );
  
  return newSharedObject->data;
}

void SharedObjects_DestroyObject( void* sharedObjectData )
{
  for( khint_t sharedObjectID = 0; sharedObjectID != kh_end( sharedObjectsList ); sharedObjectID++ )
  {
    if( !kh_exist( sharedObjectsList, sharedObjectID ) ) continue;
    
    if( kh_value( sharedObjectsList, sharedObjectID ).data == sharedObjectData )
    {
      DEBUG_PRINT( "destroying shared object %d", sharedObjectID );
      
      SharedObject* sharedObject = &(kh_value( sharedObjectsList, sharedObjectID ));
      
      DEBUG_PRINT( "discarding timer %d", sharedObject->timerID );
      DiscardAsyncTimer( sharedObject->timerID );
      
      if( sharedObject->reader ) CNVDispose( sharedObject->reader );
	    if( sharedObject->writer ) CNVDispose( sharedObject->writer );
      
      DEBUG_PRINT( "disposing data %p", sharedObject->data );
      CNVDisposeData( sharedObject->networkData );
      free( sharedObject->data );
      
      kh_del( SO, sharedObjectsList, sharedObjectID );
      
      if( kh_size( sharedObjectsList ) == 0 )
      {
        CNVFinish();
        
        kh_destroy( SO, sharedObjectsList );
        sharedObjectsList = NULL;
      }
      
      break;
    }
  }
}


void CVICALLBACK UpdateDataIn( void* handle, CNVData data, void* callbackData )
{
  SharedObject* sharedObject = (SharedObject*) callbackData;
  
  if( sharedObject->reader == (CNVSubscriber) handle )
  {
    DEBUG_UPDATE( "received data %p on subscriber %p", data, handle );
    
    int status = CNVGetArrayDataValue( data, CNVUInt8, (void*) sharedObject->data, sharedObject->dataSize );
    if( status != 0 ) DEBUG_PRINT( "CNVGetArrayDataValue: %s", CNVGetErrorDescription( status ) );
    
    CNVDisposeData( data );
  }
}

int CVICALLBACK UpdateDataOut( int reserved, int timerId, int event, void* callbackData, int eventData1, int eventData2 )
{
  SharedObject* sharedObject = (SharedObject*) callbackData;
  
  if( sharedObject->timerID == timerId && event == EVENT_TIMER_TICK )
  {
    for( size_t byteIndex = 0; byteIndex < sharedObject->dataSize; byteIndex++ )
    {
      if( ( (uint8_t*) sharedObject->data )[ byteIndex ] != ( (uint8_t*) sharedObject->oldData )[ byteIndex ] )
      {
        int status = CNVCreateArrayDataValue( &(sharedObject->networkData), CNVUInt8, sharedObject->data, 1, &(sharedObject->dataSize) );
        if( status != 0 )
        {
          DEBUG_PRINT( "%s", CNVGetErrorDescription( status ) );
          return 0;
        }
        status = CNVWrite( sharedObject->writer, sharedObject->networkData, 100 );
        if( status != 0 )
        {
          DEBUG_PRINT( "%s", CNVGetErrorDescription( status ) );
          return 0;
        }
        
        DEBUG_UPDATE( "writing data %p on writer %p", sharedObject->networkData, sharedObject->writer );
        
        memcpy( sharedObject->oldData, sharedObject->data, sharedObject->dataSize );
        
        break;
      }
    }
  }
  
  return 0;
}

#endif // SHARED_MEMORY_H
