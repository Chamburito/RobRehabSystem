#ifndef SHM_CONTROL_H
#define SHM_CONTROL_H

#ifdef _CVI_
  #include "shared_memory/shm_cvi.h"
#elif WIN32
  #include "shared_memory/shm_windows.h"
#else
  #include "shared_memory/shm_unix.h"
#endif

#include "debug/async_debug.h"

#include <stdbool.h>
#include <stdint.h>

enum SHMControlTypes { SHM_CONTROL_OUT, SHM_CONTROL_IN, SHM_CONTROL_TYPES_NUMBER };

#define SHM_CONTROL_MASK_SIZE 32
#define SHM_CONTROL_MAX_DATA_SIZE ( 8 * SHM_CONTROL_MASK_SIZE )

//const bool SHM_CONTROL_PEEK = false;
//const bool SHM_CONTROL_REMOVE = true;

#define SHM_CONTROL_BIT_INDEX( index ) ( 1 << (index) )
#define SHM_CONTROL_SET_BIT( field, index ) ( (field) |= SHM_CONTROL_BIT_INDEX( index ) )
#define SHM_CONTROL_CLR_BIT( field, index ) ( (field) &= (~SHM_CONTROL_BIT_INDEX( index )) )
#define SHM_CONTROL_IS_BIT_SET( field, index ) ( (field) & SHM_CONTROL_BIT_INDEX( index ) )

//#define SHM_CONTROL_SET_BYTE( field, index, byte ) ( (field) |= ( ( (byte) << index * 8 ) ) )
//#define SHM_CONTROL_GET_BYTE( field, index ) ( ( (field) >> index * 8 ) & 0xFF )

typedef struct _ControlChannelData
{
  uint8_t dataMask[ SHM_CONTROL_MASK_SIZE ];
  uint8_t data[ SHM_CONTROL_MAX_DATA_SIZE ];
}
ControlChannelData;

typedef ControlChannelData* ControlChannel;

typedef struct _SHMControlData
{
  ControlChannel channelIn;
  ControlChannel channelOut;
}
SHMControlData;

typedef SHMControlData* SHMController;


/////////////////////////////////////////////////////////////////////////////
/////                             INTERFACE                             /////
/////////////////////////////////////////////////////////////////////////////

#define SHM_CONTROL_FUNCTIONS( namespace, function_init ) \
        function_init( SHMController, namespace, InitData, const char*, enum SHMControlTypes ) \
        function_init( void, namespace, EndData, SHMController ) \
        function_init( bool, namespace, GetData, SHMController, void*, size_t, size_t ) \
        function_init( bool, namespace, SetData, SHMController, void*, size_t, size_t ) \
        function_init( uint8_t, namespace, GetMaskByte, SHMController, size_t ) \
        function_init( uint8_t, namespace, SetMaskByte, SHMController, size_t, uint8_t )

INIT_NAMESPACE_INTERFACE( SHMControl, SHM_CONTROL_FUNCTIONS )


#define SHARED_VARIABLE_NAME_MAX_LENGTH 128
const char* SHARED_VARIABLE_IN_SUFFIX = "_in";
const char* SHARED_VARIABLE_OUT_SUFFIX = "_out";
SHMController SHMControl_InitData( const char* bufferName, enum SHMControlTypes controlType )
{
  char channelName[ SHARED_VARIABLE_NAME_MAX_LENGTH ];
  
  if( controlType >= SHM_CONTROL_TYPES_NUMBER ) 
  {
    DEBUG_PRINT( "invalid control type: %d", controlType );
    return NULL;
  }
  
  DEBUG_PRINT( "creating control shared memory buffer %s", bufferName );
  
  SHMController newController = (SHMController) malloc( sizeof(SHMControlData) );
  memset( newController, 0, sizeof(SHMControlData) );
  
  sprintf( channelName, "%s%s", bufferName, ( controlType == SHM_CONTROL_IN ) ? SHARED_VARIABLE_IN_SUFFIX : SHARED_VARIABLE_OUT_SUFFIX );
  newController->channelIn = (ControlChannel) SharedObjects.CreateObject( channelName, sizeof(ControlChannelData), SHM_READ );
  if( newController->channelIn == (void*) -1 ) return NULL;
  
  sprintf( channelName, "%s%s", bufferName, ( controlType == SHM_CONTROL_IN ) ? SHARED_VARIABLE_OUT_SUFFIX : SHARED_VARIABLE_IN_SUFFIX );
  newController->channelOut = (ControlChannel) SharedObjects.CreateObject( channelName, sizeof(ControlChannelData), SHM_WRITE );
  if( newController->channelOut == (void*) -1 )
  {
    SharedObjects.DestroyObject( (void*) newController->channelOut );
    return NULL;
  }
  
  DEBUG_PRINT( "control %s configuration: %p %p", ( controlType == SHM_CONTROL_IN ) ? "IN" : "OUT", newController->channelIn, newController->channelOut );
  
  return newController;
}

void SHMControl_EndData( SHMController controller )
{
  if( controller == NULL ) return;
  
  DEBUG_PRINT( "Destroying controller %p data", controller );

  SharedObjects.DestroyObject( (void*) controller->channelIn );
  SharedObjects.DestroyObject( (void*) controller->channelOut );
  
  free( controller );
}

bool SHMControl_GetData( SHMController controller, void* valuesList, size_t dataOffset, size_t dataLength )
{
  if( controller == NULL ) return false;
  
  if( controller->channelIn == NULL ) return false;
  
  if( dataOffset + dataLength > SHM_CONTROL_MAX_DATA_SIZE ) return false;
  
  if( valuesList == NULL ) return false;
    
  memcpy( valuesList, controller->channelIn->data + dataOffset, dataLength );
  
  return true;
}

bool SHMControl_SetData( SHMController controller, void* valuesList, size_t dataOffset, size_t dataLength )
{
  if( controller == NULL ) return false;
  
  if( controller->channelOut == NULL ) return false;
  
  if( dataOffset + dataLength > SHM_CONTROL_MAX_DATA_SIZE ) return false;
  
  if( valuesList == NULL ) return false;
  
  memcpy( controller->channelOut->data + dataOffset, valuesList, dataLength );
    
  return true;
}

uint8_t SHMControl_GetMaskByte( SHMController controller, size_t maskByteIndex )
{
  if( controller == NULL ) return 0;
  
  if( controller->channelIn == NULL ) return 0;
  
  if( maskByteIndex >= SHM_CONTROL_MASK_SIZE ) return 0;
  
  return controller->channelIn->dataMask[ maskByteIndex ];
}

uint8_t SHMControl_SetMaskByte( SHMController controller, size_t maskByteIndex, uint8_t maskByteValue )
{
  if( controller == NULL ) return 0;
  
  if( controller->channelOut == NULL ) return 0;
  
  if( maskByteIndex >= SHM_CONTROL_MASK_SIZE ) return 0;
    
  controller->channelOut->dataMask[ maskByteIndex ] = maskByteValue;
    
  return controller->channelOut->dataMask[ maskByteIndex ];
}

#endif // SHM_CONTROL_H
