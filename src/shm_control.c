#include <stdbool.h>
#include <stdint.h>

#include "debug/async_debug.h"

#include "shared_memory/shared_memory.h"

#include "shm_control.h"

typedef struct _ControlChannelData
{
  uint8_t dataMask[ SHM_CONTROL_MASK_SIZE ];
  uint8_t data[ SHM_CONTROL_MAX_DATA_SIZE ];
}
ControlChannelData;

typedef ControlChannelData* ControlChannel;

struct _SHMControlData
{
  ControlChannel channelIn;
  ControlChannel channelOut;
};


DEFINE_NAMESPACE_INTERFACE( SHMControl, SHM_CONTROL_INTERFACE )


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