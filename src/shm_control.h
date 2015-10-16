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

const size_t SHM_CONTROL_MAX_FLOATS_NUMBER = 8;
const uint8_t SHM_CONTROL_BYTE_NULL = 0x00;

const bool SHM_CONTROL_PEEK = false;
const bool SHM_CONTROL_REMOVE = true;

#define SHM_CONTROL_BIT_INDEX( index ) ( 1 << index )
#define SHM_CONTROL_IS_BIT_SET( byte, index ) ( (byte) & SHM_CONTROL_BIT_INDEX( index ) )

typedef struct _ControlChannelData
{
  float numericValuesList[ SHM_CONTROL_MAX_FLOATS_NUMBER ];
  uint8_t numericValuesUpdatedList;
  uint8_t byteValue;
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
        function_init( bool, namespace, GetNumericValue, SHMController, size_t , float*, bool ) \
        function_init( bool, namespace, SetNumericValue, SHMController, size_t , float ) \
        function_init( uint8_t, namespace, GetNumericValuesList, SHMController, float*, bool ) \
        function_init( uint8_t, namespace, SetNumericValuesList, SHMController, float*, uint8_t ) \
        function_init( uint8_t, namespace, GetByteValue, SHMController, bool ) \
        function_init( bool, namespace, SetByteValue, SHMController, uint8_t )

INIT_NAMESPACE_INTERFACE( SHMControl, SHM_CONTROL_FUNCTIONS )


const size_t SHARED_VARIABLE_NAME_MAX_LENGTH = 128;
const char* SHARED_VARIABLE_IN_SUFFIX = "_in";
const char* SHARED_VARIABLE_OUT_SUFFIX = "_out";
SHMController SHMControl_InitData( const char* bufferName, enum SHMControlTypes controlType )
{
  char channelName[ SHARED_VARIABLE_NAME_MAX_LENGTH ];
  
  if( controlType < 0 || controlType >= SHM_CONTROL_TYPES_NUMBER ) 
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

bool SHMControl_GetNumericValue( SHMController controller, size_t  valueIndex, float* ref_value, bool remove )
{
  if( controller == NULL ) return false;
    
  if( valueIndex < 0 || valueIndex >= SHM_CONTROL_MAX_FLOATS_NUMBER ) return false;
    
  if( !( controller->channelIn->numericValuesUpdatedList & SHM_CONTROL_BIT_INDEX( valueIndex ) ) ) return false;
  
  if( ref_value != NULL )
  {
    *ref_value = controller->channelIn->numericValuesList[ valueIndex ];
    if( remove ) controller->channelIn->numericValuesUpdatedList ^= SHM_CONTROL_BIT_INDEX( valueIndex );
  }
    
  return true;
}

bool SHMControl_SetNumericValue( SHMController controller, size_t  valueIndex, float value )
{
  if( controller == NULL ) return false;
    
  if( valueIndex < 0 || valueIndex >= SHM_CONTROL_MAX_FLOATS_NUMBER ) return false;
  
  controller->channelOut->numericValuesList[ valueIndex ] = value;
  controller->channelOut->numericValuesUpdatedList |= SHM_CONTROL_BIT_INDEX( valueIndex );
  
  return true;
}

uint8_t SHMControl_GetNumericValuesList( SHMController controller, float* valuesList, bool remove )
{
  if( controller == NULL ) return 0;
  
  uint8_t mask = controller->channelIn->numericValuesUpdatedList;
  
  if( valuesList != NULL )
  {
    size_t dataSize = sizeof(float) * SHM_CONTROL_MAX_FLOATS_NUMBER;
    memcpy( valuesList, controller->channelIn->numericValuesList, dataSize );
    
    DEBUG_UPDATE( "got %x: p: %.3f - v: %.3f", mask, valuesList[ 0 ], valuesList[ 1 ] );
    
    if( remove ) controller->channelIn->numericValuesUpdatedList = 0x00;
  }
  
  return mask;
}

uint8_t SHMControl_SetNumericValuesList( SHMController controller, float* valuesList, uint8_t mask )
{
  if( controller == NULL ) return SHM_CONTROL_BYTE_NULL;
  
  if( valuesList == NULL ) return SHM_CONTROL_BYTE_NULL;
  
  size_t dataSize = sizeof(float) * SHM_CONTROL_MAX_FLOATS_NUMBER;
  memcpy( controller->channelOut->numericValuesList, valuesList, dataSize );
    
  controller->channelOut->numericValuesUpdatedList = mask;
    
  return controller->channelOut->numericValuesUpdatedList;
}

uint8_t SHMControl_GetByteValue( SHMController controller, bool remove )
{
  if( controller == NULL ) return SHM_CONTROL_BYTE_NULL;
  
  uint8_t value = controller->channelIn->byteValue;
  
  if( remove ) controller->channelIn->byteValue = SHM_CONTROL_BYTE_NULL;
  
  return value;
}

bool SHMControl_SetByteValue( SHMController controller, uint8_t value )
{
  if( controller == NULL ) return false;
  
  //DEBUG_PRINT( "byte value %u being set", value );
  
  controller->channelOut->byteValue = value;
  
  return true;
}

#endif // SHM_CONTROL_H
