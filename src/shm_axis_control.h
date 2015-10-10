#ifndef SHM_AXIS_CONTROL_H
#define SHM_AXIS_CONTROL_H

#ifdef _CVI_
  #include "shared_memory/shared_memory_cvi.h"
#elif WIN32
  #include "shared_memory/shared_memory_windows.h"
#else
  #include "shared_memory/shared_memory_unix.h"
#endif

#include "debug/async_debug.h"

#include <stdbool.h>
#include <stdint.h>

enum SHMControlTypes { SHM_CONTROL_OUT, SHM_CONTROL_IN, SHM_CONTROL_TYPES_NUMBER };

enum SHMControlFloats { SHM_CONTROL_POSITION, SHM_CONTROL_VELOCITY, SHM_CONTROL_FORCE, SHM_CONTROL_STIFFNESS = SHM_CONTROL_FORCE,
                        SHM_CONTROL_ACCELERATION, SHM_CONTROL_DAMPING = SHM_CONTROL_ACCELERATION, SHM_CONTROL_TIME, SHM_CONTROL_FLOATS_NUMBER };

enum SHMControlBytes { SHM_COMMAND_DISABLE, SHM_STATE_DISABLED = SHM_COMMAND_DISABLE, SHM_COMMAND_ENABLE, SHM_STATE_ENABLED = SHM_COMMAND_ENABLE, 
                       SHM_COMMAND_RESET, SHM_STATE_ERROR = SHM_COMMAND_RESET, SHM_COMMAND_CALIBRATE, SHM_CONTROL_BYTES_NUMBER };

const bool SHM_PEEK = false;
const bool SHM_REMOVE = true;

#define FIRST_BIT_INDEX 128
#define BIT_INDEX( index ) ( FIRST_BIT_INDEX >> index )

typedef struct _ControlChannelData
{
  float numericValuesList[ SHM_CONTROL_FLOATS_NUMBER ];
  uint8_t numericValuesUpdatedList;
  uint8_t byteValue;
  bool byteValueUpdated;
}
ControlChannelData;

typedef ControlChannelData* ControlChannel;

typedef struct _SHMAxisControlData
{
  ControlChannel channelIn;
  ControlChannel channelOut;
}
SHMAxisControlData;

typedef SHMAxisControlData* SHMAxis;

/////////////////////////////////////////////////////////////////////////////
/////                             INTERFACE                             /////
/////////////////////////////////////////////////////////////////////////////

#define SHM_AXIS_CONTROL_FUNCTIONS( namespace, function_init ) \
        function_init( SHMAxis, namespace, InitData, const char*, enum SHMControlTypes ) \
        function_init( void, namespace, EndData, SHMAxis ) \
        function_init( bool, namespace, GetNumericValue, SHMAxis, enum SHMControlFloats, float*, bool ) \
        function_init( void, namespace, SetNumericValue, SHMAxis, enum SHMControlFloats, float ) \
        function_init( uint8_t, namespace, GetNumericValuesList, SHMAxis, float*, bool ) \
        function_init( void, namespace, SetNumericValuesList, SHMAxis, float*, uint8_t ) \
        function_init( bool, namespace, GetByteValue, SHMAxis, uint8_t*, bool ) \
        function_init( void, namespace, SetByteValue, SHMAxis, uint8_t )

INIT_NAMESPACE_INTERFACE( SHMAxisControl, SHM_AXIS_CONTROL_FUNCTIONS )


const size_t SHARED_VARIABLE_NAME_MAX_LEN = 128;
const char* SHARED_VARIABLE_IN_SUFFIX = "_in";
const char* SHARED_VARIABLE_OUT_SUFFIX = "_out";
SHMAxis SHMAxisControl_InitData( const char* bufferName, enum SHMControlTypes controlType )
{
  char channelName[ SHARED_VARIABLE_NAME_MAX_LEN ];
  
  if( controlType < 0 || controlType >= SHM_CONTROL_TYPES_NUMBER ) 
  {
    DEBUG_PRINT( "invalid control type: %d", controlType );
    return NULL;
  }
  
  DEBUG_PRINT( "creating axis control shared memory buffer %s", bufferName );
  
  SHMAxis newAxis = (SHMAxis) malloc( sizeof(SHMAxisControlData) );
  memset( newAxis, 0, sizeof(SHMAxisControlData) );
  
  sprintf( channelName, "%s%s", bufferName, ( controlType == SHM_CONTROL_IN ) ? SHARED_VARIABLE_IN_SUFFIX : SHARED_VARIABLE_OUT_SUFFIX );
  newAxis->channelIn = (ControlChannel) SharedObjects.CreateObject( channelName, sizeof(ControlChannelData), SHM_READ );
  if( newAxis->channelIn == (void*) -1 ) return NULL;
  
  sprintf( channelName, "%s%s", bufferName, ( controlType == SHM_CONTROL_IN ) ? SHARED_VARIABLE_OUT_SUFFIX : SHARED_VARIABLE_IN_SUFFIX );
  newAxis->channelOut = (ControlChannel) SharedObjects.CreateObject( channelName, sizeof(ControlChannelData), SHM_WRITE );
  if( newAxis->channelOut == (void*) -1 )
  {
    SharedObjects.DestroyObject( (void*) newAxis->channelOut );
    return NULL;
  }
  
  DEBUG_PRINT( "control %s configuration: %p %p", ( controlType == SHM_CONTROL_IN ) ? "IN" : "OUT", newAxis->channelIn, newAxis->channelOut );
  
  return newAxis;
}

void SHMAxisControl_EndData( SHMAxis axis )
{
  if( axis == NULL ) return;
  
  DEBUG_PRINT( "Destroying control %p data", axis );

  SharedObjects.DestroyObject( (void*) axis->channelIn );
  SharedObjects.DestroyObject( (void*) axis->channelOut );
  
  free( axis );
}

bool SHMAxisControl_GetNumericValue( SHMAxis axis, enum SHMControlFloats valueIndex, float* ref_value, bool remove )
{
  if( axis == NULL ) return false;
    
  if( valueIndex < 0 || valueIndex >= SHM_CONTROL_FLOATS_NUMBER ) return false;
    
  if( !( axis->channelIn->numericValuesUpdatedList & BIT_INDEX( valueIndex ) ) ) return false;
  
  if( ref_value != NULL )
  {
    *ref_value = axis->channelIn->numericValuesList[ valueIndex ];
    if( remove ) axis->channelIn->numericValuesUpdatedList ^= BIT_INDEX( valueIndex );
  }
    
  return true;
}

void SHMAxisControl_SetNumericValue( SHMAxis axis, enum SHMControlFloats valueIndex, float value )
{
  if( axis == NULL ) return;
    
  if( valueIndex < 0 || valueIndex >= SHM_CONTROL_FLOATS_NUMBER ) return;
  
  axis->channelOut->numericValuesList[ valueIndex ] = value;
  axis->channelOut->numericValuesUpdatedList |= BIT_INDEX( valueIndex );
}

uint8_t SHMAxisControl_GetNumericValuesList( SHMAxis axis, float* valuesList, bool remove )
{
  if( axis == NULL ) return 0;
  
  uint8_t mask = axis->channelIn->numericValuesUpdatedList;
  
  if( valuesList != NULL )
  {
    size_t dataSize = sizeof(float) * SHM_CONTROL_FLOATS_NUMBER;
    memcpy( valuesList, axis->channelIn->numericValuesList, dataSize );
    
    DEBUG_UPDATE( "got %x: p: %.3f - v: %.3f", mask, valuesList[ SHM_CONTROL_POSITION ], valuesList[ SHM_CONTROL_VELOCITY ] );
    
    if( remove ) axis->channelIn->numericValuesUpdatedList = 0x00;
  }
  
  return mask;
}

void SHMAxisControl_SetNumericValuesList( SHMAxis axis, float* valuesList, uint8_t mask )
{
  if( axis == NULL ) return;
  
  if( valuesList != NULL ) 
  {
    size_t dataSize = sizeof(float) * SHM_CONTROL_FLOATS_NUMBER;
    memcpy( axis->channelOut->numericValuesList, valuesList, dataSize );
    
    axis->channelOut->numericValuesUpdatedList = mask;
  }
}

bool SHMAxisControl_GetByteValue( SHMAxis axis, uint8_t* ref_value, bool remove )
{
  if( axis == NULL ) return false;
    
  //DEBUG_PRINT( "new byte ?: %u", axis->channelIn->byteValueUpdated );
  
  if( !( axis->channelIn->byteValueUpdated ) ) return false;
  
  if( ref_value != NULL )
  {
    *ref_value = axis->channelIn->byteValue;
    if( remove ) axis->channelIn->byteValueUpdated = false;
  }
  
  return true;
}

void SHMAxisControl_SetByteValue( SHMAxis axis, uint8_t value )
{
  if( axis == NULL ) return;
  
  //DEBUG_PRINT( "byte value %u being set", value );
  
  axis->channelOut->byteValue = value;
  axis->channelOut->byteValueUpdated = true;
}

#endif // SHM_AXIS_CONTROL_H
