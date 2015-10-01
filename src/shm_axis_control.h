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

enum SHMControlBytes { SHM_COMMAND_DISABLE, SHM_COMMAND_ENABLE, SHM_COMMAND_RESET, SHM_COMMAND_CALIBRATE,
                       SHM_STATE_DISABLED = SHM_COMMAND_DISABLE, SHM_STATE_ENABLED, SHM_STATE_ERROR, SHM_CONTROL_BYTES_NUMBER,};

const bool SHM_PEEK = false;
const bool SHM_REMOVE = true;

#define FIRST_BIT_INDEX 128
#define BIT_INDEX( index ) ( FIRST_BIT_INDEX >> index )

typedef struct _SHMAxisControlData
{
  float numericValuesTable[ SHM_CONTROL_TYPES_NUMBER ][ SHM_CONTROL_FLOATS_NUMBER ];
  uint8_t numericValuesUpdatedTable[ SHM_CONTROL_TYPES_NUMBER ];
  uint8_t byteValuesList[ SHM_CONTROL_TYPES_NUMBER ];
  uint8_t byteValuesUpdatedList;
}
SHMAxisControlData;

typedef SHMAxisControlData* SHMAxis;

/////////////////////////////////////////////////////////////////////////////
/////                             INTERFACE                             /////
/////////////////////////////////////////////////////////////////////////////

#define SHM_AXIS_CONTROL_FUNCTIONS( namespace, function_init ) \
        function_init( SHMAxis, namespace, InitData, const char* ) \
        function_init( void, namespace, EndData, SHMAxis ) \
        function_init( bool, namespace, GetNumericValue, SHMAxis, int, int, float*, bool ) \
        function_init( void, namespace, SetNumericValue, SHMAxis, int, int, float ) \
        function_init( float*, namespace, GetNumericValuesList, SHMAxis, int, uint8_t*, bool ) \
        function_init( void, namespace, SetNumericValuesList, SHMAxis, int, float*, uint8_t ) \
        function_init( bool, namespace, GetByteValue, SHMAxis, int, uint8_t*, bool ) \
        function_init( void, namespace, SetByteValue, SHMAxis, int, uint8_t )

INIT_NAMESPACE_INTERFACE( SHMAxisControl, SHM_AXIS_CONTROL_FUNCTIONS )


SHMAxis SHMAxisControl_InitData( const char* bufferName )
{
  DEBUG_PRINT( "creating axis control shared memory buffer %s", bufferName );
  
  SHMAxis newAxis = (SHMAxis) SharedObjects.CreateObject( bufferName, sizeof(SHMAxisControlData), SHM_READ_WRITE );
  if( newAxis == (void*) -1 ) return NULL;
  
  return newAxis;
}

void SHMAxisControl_EndData( SHMAxis axis )
{
  if( axis == NULL ) return;
  
  DEBUG_PRINT( "Destroying control %p data", axis );

  SharedObjects.DestroyObject( (void*) axis );
}

bool SHMAxisControl_GetNumericValue( SHMAxis axis, int valueType, int valueIndex, float* ref_value, bool remove )
{
  if( axis == NULL ) return false;
    
  if( valueType < 0 || valueType >= SHM_CONTROL_TYPES_NUMBER ) return false;
    
  if( valueIndex < 0 || valueIndex >= SHM_CONTROL_FLOATS_NUMBER ) return false;
    
  if( !( axis->numericValuesUpdatedTable[ valueType ] & BIT_INDEX( valueIndex ) ) ) return false;
    
  if( remove ) axis->numericValuesUpdatedTable[ valueType ] ^= BIT_INDEX( valueIndex );
  
  if( ref_value != NULL ) *ref_value = axis->numericValuesTable[ valueType ][ valueIndex ];
    
  return true;
}

void SHMAxisControl_SetNumericValue( SHMAxis axis, int valueType, int valueIndex, float value )
{
  if( axis == NULL ) return;
    
  if( valueType < 0 || valueType >= SHM_CONTROL_TYPES_NUMBER ) return;
    
  if( valueIndex < 0 || valueIndex >= SHM_CONTROL_FLOATS_NUMBER ) return;
  
  axis->numericValuesTable[ valueType ][ valueIndex ] = value;
  axis->numericValuesUpdatedTable[ valueType ] |= BIT_INDEX( valueIndex );
}

void SHMAxisControl_SetNumericValuesList( SHMAxis axis, int valueType, float* numericValuesList, uint8_t mask )
{
  if( axis == NULL ) return;
    
  if( valueType < 0 || valueType >= SHM_CONTROL_TYPES_NUMBER ) return;
    
  if( numericValuesList != NULL ) 
  {
    size_t dataSize = sizeof(float) * SHM_CONTROL_FLOATS_NUMBER;
    memcpy( &(axis->numericValuesTable[ valueType ]), numericValuesList, dataSize );
  }
  
  axis->numericValuesUpdatedTable[ valueType ] = mask;
}

float* SHMAxisControl_GetNumericValuesList( SHMAxis axis, int valueType, uint8_t* ref_mask, bool remove )
{
  if( axis == NULL ) return false;
    
  if( valueType < 0 || valueType >= SHM_CONTROL_TYPES_NUMBER ) return false;
    
  if( ref_mask != NULL ) *ref_mask = axis->numericValuesUpdatedTable[ valueType ];
  
  return (float*) axis->numericValuesTable[ valueType ];
}

bool SHMAxisControl_GetByteValue( SHMAxis axis, int valueType, uint8_t* ref_value, bool remove )
{
  if( axis == NULL ) return false;
    
  if( valueType < 0 || valueType >= SHM_CONTROL_TYPES_NUMBER ) return false;
    
  if( !( axis->byteValuesUpdatedList & BIT_INDEX( valueType ) ) ) return false;
  
  DEBUG_PRINT( "Getting %s byte from axis control data (address: %p)", ( valueType == SHM_CONTROL_OUT ) ? "out" : "in", axis );
  
  if( remove ) axis->byteValuesUpdatedList ^= BIT_INDEX( valueType );
    
  if( ref_value != NULL ) *ref_value = axis->byteValuesList[ valueType ];
  
  return true;
}

void SHMAxisControl_SetByteValue( SHMAxis axis, int valueType, uint8_t value )
{
  if( axis == NULL ) return;
    
  if( valueType < 0 || valueType >= SHM_CONTROL_TYPES_NUMBER ) return;
    
  DEBUG_PRINT( "Setting %s byte %u for axis control data (address: %p)", ( valueType == SHM_CONTROL_OUT ) ? "out" : "in", value, axis );
  
  axis->byteValuesList[ valueType ] = value;
  axis->byteValuesUpdatedList |= BIT_INDEX( valueType );
}

#endif // SHM_AXIS_CONTROL_H
