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

typedef struct _AxisControlData
{
  float numericValuesTable[ SHM_CONTROL_TYPES_NUMBER ][ SHM_CONTROL_FLOATS_NUMBER ];
  uint8_t numericValuesUpdatedTable[ SHM_CONTROL_TYPES_NUMBER ];
  uint8_t byteValuesList[ SHM_CONTROL_TYPES_NUMBER ];
  uint8_t byteValuesUpdatedList;
}
AxisControlData;

typedef AxisControlData* SHMAxisController;

/*int InitControllerData( const char* );
void EndControllerData( int );
float* GetNumericValuesList( int, enum SHMControlNumericLists, bool, size_t* );
void SetNumericValues( int, enum SHMControlNumericLists, float* );
bool* GetByteValuesList( int, enum SHMControlByteLists, bool, size_t* );
void SetByteValues( int, enum SHMControlByteLists, bool* );

const struct
{
  int (*InitControllerData)( const char* );
  void (*EndControllerData)( int );
  float* (*GetNumericValuesList)( int, enum SHMControlNumericLists, bool, size_t* );
  void (*SetNumericValues)( int, enum SHMControlNumericLists, float* );
  bool* (*GetByteValuesList)( int, enum SHMControlByteLists, bool, size_t* );
  void (*SetByteValues)( int, enum SHMControlByteLists, bool* );
}
SHMAxisControl = { .InitControllerData = InitControllerData, .EndControllerData = EndControllerData,
                   .GetNumericValuesList = GetNumericValuesList, .SetNumericValues = SetNumericValues,
                   .GetByteValuesList = GetByteValuesList, .SetByteValues = SetByteValues };*/

/////////////////////////////////////////////////////////////////////////////
/////                             INTERFACE                             /////
/////////////////////////////////////////////////////////////////////////////

#define NAMESPACE SHMAxisControl

#define NAMESPACE_FUNCTIONS( FUNCTION_INIT, namespace ) \
        FUNCTION_INIT( SHMAxisController, namespace, InitController, const char* ) \
        FUNCTION_INIT( void, namespace, EndController, SHMAxisController ) \
        FUNCTION_INIT( bool, namespace, GetNumericValue, SHMAxisController, int, int, float*, bool ) \
        FUNCTION_INIT( void, namespace, SetNumericValue, SHMAxisController, int, int, float ) \
        FUNCTION_INIT( float*, namespace, GetNumericValuesList, SHMAxisController, int, uint8_t*, bool ) \
        FUNCTION_INIT( void, namespace, SetNumericValuesList, SHMAxisController, int, float*, uint8_t ) \
        FUNCTION_INIT( bool, namespace, GetByteValue, SHMAxisController, int, uint8_t*, bool ) \
        FUNCTION_INIT( void, namespace, SetByteValue, SHMAxisController, int, uint8_t )

NAMESPACE_FUNCTIONS( INIT_NAMESPACE_FILE, NAMESPACE )

const struct { NAMESPACE_FUNCTIONS( INIT_NAMESPACE_POINTER, NAMESPACE ) } NAMESPACE = { NAMESPACE_FUNCTIONS( INIT_NAMESPACE_STRUCT, NAMESPACE ) };

#undef NAMESPACE_FUNCTIONS
#undef NAMESPACE


SHMAxisController SHMAxisControl_InitController( const char* bufferName )
{
  DEBUG_PRINT( "creating axis control shared memory buffer %s", bufferName );
  
  SHMAxisController newController = (SHMAxisController) SharedObjects.CreateObject( bufferName, sizeof(AxisControlData), SHM_READ_WRITE );
  if( newController == (void*) -1 ) return NULL;
  
  return newController;
}

void SHMAxisControl_EndController( SHMAxisController controller )
{
  if( controller == NULL ) return;
  
  DEBUG_PRINT( "Destroying control %p data", controller );

  SharedObjects.DestroyObject( (void*) controller );
}

bool SHMAxisControl_GetNumericValue( SHMAxisController controller, int valueType, int valueIndex, float* ref_value, bool remove )
{
  if( controller == NULL ) return false;
    
  if( valueType < 0 || valueType >= SHM_CONTROL_TYPES_NUMBER ) return false;
    
  if( valueIndex < 0 || valueIndex >= SHM_CONTROL_FLOATS_NUMBER ) return false;
    
  if( !( controller->numericValuesUpdatedTable[ valueType ] & BIT_INDEX( valueIndex ) ) ) return false;
    
  if( remove ) controller->numericValuesUpdatedTable[ valueType ] ^= BIT_INDEX( valueIndex );
  
  if( ref_value != NULL ) *ref_value = controller->numericValuesTable[ valueType ][ valueIndex ];
    
  return true;
}

void SHMAxisControl_SetNumericValue( SHMAxisController controller, int valueType, int valueIndex, float value )
{
  if( controller == NULL ) return;
    
  if( valueType < 0 || valueType >= SHM_CONTROL_TYPES_NUMBER ) return;
    
  if( valueIndex < 0 || valueIndex >= SHM_CONTROL_FLOATS_NUMBER ) return;
  
  controller->numericValuesTable[ valueType ][ valueIndex ] = value;
  controller->numericValuesUpdatedTable[ valueType ] |= BIT_INDEX( valueIndex );
}

void SHMAxisControl_SetNumericValuesList( SHMAxisController controller, int valueType, float* numericValuesList, uint8_t mask )
{
  if( controller == NULL ) return;
    
  if( valueType < 0 || valueType >= SHM_CONTROL_TYPES_NUMBER ) return;
    
  if( numericValuesList != NULL ) 
  {
    size_t dataSize = sizeof(float) * SHM_CONTROL_FLOATS_NUMBER;
    memcpy( &(controller->numericValuesTable[ valueType ]), numericValuesList, dataSize );
  }
  
  controller->numericValuesUpdatedTable[ valueType ] = mask;
}

float* SHMAxisControl_GetNumericValuesList( SHMAxisController controller, int valueType, uint8_t* ref_mask, bool remove )
{
  if( controller == NULL ) return false;
    
  if( valueType < 0 || valueType >= SHM_CONTROL_TYPES_NUMBER ) return false;
    
  if( ref_mask != NULL ) *ref_mask = controller->numericValuesUpdatedTable[ valueType ];
  
  return (float*) controller->numericValuesTable[ valueType ];
}

bool SHMAxisControl_GetByteValue( SHMAxisController controller, int valueType, uint8_t* ref_value, bool remove )
{
  if( controller == NULL ) return false;
    
  if( valueType < 0 || valueType >= SHM_CONTROL_TYPES_NUMBER ) return false;
    
  if( !( controller->byteValuesUpdatedList & BIT_INDEX( valueType ) ) ) return false;
  
  DEBUG_PRINT( "Getting %s byte from axis control data (address: %p)", ( valueType == SHM_CONTROL_OUT ) ? "out" : "in", controller );
  
  if( remove ) controller->byteValuesUpdatedList ^= BIT_INDEX( valueType );
    
  if( ref_value != NULL ) *ref_value = controller->byteValuesList[ valueType ];
  
  return true;
}

void SHMAxisControl_SetByteValue( SHMAxisController controller, int valueType, uint8_t value )
{
  if( controller == NULL ) return;
    
  if( valueType < 0 || valueType >= SHM_CONTROL_TYPES_NUMBER ) return;
    
  DEBUG_PRINT( "Setting %s byte %u for axis control data (address: %p)", ( valueType == SHM_CONTROL_OUT ) ? "out" : "in", value, controller );
  
  controller->byteValuesList[ valueType ] = value;
  controller->byteValuesUpdatedList |= BIT_INDEX( valueType );
}

#endif // SHM_AXIS_CONTROL_H
