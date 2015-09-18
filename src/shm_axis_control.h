#ifndef SHM_AXIS_CONTROL_H
#define SHM_AXIS_CONTROL_H

#include <stdbool.h>

#ifdef _CVI_
  #include "shared_memory/shared_memory_cvi.h"
#elif WIN32
  #include "shared_memory/shared_memory_windows.h"
#else
  #include "shared_memory/shared_memory_unix.h"
#endif

#include "klib/khash.h"

#include "debug/async_debug.h"

enum SHMControlNumericLists { SHM_CONTROL_MEASURES, SHM_CONTROL_PARAMETERS, SHM_CONTROL_NUMERIC_LISTS_NUMBER };
enum SHMControlBooleanLists { SHM_CONTROL_STATES, SHM_CONTROL_COMMANDS, SHM_CONTROL_BOOLEAN_LISTS_NUMBER };

enum SHMControlMeasures { SHM_CONTROL_POSITION, SHM_CONTROL_VELOCITY, SHM_CONTROL_ACCELERATION, SHM_CONTROL_FORCE, SHM_CONTROL_MEASURES_NUMBER };
enum SHMControlParameters { SHM_CONTROL_REF_POSITION, SHM_CONTROL_REF_VELOCITY, SHM_CONTROL_STIFFNESS, SHM_CONTROL_DAMPING, SHM_CONTROL_PARAMETERS_NUMBER };
enum SHMControlStates { SHM_CONTROL_ENABLED, SHM_CONTROL_HAS_ERROR, SHM_CONTROL_STATES_NUMBER };
enum SHMControlCommands { SHM_CONTROL_ENABLE, SHM_CONTROL_DISABLE, SHM_CONTROL_RESET, SHM_CONTROL_CALIBRATE, SHM_CONTROL_COMMANDS_NUMBER };

static const size_t FLOAT_VALUES_NUMBER_LIST[ SHM_CONTROL_NUMERIC_LISTS_NUMBER ] = { SHM_CONTROL_MEASURES_NUMBER, SHM_CONTROL_PARAMETERS_NUMBER };
static const size_t BOOL_VALUES_NUMBER_LIST[ SHM_CONTROL_BOOLEAN_LISTS_NUMBER ] = { SHM_CONTROL_STATES_NUMBER, SHM_CONTROL_COMMANDS_NUMBER }; 

const bool SHM_PEEK = false;
const bool SHM_REMOVE = true;

typedef struct _AxisControlData
{
  // control -> user
  float measuresList[ SHM_CONTROL_MEASURES_NUMBER ];
  bool statesList[ SHM_CONTROL_STATES_NUMBER ];
  // user -> control
  float parametersList[ SHM_CONTROL_PARAMETERS_NUMBER ];
  bool commandsList[ SHM_CONTROL_COMMANDS_NUMBER ];
  // indirect access
  float* numericValuesTable[ SHM_CONTROL_NUMERIC_LISTS_NUMBER ];
  bool numericValuesUpdatedList[ SHM_CONTROL_NUMERIC_LISTS_NUMBER ];
  bool* booleanValuesTable[ SHM_CONTROL_BOOLEAN_LISTS_NUMBER ];
  bool booleanValuesUpdatedList[ SHM_CONTROL_BOOLEAN_LISTS_NUMBER ];
}
AxisControlData;

KHASH_MAP_INIT_STR( AxisControl, AxisControlData* )
static khash_t( AxisControl )* controlsDataList = NULL;

int InitControllerData( const char* );
void EndControllerData( int );
float* GetNumericValuesList( int, enum SHMControlNumericLists, bool, size_t* );
void SetNumericValues( int, enum SHMControlNumericLists, float* );
bool* GetBooleanValuesList( int, enum SHMControlBooleanLists, bool, size_t* );
void SetBooleanValues( int, enum SHMControlBooleanLists, bool* );

const struct
{
  int (*InitControllerData)( const char* );
  void (*EndControllerData)( int );
  float* (*GetNumericValuesList)( int, enum SHMControlNumericLists, bool, size_t* );
  void (*SetNumericValues)( int, enum SHMControlNumericLists, float* );
  bool* (*GetBooleanValuesList)( int, enum SHMControlBooleanLists, bool, size_t* );
  void (*SetBooleanValues)( int, enum SHMControlBooleanLists, bool* );
}
SHMAxisControl = { .InitControllerData = InitControllerData, .EndControllerData = EndControllerData,
                   .GetNumericValuesList = GetNumericValuesList, .SetNumericValues = SetNumericValues,
                   .GetBooleanValuesList = GetBooleanValuesList, .SetBooleanValues = SetBooleanValues };

int InitControllerData( const char* sharedMemoryKey )
{
  if( controlsDataList == NULL ) controlsDataList = kh_init( AxisControl );
  
  DEBUG_PRINT( "Trying to create axis shared memory area with key %x", sharedMemoryKey );
  
  AxisControlData* newShmControlData = SharedObjects.CreateObject( sharedMemoryKey, sizeof(AxisControlData), SHM_READ_WRITE );
  if( newShmControlData == (void*) -1 ) return -1;
  
  int insertionStatus;
  khint_t newShmControllerID = kh_put( AxisControl, controlsDataList, sharedMemoryKey, &insertionStatus );
  if( insertionStatus == -1 ) return -1;
  
  memset( newShmControlData, 0, sizeof(AxisControlData) );
  
  newShmControlData->numericValuesTable[ SHM_CONTROL_MEASURES ] = (float*) &(newShmControlData->measuresList);
  newShmControlData->numericValuesTable[ SHM_CONTROL_PARAMETERS ] = (float*) &(newShmControlData->parametersList);
  
  newShmControlData->booleanValuesTable[ SHM_CONTROL_STATES ] = (bool*) &(newShmControlData->statesList);
  newShmControlData->booleanValuesTable[ SHM_CONTROL_COMMANDS ] = (bool*) &(newShmControlData->commandsList);
  
  kh_value( controlsDataList, newShmControllerID ) = newShmControlData;
  
  return (int) newShmControllerID;
}

void EndControllerData( int shmControlDataID )
{
  if( !kh_exist( controlsDataList, (khint_t) shmControlDataID ) ) return;
  
  DEBUG_PRINT( "Destroying control %d data", shmControlDataID );

  SharedObjects.DestroyObject( kh_value( controlsDataList, (khint_t) shmControlDataID ) );
    
  kh_del( AxisControl, controlsDataList, (khint_t) shmControlDataID );
    
  if( kh_size( controlsDataList ) == 0 )
  {
    kh_destroy( AxisControl, controlsDataList );
    controlsDataList = NULL;
  }
}

float* GetNumericValuesList( int shmControlDataID, enum SHMControlNumericLists listIndex, bool remove, size_t* ref_valuesNumber )
{
  if( !kh_exist( controlsDataList, (khint_t) shmControlDataID ) ) return NULL;
    
  if( listIndex < 0 || listIndex >= SHM_CONTROL_NUMERIC_LISTS_NUMBER ) return NULL;
    
  AxisControlData* controlData = kh_value( controlsDataList, (khint_t) shmControlDataID );
    
  if( !controlData->numericValuesUpdatedList[ listIndex ] ) return NULL;
    
  if( remove ) controlData->numericValuesUpdatedList[ listIndex ] = false;
  
  if( ref_valuesNumber != NULL ) *ref_valuesNumber = FLOAT_VALUES_NUMBER_LIST[ listIndex ];
    
  return controlData->numericValuesTable[ listIndex ];
}

void SetNumericValues( int shmControlDataID, enum SHMControlNumericLists listIndex, float* numericValuesList )
{
  if( !kh_exist( controlsDataList, (khint_t) shmControlDataID ) ) return;
  
  if( listIndex < 0 || listIndex >= SHM_CONTROL_NUMERIC_LISTS_NUMBER ) return;
  
  AxisControlData* controlData = kh_value( controlsDataList, (khint_t) shmControlDataID );
    
  if( numericValuesList != NULL ) 
  {
    size_t dataSize = sizeof(float) * FLOAT_VALUES_NUMBER_LIST[ listIndex ];
    memcpy( controlData->numericValuesTable[ listIndex ], numericValuesList, dataSize );
  }
    
  controlData->numericValuesUpdatedList[ listIndex ] = true;
}

bool* GetBooleanValuesList( int shmControlDataID, enum SHMControlBooleanLists listIndex, bool remove, size_t* ref_valuesNumber )
{
  if( !kh_exist( controlsDataList, (khint_t) shmControlDataID ) ) return NULL;
    
  if( listIndex < 0 || listIndex >= SHM_CONTROL_BOOLEAN_LISTS_NUMBER ) return NULL;
  
  AxisControlData* controlData = kh_value( controlsDataList, (khint_t) shmControlDataID );
    
  if( !controlData->booleanValuesUpdatedList[ listIndex ] ) return NULL;
  
  DEBUG_PRINT( "Getting states for axis data ID %u (address: %p)", (khint_t) shmControlDataID, controlData );
  
  if( remove ) controlData->booleanValuesUpdatedList[ listIndex ] = false;
  
  if( ref_valuesNumber != NULL ) *ref_valuesNumber = BOOL_VALUES_NUMBER_LIST[ listIndex ];
    
  return controlData->booleanValuesTable[ listIndex ];
}

void SetBooleanValues( int shmControlDataID, enum SHMControlBooleanLists listIndex, bool* booleanValuesList )
{
  if( !kh_exist( controlsDataList, (khint_t) shmControlDataID ) ) return;
  
  if( listIndex < 0 || listIndex >= SHM_CONTROL_BOOLEAN_LISTS_NUMBER ) return;
  
  AxisControlData* controlData = kh_value( controlsDataList, (khint_t) shmControlDataID );
    
  DEBUG_PRINT( "Setting states for axis data ID %u (address: %p)", (khint_t) shmControlDataID, controlData );
    
  if( booleanValuesList != NULL ) 
  {
    size_t dataSize = sizeof(bool) * BOOL_VALUES_NUMBER_LIST[ listIndex ];
    memcpy( controlData->booleanValuesTable[ listIndex ], booleanValuesList, dataSize );
  }
    
  controlData->booleanValuesUpdatedList[ listIndex ] = true;
}

#endif // SHM_AXIS_CONTROL_H
