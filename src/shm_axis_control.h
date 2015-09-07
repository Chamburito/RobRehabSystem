#ifndef SHM_AXIS_CONTROL_H
#define SHM_AXIS_CONTROL_H

#include <stdbool.h>

#ifdef WIN32
  #include "shared_memory/shared_memory_windows.h"
#else
  #include "shared_memory/shared_memory_unix.h"
#endif

#include "klib/khash.h"

#include "impedance_control.h"

#include "debug/async_debug.h"

enum ControlNumericLists { CONTROL_MEASURES, CONTROL_PARAMETERS, CONTROL_NUMERIC_LISTS_NUMBER };
enum ControlBooleanLists { CONTROL_STATES, CONTROL_COMMANDS, CONTROL_BOOLEAN_LISTS_NUMBER };

enum ControlStates { CONTROL_ENABLED, CONTROL_HAS_ERROR, CONTROL_STATES_NUMBER };
enum ControlCommands { CONTROL_ENABLE, CONTROL_RESET, CONTROL_COMMANDS_NUMBER };

const bool PEEK = false;
const bool REMOVE = true;

typedef struct _AxisControlData
{
  // control -> user
  double measuresList[ CONTROL_MEASURES_NUMBER ];
  bool statesList[ CONTROL_STATES_NUMBER ];
  // user -> control
  double parametersList[ CONTROL_SETPOINTS_NUMBER ];
  bool commandsList[ CONTROL_COMMANDS_NUMBER ];
  // indirect access
  double* numericValuesTable[ CONTROL_NUMERIC_LISTS_NUMBER ];
  bool numericValuesUpdatedList[ CONTROL_NUMERIC_LISTS_NUMBER ];
  size_t numericValuesNumberList [ CONTROL_NUMERIC_LISTS_NUMBER ];
  bool* booleanValuesTable[ CONTROL_BOOLEAN_LISTS_NUMBER ];
  bool booleanValuesUpdatedList[ CONTROL_BOOLEAN_LISTS_NUMBER ];
  size_t booleanValuesNumberList[ CONTROL_BOOLEAN_LISTS_NUMBER ];
}
AxisControlData;

KHASH_MAP_INIT_INT( AxisControl, AxisControlData* )
static khash_t( AxisControl )* controlsDataList = NULL;

int InitControllerData( const char* );
void EndControllerData( int );
double* GetNumericValuesList( int, enum ControlNumericLists, bool, size_t* );
void SetNumericValues( int, enum ControlNumericLists, double* );
bool* GetBooleanValuesList( int, enum ControlBooleanLists, bool, size_t* );
void SetBooleanValues( int, enum ControlBooleanLists, bool* );

const struct
{
  int (*InitControllerData)( const char* );
  void (*EndControllerData)( int );
  double* (*GetNumericValuesList)( int, enum ControlNumericLists, bool, size_t* );
  void (*SetNumericValues)( int, enum ControlNumericLists, double* );
  bool* (*GetBooleanValuesList)( int, enum ControlBooleanLists, bool, size_t* );
  void (*SetBooleanValues)( int, enum ControlBooleanLists, bool* );
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
  
  DEBUG_PRINT( "Got hash table iterator %u (insertion status: %d)", newShmControllerID, insertionStatus );
  
  memset( newShmControlData, 0, sizeof(AxisControlData) );
  
  newShmControlData->numericValuesTable[ CONTROL_MEASURES ] = &(newShmControlData->measuresList);
  newShmControlData->numericValuesNumberList[ CONTROL_MEASURES ] = CONTROL_MEASURES_NUMBER;
  newShmControlData->numericValuesTable[ CONTROL_PARAMETERS ] = &(newShmControlData->parametersList);
  newShmControlData->numericValuesNumberList[ CONTROL_PARAMETERS ] = CONTROL_SETPOINTS_NUMBER;
  
  newShmControlData->booleanValuesTable[ CONTROL_STATES ] = &(newShmControlData->statesList);
  newShmControlData->booleanValuesNumberList[ CONTROL_STATES ] = CONTROL_STATES_NUMBER;
  newShmControlData->booleanValuesTable[ CONTROL_COMMANDS ] = &(newShmControlData->commandsList);
  newShmControlData->booleanValuesNumberList[ CONTROL_COMMANDS ] = CONTROL_COMMANDS_NUMBER;
  
  kh_value( controlsDataList, newShmControllerID ) = newShmControlData;
  
  return (int) newShmControllerID;
}

void EndControllerData( int shmControlDataID )
{
  if( !kh_exist( controlsDataList, (khint_t) shmControlDataID ) ) return;

  SharedObjects.DestroyObject( kh_value( controlsDataList, (khint_t) shmControlDataID ) );
    
  kh_del( AxisControl, controlsDataList, (khint_t) shmControlDataID );
    
  if( kh_size( controlsDataList ) == 0 )
  {
    kh_destroy( AxisControl, controlsDataList );
    controlsDataList = NULL;
  }
}

double* GetNumericValuesList( int shmControlDataID, enum ControlNumericLists listIndex, bool remove, size_t* ref_valuesNumber )
{
  if( !kh_exist( controlsDataList, (khint_t) shmControlDataID ) ) return NULL;
    
  if( listIndex < 0 || listIndex >= CONTROL_NUMERIC_LISTS_NUMBER ) return NULL;
    
  AxisControlData* controlData = kh_value( controlsDataList, (khint_t) shmControlDataID );
    
  if( !controlData->numericValuesUpdatedList[ listIndex ] ) return NULL;
    
  if( remove ) controlData->numericValuesUpdatedList[ listIndex ] = false;
  
  if( ref_valuesNumber != NULL ) *ref_valuesNumber = controlData->numericValuesNumberList[ listIndex ];
    
  return controlData->numericValuesTable[ listIndex ];
}

void SetNumericValues( int shmControlDataID, enum ControlNumericLists listIndex, double* numericValuesList )
{
  if( !kh_exist( controlsDataList, (khint_t) shmControlDataID ) ) return;
  
  if( listIndex < 0 || listIndex >= CONTROL_NUMERIC_LISTS_NUMBER ) return;
  
  AxisControlData* controlData = kh_value( controlsDataList, (khint_t) shmControlDataID );
    
  if( numericValuesList != NULL ) 
  {
    size_t dataSize = sizeof(double) * controlData->numericValuesNumberList[ listIndex ];
    memcpy( controlData->numericValuesTable[ listIndex ], numericValuesList, dataSize );
  }
    
  controlData->numericValuesUpdatedList[ listIndex ] = true;
}

bool* GetBooleanValuesList( int shmControlDataID, enum ControlBooleanLists listIndex, bool remove, size_t* ref_valuesNumber )
{
  if( !kh_exist( controlsDataList, (khint_t) shmControlDataID ) ) return NULL;
    
  if( listIndex < 0 || listIndex >= CONTROL_BOOLEAN_LISTS_NUMBER ) return NULL;
  
  AxisControlData* controlData = kh_value( controlsDataList, (khint_t) shmControlDataID );
    
  DEBUG_PRINT( "Getting states for axis data ID %u (address: %p)", (khint_t) shmControlDataID, controlData );
    
  if( !controlData->booleanValuesUpdatedList[ listIndex ] ) return NULL;
    
  if( remove ) controlData->booleanValuesUpdatedList[ listIndex ] = false;
  
  if( ref_valuesNumber != NULL ) *ref_valuesNumber = controlData->booleanValuesNumberList[ listIndex ];
    
  return controlData->booleanValuesTable[ listIndex ];
}

void SetBooleanValues( int shmControlDataID, enum ControlBooleanLists listIndex, bool* booleanValuesList )
{
  if( !kh_exist( controlsDataList, (khint_t) shmControlDataID ) ) return;
  
  if( listIndex < 0 || listIndex >= CONTROL_BOOLEAN_LISTS_NUMBER ) return;
  
  AxisControlData* controlData = kh_value( controlsDataList, (khint_t) shmControlDataID );
    
  DEBUG_PRINT( "Setting states for axis data ID %u (address: %p)", (khint_t) shmControlDataID, controlData );
    
  if( booleanValuesList != NULL ) 
  {
    size_t dataSize = sizeof(double) * controlData->booleanValuesNumberList[ listIndex ];
    memcpy( controlData->booleanValuesTable[ listIndex ], booleanValuesList, dataSize );
  }
    
  controlData->booleanValuesUpdatedList[ listIndex ] = true;
}

#endif // SHM_AXIS_CONTROL_H
