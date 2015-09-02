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

enum ControlStates { CONTROL_ENABLED, CONTROL_HAS_ERROR, CONTROL_STATES_NUMBER };
enum ControlCommands { CONTROL_ENABLE, CONTROL_RESET, CONTROL_COMMANDS_NUMBER };

const bool PEEK = false;
const bool REMOVE = true;

typedef struct _AxisControlData
{
  // control -> user
  double measuresList[ CONTROL_MEASURES_NUMBER ];
  bool measureUpdated;
  bool commandsList[ CONTROL_COMMANDS_NUMBER ];
  bool commandUpdated;
  // user -> control
  double parametersList[ CONTROL_SETPOINTS_NUMBER ];
  bool parameterUpdated;
  bool statesList[ CONTROL_STATES_NUMBER ];
  bool stateUpdated;
}
AxisControlData;

KHASH_MAP_INIT_INT( AxisControl, AxisControlData* )
static khash_t( AxisControl )* controlsDataList = NULL;

int ShmAxisControl_Init( int sharedMemoryKey )
{
  if( controlsDataList == NULL ) controlsDataList = kh_init( AxisControl );
  
  DEBUG_PRINT( "Trying to create axis shared memory area with key %x", sharedMemoryKey );
  
  AxisControlData* newShmControlData = SharedObject.CreateByKey( sharedMemoryKey, sizeof(AxisControlData), SHM_READ | SHM_WRITE );
  if( newShmControlData == (void*) -1 ) return -1;
  
  int insertionStatus;
  khint_t newShmControllerID = kh_put( AxisControl, controlsDataList, sharedMemoryKey, &insertionStatus );
  if( insertionStatus == -1 ) return -1;
  
  DEBUG_PRINT( "Got hash table iterator %u (insertion status: %d)", newShmControllerID, insertionStatus );
  
  memset( newShmControlData, 0, sizeof(AxisControlData) );
  
  kh_value( controlsDataList, newShmControllerID ) = newShmControlData;
  
  return (int) newShmControllerID;
}

void ShmAxisControl_End( int shmControlDataID )
{
  if( kh_exist( controlsDataList, (khint_t) shmControlDataID ) )
  {
    SharedObject.Destroy( kh_value( controlsDataList, (khint_t) shmControlDataID ) );
    
    kh_del( AxisControl, controlsDataList, (khint_t) shmControlDataID );
    
    if( kh_size( controlsDataList ) == 0 )
    {
      kh_destroy( AxisControl, controlsDataList );
      controlsDataList = NULL;
    }
  }
}

double* ShmAxisControl_GetMeasuresList( int shmControlDataID, bool remove )
{
  if( kh_exist( controlsDataList, (khint_t) shmControlDataID ) )
  {
    AxisControlData* controlData = kh_value( controlsDataList, (khint_t) shmControlDataID );
    
    if( !controlData->measureUpdated ) return NULL;
    
    if( remove ) controlData->measureUpdated = false;
    
    return (double*) controlData->measuresList;
  }
  
  return NULL;
}

void ShmAxisControl_SetMeasures( int shmControlDataID, double* measuresList )
{
  if( kh_exist( controlsDataList, (khint_t) shmControlDataID ) )
  {
    AxisControlData* controlData = kh_value( controlsDataList, (khint_t) shmControlDataID );
    
    if( controlData != NULL ) memcpy( controlData->measuresList, measuresList, sizeof(double) * CONTROL_MEASURES_NUMBER );
    
    controlData->measureUpdated = true;
  }
}

double* ShmAxisControl_GetParametersList( int shmControlDataID, bool remove )
{
  if( kh_exist( controlsDataList, (khint_t) shmControlDataID ) )
  {
    AxisControlData* controlData = kh_value( controlsDataList, (khint_t) shmControlDataID );
    
    DEBUG_PRINT( "Getting parameters for axis data ID %u (address: %p)", (khint_t) shmControlDataID, controlData );
    
    if( !controlData->parameterUpdated ) return NULL;
    
    if( remove ) controlData->parameterUpdated = false;
    
    return (double*) controlData->parametersList;
  }
  
  return NULL;
}

void ShmAxisControl_SetParameters( int shmControlDataID, double* parametersList )
{
  if( kh_exist( controlsDataList, (khint_t) shmControlDataID ) )
  {
    AxisControlData* controlData = kh_value( controlsDataList, (khint_t) shmControlDataID );
    
    DEBUG_PRINT( "Setting parameters for axis data ID %u (address: %p)", (khint_t) shmControlDataID, controlData );
    
    if( parametersList != NULL ) memcpy( controlData->parametersList, parametersList, sizeof(double) * CONTROL_SETPOINTS_NUMBER );
    
    controlData->parameterUpdated = true;
  }
}

bool* ShmAxisControl_GetStatesList( int shmControlDataID, bool remove )
{
  if( kh_exist( controlsDataList, (khint_t) shmControlDataID ) )
  {
    AxisControlData* controlData = kh_value( controlsDataList, (khint_t) shmControlDataID );
    
    DEBUG_PRINT( "Getting states for axis data ID %u (address: %p)", (khint_t) shmControlDataID, controlData );
    
    if( !controlData->stateUpdated ) return NULL;
    
    if( remove ) controlData->stateUpdated = false;
    
    return (bool*) controlData->statesList;
  }
  
  return NULL;
}

void ShmAxisControl_SetStates( int shmControlDataID, bool* statesList )
{
  if( kh_exist( controlsDataList, (khint_t) shmControlDataID ) )
  {
    AxisControlData* controlData = kh_value( controlsDataList, (khint_t) shmControlDataID );
    
    DEBUG_PRINT( "Setting states for axis data ID %u (address: %p)", (khint_t) shmControlDataID, controlData );
    
    if( statesList != NULL ) memcpy( controlData->statesList, statesList, sizeof(double) * CONTROL_STATES_NUMBER );
    
    controlData->stateUpdated = true;
  }
}

bool* ShmAxisControl_GetCommandsList( int shmControlDataID, bool remove )
{
  if( kh_exist( controlsDataList, (khint_t) shmControlDataID ) )
  {
    AxisControlData* controlData = kh_value( controlsDataList, (khint_t) shmControlDataID );
    
    DEBUG_PRINT( "Getting states for axis data ID %u (address: %p)", (khint_t) shmControlDataID, controlData );
    
    if( !controlData->stateUpdated ) return NULL;
    
    if( remove ) controlData->stateUpdated = false;
    
    return (bool*) controlData->statesList;
  }
  
  return NULL;
}

void ShmAxisControl_SetCommands( int shmControlDataID, bool* commandsList )
{
  if( kh_exist( controlsDataList, (khint_t) shmControlDataID ) )
  {
    AxisControlData* controlData = kh_value( controlsDataList, (khint_t) shmControlDataID );
    
    DEBUG_PRINT( "Setting states for axis data ID %u (address: %p)", (khint_t) shmControlDataID, controlData );
    
    if( commandsList != NULL ) memcpy( controlData->commandsList, commandsList, sizeof(double) * CONTROL_COMMANDS_NUMBER );
    
    controlData->stateUpdated = true;
  }
}

#endif // SHM_AXIS_CONTROL_H
