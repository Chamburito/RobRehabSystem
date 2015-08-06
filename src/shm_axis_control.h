#ifndef SHM_AXIS_CONTROL_H
#define SHM_AXIS_CONTROL_H

#include <stdbool.h>

#include <sys/mman.h>

#include <sys/ipc.h>
#include <sys/shm.h>

#include <fcntl.h>

#include "shm_axis/robdecls.h"
//#include "shm_axis/rtl_inc.h"
//#include "shm_axis/ruser.h"

#include "klib/khash.h"

#include "impedance_control.h"

enum ControlStates { CONTROL_HAS_ERROR, CONTROL_ENABLED, CONTROL_RESET, CONTROL_STATES_NUMBER };

typedef Ob AxisController;

typedef struct _ShmAxisController
{
  int actuatorID;
  AxisController* controller;
  double measuresList[ CONTROL_DIMS_NUMBER ];
  double parametersList[ CONTROL_PARAMS_NUMBER ];
  bool statesList[ CONTROL_STATES_NUMBER ];
}
ShmAxisController;

KHASH_MAP_INIT_INT( Shm, ShmAxisController );
static khash_t( Shm )* controllersList = NULL;

int ShmAxisControl_Init( int sharedMemoryKey )
{
  if( controllersList == NULL ) controllersList = kh_init( Shm );
  
  DEBUG_PRINT( "Trying to create axis shared memory area with key %x", sharedMemoryKey );
  
  int insertionStatus;
  khint_t newControllerID = kh_put( Shm, controllersList, sharedMemoryKey, &insertionStatus );
  
  if( insertionStatus == -1 ) return -1;
  //else if( insertionStatus == 0 ) return kh_get( Shm, controllersList, sharedMemoryKey );
  
  DEBUG_PRINT( "Got hash table iterator %u (insertion status: %d)", newControllerID, insertionStatus );
  
  ShmAxisController* newShmController = &(kh_value( controllersList, newControllerID ));
  memset( newShmController, 0, sizeof(ShmAxisController) );
  
  int sharedMemoryID = shmget( OB_KEY, sizeof(AxisController), IPC_CREAT | 0666 );
  
  DEBUG_PRINT( "Got shared memory area ID %d", sharedMemoryID );
  
  if( sharedMemoryID == -1 )
  {
    perror( "Failed to create shared memory segment" );
    kh_del( Shm, controllersList, newControllerID );
    return -1;
  }
  
  newShmController->controller = shmat( sharedMemoryID, NULL, 0 );
  
  DEBUG_PRINT( "Binded object address %p to shared memory area", newShmController->controller );
  
  if( newShmController->controller == (void*) -1 )
    perror( "Failed to bind object" );
  
  return (int) newControllerID;
}

void ShmAxisControl_End( int controllerID )
{
  if( kh_exist( controllersList, (khint_t) controllerID ) )
  {
    shmdt( kh_value( controllersList, (khint_t) controllerID ).controller );
    
    kh_del( Shm, controllersList, (khint_t) controllerID );
    
    if( kh_size( controllersList ) == 0 )
    {
      kh_destroy( Shm, controllersList );
      controllersList = NULL;
    }
  }
}

double* ShmAxisControl_GetMeasuresList( int controllerID )
{
  if( kh_exist( controllersList, (khint_t) controllerID ) )
  {
    AxisController* controller = kh_value( controllersList, (khint_t) controllerID ).controller;
    double* measuresList = kh_value( controllersList, (khint_t) controllerID ).measuresList;
    
    measuresList[ CONTROL_POSITION ] = controller->ankle.pos.dp;
    measuresList[ CONTROL_VELOCITY ] = controller->ankle.vel.dp;
    measuresList[ CONTROL_ACCELERATION ] = controller->ankle.accel.dp;
    measuresList[ CONTROL_FORCE ] = controller->ankle.torque.dp;
    
    measuresList[ CONTROL_ERROR ] = controller->pos_error.dy;
    
    return measuresList;
  }
  
  return NULL;
}

void ShmAxisControl_SetMeasures( int controllerID )
{
  if( kh_exist( controllersList, (khint_t) controllerID ) )
  {
    AxisController* controller = kh_value( controllersList, (khint_t) controllerID ).controller;
    double* measuresList = kh_value( controllersList, (khint_t) controllerID ).measuresList;
    
    controller->ankle.pos.dp = measuresList[ CONTROL_POSITION ];
    controller->ankle.vel.dp = measuresList[ CONTROL_VELOCITY ];
    controller->ankle.accel.dp = measuresList[ CONTROL_ACCELERATION ];
    controller->ankle.torque.dp = measuresList[ CONTROL_FORCE ];
    
    controller->pos_error.dy = measuresList[ CONTROL_ERROR ];
  }
}

double* ShmAxisControl_GetParametersList( int controllerID )
{
  DEBUG_PRINT( "Trying to get parameters from axis %u", (khint_t) controllerID );
  if( kh_exist( controllersList, (khint_t) controllerID ) )
  {
    DEBUG_PRINT( "Getting parameters from axis %u", (khint_t) controllerID );
    
    AxisController* controller = kh_value( controllersList, (khint_t) controllerID ).controller;
    double* parametersList = kh_value( controllersList, (khint_t) controllerID ).parametersList;
    
    DEBUG_PRINT( "Getting parameters from address %p", controller );
    
    parametersList[ CONTROL_SETPOINT ] = controller->ankle.ref_pos.dp;
    parametersList[ CONTROL_STIFFNESS ] = controller->ankle.stiff;
    parametersList[ CONTROL_DAMPING ] = controller->ankle.damp;
    parametersList[ CONTROL_OFFSET ] = controller->ankle.offset.dp;
    
    DEBUG_PRINT( "Got parameters from address %p", controller );
    
    return parametersList;
  }
  
  return NULL;
}

void ShmAxisControl_SetParameters( int controllerID )
{
  DEBUG_PRINT( "Setting parameters for axis %u", (khint_t) controllerID );
  if( kh_exist( controllersList, (khint_t) controllerID ) )
  {
    AxisController* controller = kh_value( controllersList, (khint_t) controllerID ).controller;
    double* parametersList = kh_value( controllersList, (khint_t) controllerID ).parametersList;
    
    DEBUG_PRINT( "Setting parameters for address %p", controller );
    
    controller->ankle.ref_pos.dp = parametersList[ CONTROL_SETPOINT ];
    controller->ankle.stiff = parametersList[ CONTROL_STIFFNESS ];
    controller->ankle.damp = parametersList[ CONTROL_DAMPING ];
    controller->ankle.offset.dp = parametersList[ CONTROL_OFFSET ];
  }
}

bool* ShmAxisControl_GetStatesList( int controllerID )
{
  if( kh_exist( controllersList, (khint_t) controllerID ) )
  {
    AxisController* controller = kh_value( controllersList, (khint_t) controllerID ).controller;
    bool* statesList = kh_value( controllersList, (khint_t) controllerID ).statesList;
    
    statesList[ CONTROL_HAS_ERROR ] = (bool) controller->fault;
    statesList[ CONTROL_ENABLED ] = (bool) controller->copy_slot.go;
    statesList[ CONTROL_RESET ] = (bool) controller->restart.go;
    
    return statesList;
  }
  
  return NULL;
}

void ShmAxisControl_SetStates( int controllerID )
{
  if( kh_exist( controllersList, (khint_t) controllerID ) )
  {
    AxisController* controller = kh_value( controllersList, (khint_t) controllerID ).controller;
    bool* statesList = kh_value( controllersList, (khint_t) controllerID ).statesList;
    
    controller->fault = (unsigned int) statesList[ CONTROL_HAS_ERROR ];
    controller->copy_slot.go = (unsigned int) statesList[ CONTROL_ENABLED ];
    controller->restart.go = (unsigned int) statesList[ CONTROL_RESET ];
  }
}

#endif // SHM_AXIS_CONTROL_H
