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

#include "impedance_control.h"

#include "klib/khash.h"

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
  
  int insertionStatus;
  int newControllerID = kh_put( Shm, controllersList, sharedMemoryKey, &insertionStatus );
  
  if( insertionStatus == -1 ) return -1;
  //else if( insertionStatus == 0 ) return kh_get( Shm, controllersList, sharedMemoryKey );
  
  ShmAxisController* newShmController = &(kh_value( controllersList, newControllerID ));
  memset( newShmController, 0, sizeof(ShmAxisController) );
  
  int sharedMemoryID = shmget( OB_KEY, sizeof(AxisController), 0666 );
  if( sharedMemoryID == -1 )
  {
    kh_del( Shm, controllersList, newControllerID );
    return -1;
  }
  
  newShmController->controller = shmat( sharedMemoryID, NULL, 0 );
  
  return newControllerID;
}

void ShmAxisControl_End( int controllerID )
{
  if( kh_exist( controllersList, controllerID ) )
  {
    shmdt( kh_value( controllersList, controllerID ).controller );
    
    kh_del( Shm, controllersList, controllerID );
    
    if( kh_size( controllersList ) == 0 )
    {
      kh_destroy( Shm, controllersList );
      controllersList = NULL;
    }
  }
}

double* ShmAxisControl_GetMeasuresList( int controllerID )
{
  if( kh_exist( controllersList, controllerID ) )
  {
    AxisController* controller = kh_value( controllersList, controllerID ).controller;
    double* measuresList = kh_value( controllersList, controllerID ).measuresList;
    
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
  if( kh_exist( controllersList, controllerID ) )
  {
    AxisController* controller = kh_value( controllersList, controllerID ).controller;
    double* measuresList = kh_value( controllersList, controllerID ).measuresList;
    
    controller->ankle.pos.dp = measuresList[ CONTROL_POSITION ];
    controller->ankle.vel.dp = measuresList[ CONTROL_VELOCITY ];
    controller->ankle.accel.dp = measuresList[ CONTROL_ACCELERATION ];
    controller->ankle.torque.dp = measuresList[ CONTROL_FORCE ];
    
    controller->pos_error.dy = measuresList[ CONTROL_ERROR ];
  }
}

double* ShmAxisControl_GetParametersList( int controllerID )
{
  if( kh_exist( controllersList, controllerID ) )
  {
    AxisController* controller = kh_value( controllersList, controllerID ).controller;
    double* parametersList = kh_value( controllersList, controllerID ).parametersList;
    
    parametersList[ CONTROL_SETPOINT ] = controller->ankle.ref_pos.dp;
    parametersList[ CONTROL_STIFFNESS ] = controller->ankle.stiff;
    parametersList[ CONTROL_DAMPING ] = controller->ankle.damp;
    parametersList[ CONTROL_OFFSET ] = controller->ankle.offset.dp;
    
    return parametersList;
  }
  
  return NULL;
}

void ShmAxisControl_SetParameters( int controllerID )
{
  if( kh_exist( controllersList, controllerID ) )
  {
    AxisController* controller = kh_value( controllersList, controllerID ).controller;
    double* parametersList = kh_value( controllersList, controllerID ).parametersList;
    
    controller->ankle.ref_pos.dp = parametersList[ CONTROL_SETPOINT ];
    controller->ankle.stiff = parametersList[ CONTROL_STIFFNESS ];
    controller->ankle.damp = parametersList[ CONTROL_DAMPING ];
    controller->ankle.offset.dp = parametersList[ CONTROL_OFFSET ];
  }
}

bool* ShmAxisControl_GetStatesList( int controllerID )
{
  if( kh_exist( controllersList, controllerID ) )
  {
    AxisController* controller = kh_value( controllersList, controllerID ).controller;
    bool* statesList = kh_value( controllersList, controllerID ).statesList;
    
    statesList[ CONTROL_HAS_ERROR ] = (bool) controller->fault;
    statesList[ CONTROL_ENABLED ] = (bool) controller->copy_slot.go;
    statesList[ CONTROL_RESET ] = (bool) controller->restart.go;
    
    return statesList;
  }
  
  return NULL;
}

void ShmAxisControl_SetStates( int controllerID )
{
  if( kh_exist( controllersList, controllerID ) )
  {
    AxisController* controller = kh_value( controllersList, controllerID ).controller;
    bool* statesList = kh_value( controllersList, controllerID ).statesList;
    
    controller->fault = (unsigned int) statesList[ CONTROL_HAS_ERROR ];
    controller->copy_slot.go = (unsigned int) statesList[ CONTROL_ENABLED ];
    controller->restart.go = (unsigned int) statesList[ CONTROL_RESET ];
  }
}

#endif // SHM_AXIS_CONTROL_H
