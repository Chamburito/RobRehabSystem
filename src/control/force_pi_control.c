#include "control_interface.h"

IMPLEMENT_INTERFACE( CONTROL_FUNCTIONS )

typedef struct _ControlData
{
  double positionErrorSum, positionSetpointSum;
  double velocitySetpoint;
  double forceError[ 2 ];
  double controlError;
  double outputsList[ CONTROL_VARS_NUMBER ];
}
ControlData;

Controller InitController( void )
{
  void* newController = malloc( sizeof(ControlData) );
  memset( newController, 0, sizeof(ControlData) );
  
  return (Controller) newController;
}

double* RunStep( Controller controller, double measuresList[ CONTROL_VARS_NUMBER ], double setpointsList[ CONTROL_VARS_NUMBER ], double deltaTime, double* ref_error )
{
  ControlData* controlData = (ControlData*) controller;
  
  double position = measuresList[ CONTROL_POSITION ];
  
  double positionSetpoint = setpointsList[ CONTROL_POSITION ];
  controlData->outputsList[ CONTROL_POSITION ] = positionSetpoint;
  
  double positionError = position - positionSetpoint;
  controlData->positionErrorSum += deltaTime * positionError * positionError;
  controlData->positionSetpointSum += deltaTime * positionSetpoint * positionSetpoint;
  
  if( ref_error != NULL )
  {
    if( *ref_error == 0.0 )
    {
      controlData->controlError = ( controlData->positionSetpointSum > 0.0 ) ? controlData->positionErrorSum / controlData->positionSetpointSum : 1.0;
      if( controlData->controlError > 1.0 ) controlData->controlError = 1.0;
    
      controlData->positionErrorSum = 0.0;
      controlData->positionSetpointSum = 0.0;
    }
    
    *ref_error = positionError / positionSetpoint;
  }
  
  double forceSetpoint = controlData->controlError * setpointsList[ CONTROL_FORCE ];
  controlData->outputsList[ CONTROL_FORCE ] = forceSetpoint;

  double force = measuresList[ CONTROL_FORCE ];
  controlData->forceError[ 0 ] = forceSetpoint - force;
  
  controlData->velocitySetpoint += 370.0 * ( controlData->forceError[ 0 ] - controlData->forceError[ 1 ] ) + 3.5 * deltaTime * controlData->forceError[ 0 ];
  controlData->outputsList[ CONTROL_VELOCITY ] = controlData->velocitySetpoint;
  
  //velocitySetpoint[0] = 0.9822 * velocitySetpoint[1] + 0.01407 * velocitySetpoint[2] + 338.6 * forceError[1] - 337.4 * forceError[2]; //5ms
  
  controlData->forceError[ 1 ] = controlData->forceError[ 0 ];

  return (double*) controlData->outputsList;
}

void EndController( Controller controller )
{
  free( (void*) controller );
}
