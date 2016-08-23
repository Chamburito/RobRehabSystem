#include "actuator_control/interface.h"

#include <string.h>

typedef struct _ControlData
{
  double positionErrorSum, positionSetpointSum;
  double velocitySetpoint;
  double forceError[ 2 ];
  double controlError;
  ControlVariablesList outputsList;
}
ControlData;

DECLARE_MODULE_INTERFACE( ACTUATOR_CONTROL_INTERFACE )

Controller InitController( void )
{
  void* newController = malloc( sizeof(ControlData) );
  memset( newController, 0, sizeof(ControlData) );
  
  return (Controller) newController;
}

double* RunControlStep( Controller controller, double* measuresList, double* setpointsList, double* ref_error )
{
  const double K_P = 370;//1.6527; // 370 * ( F_in_max / V_out_max )
  const double K_I = 3.5;//0.0156; // 3.5 * ( F_in_max / V_out_max )
  
  ControlData* controlData = (ControlData*) controller;
  
  controlData->outputsList[ CONTROL_POSITION ] = setpointsList[ CONTROL_POSITION ];
  
  double positionError = measuresList[ CONTROL_POSITION ] - setpointsList[ CONTROL_POSITION ];
  controlData->positionErrorSum += CONTROL_PASS_INTERVAL * positionError * positionError;
  controlData->positionSetpointSum += CONTROL_PASS_INTERVAL * setpointsList[ CONTROL_POSITION ] * setpointsList[ CONTROL_POSITION ];
  
  if( ref_error != NULL )
  {
    if( *ref_error == 0.0 )
    {
      controlData->controlError = ( controlData->positionSetpointSum > 0.0 ) ? controlData->positionErrorSum / controlData->positionSetpointSum : 1.0;
      if( controlData->controlError > 1.0 ) controlData->controlError = 1.0;
    
      controlData->positionErrorSum = 0.0;
      controlData->positionSetpointSum = 0.0;
    }
    
    *ref_error = positionError / setpointsList[ CONTROL_POSITION ];
  }
  
  double forceSetpoint = controlData->controlError * setpointsList[ CONTROL_FORCE ];
  controlData->outputsList[ CONTROL_FORCE ] = forceSetpoint;

  controlData->forceError[ 0 ] = forceSetpoint - measuresList[ CONTROL_FORCE ];
  
  controlData->velocitySetpoint += K_P * ( controlData->forceError[ 0 ] - controlData->forceError[ 1 ] ) + K_I * CONTROL_PASS_INTERVAL * controlData->forceError[ 0 ];
  controlData->outputsList[ CONTROL_VELOCITY ] = controlData->velocitySetpoint;
  
  //velocitySetpoint[0] = 0.9822 * velocitySetpoint[1] + 0.01407 * velocitySetpoint[2] + 338.6 * forceError[1] - 337.4 * forceError[2]; //5ms
  
  controlData->forceError[ 1 ] = controlData->forceError[ 0 ];

  return (double*) controlData->outputsList;
}

void EndController( Controller controller )
{
  free( (void*) controller );
}
