#include "control_interface.h"

IMPLEMENT_INTERFACE( CONTROL_FUNCTIONS )

double* Run( double measuresList[ CONTROL_VARS_NUMBER ], double setpointsList[ CONTROL_VARS_NUMBER ], double deltaTime, double* ref_error )
{
  static double position, positionErrorSum, positionSetpointSum;
  static double velocitySetpoint;
  static double forceError[ 2 ];
  
  static double outputsList[ CONTROL_VARS_NUMBER ];
  
  static double error;
  
  if( *ref_error == 0.0 )
  {
    /*error = ( positionSetpointSum > 0.0 ) ? positionErrorSum / positionSetpointSum : 1.0;
    if( error > 1.0 )*/ error = 1.0;
    
    positionErrorSum = 0.0;
    positionSetpointSum = 0.0;
  }

  position = measuresList[ CONTROL_POSITION ];
  
  double positionSetpoint = setpointsList[ CONTROL_POSITION ];
  outputsList[ CONTROL_POSITION ] = positionSetpoint;
  double positionError = position - positionSetpoint;
  positionErrorSum += deltaTime * positionError * positionError;
  positionSetpointSum += deltaTime * positionSetpoint * positionSetpoint;
  
  *ref_error = positionError / positionSetpoint;
  
  double forceSetpoint = error * setpointsList[ CONTROL_FORCE ];
  outputsList[ CONTROL_FORCE ] = forceSetpoint;

  double force = measuresList[ CONTROL_FORCE ];
  forceError[ 0 ] = forceSetpoint - force;
  
  velocitySetpoint += 370.0 * ( forceError[0] - forceError[1] ) + 3.5 * deltaTime * forceError[0];
  outputsList[ CONTROL_VELOCITY ] = velocitySetpoint;
  
  //DEBUG_PRINT( "force:%.3f (set:%.3f) - pos:%.3f (set:%.3f) -> %.3f", force, forceSetpoint, position, positionSetpoint, velocitySetpoint );
  
  //velocitySetpoint[0] = 0.9822 * velocitySetpoint[1] + 0.01407 * velocitySetpoint[2] + 338.6 * forceError[1] - 337.4 * forceError[2]; //5ms
  
  forceError[ 1 ] = forceError[ 0 ];

  return (double*) outputsList;
}
