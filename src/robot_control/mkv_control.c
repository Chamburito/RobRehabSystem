#include "robot_control/interface.h"

#include <math.h>

#include "matrices.h"
#include "debug/data_logging.h"

#define DOFS_NUMBER 1

const char* DOF_NAMES[ DOFS_NUMBER ] = { "angle" };

typedef struct _ControlData
{
  double positionsMaxList[ DOFS_NUMBER ];
  Matrix statesProbability;
  double proportionalGain, derivativeGain;
  int logID;
}
ControlData;

DECLARE_MODULE_INTERFACE( ROBOT_CONTROL_INTERFACE ) 

const double PROPORTIONAL_GAINS[ 3 ] = { 20.0, 0.0, 20.0 };//{ 44.127, -171.0346, -68.9648 };
const double DERIVATIVE_GAINS[ 3 ] = { 0.0, 0.0, 0.0 };//{ 26.8266, 53.0479, -32.0771 };

Controller InitController( const char* data )
{
  ControlData* newController = (ControlData*) malloc( sizeof(ControlData) );
  
  newController->positionsMaxList[ 0 ] = 0.0;
  
  double stateProbabilityData[ 3 ][ 3 ] = { { 0.9317, 0.0595, 0.0088 }, { 0.0175, 0.9708, 0.0117 }, { 0.0088, 0.04, 0.9512 } };
  newController->statesProbability = Matrices.Create( (double*) stateProbabilityData, 3, 3 );
  
  newController->logID = DataLogging.InitLog( "test", 1, 1000 );
  
  return (Controller) newController;
}

void EndController( Controller ref_controller )
{
  if( ref_controller == NULL ) return;
  
  ControlData* controller = (ControlData*) ref_controller;
  
  Matrices.Discard( controller->statesProbability );
  
  DataLogging.EndLog( controller->logID );
  
  free( controller );
}

size_t GetJointsNumber( Controller ref_controller )
{
  return DOFS_NUMBER;
}

char** GetJointNamesList( Controller ref_controller )
{
  return (char**) DOF_NAMES;
}

size_t GetAxesNumber( Controller ref_controller )
{
  return DOFS_NUMBER;
}

char** GetAxisNamesList( Controller ref_controller )
{
  return (char**) DOF_NAMES;
}

void SetControlState( Controller ref_controller, enum ControlState controlState )
{
  fprintf( stderr, "Setting robot control state: %x\n", controlState );
}

void RunControlStep( Controller ref_controller, double** jointMeasuresTable, double** axisMeasuresTable, double** jointSetpointsTable, double** axisSetpointsTable )
{
  if( ref_controller == NULL ) return;
  
  ControlData* controller = (ControlData*) ref_controller;
  
  axisMeasuresTable[ 0 ][ CONTROL_POSITION ] = jointMeasuresTable[ 0 ][ CONTROL_POSITION ];
  axisMeasuresTable[ 0 ][ CONTROL_VELOCITY ] = jointMeasuresTable[ 0 ][ CONTROL_VELOCITY ];
  axisMeasuresTable[ 0 ][ CONTROL_ACCELERATION ] = jointMeasuresTable[ 0 ][ CONTROL_ACCELERATION ];
  axisMeasuresTable[ 0 ][ CONTROL_FORCE ] = jointMeasuresTable[ 0 ][ CONTROL_FORCE ];
  axisMeasuresTable[ 0 ][ CONTROL_STIFFNESS ] = jointMeasuresTable[ 0 ][ CONTROL_STIFFNESS ];
  axisMeasuresTable[ 0 ][ CONTROL_DAMPING ] = jointMeasuresTable[ 0 ][ CONTROL_DAMPING ];
  
  bool enabled = ( axisSetpointsTable[ 0 ][ CONTROL_STIFFNESS ] > 0.0 ) ? true : false;
  
  if( fabs( jointMeasuresTable[ 0 ][ CONTROL_POSITION ] ) > controller->positionsMaxList[ 0 ] ) 
    controller->positionsMaxList[ 0 ] = fabs( jointMeasuresTable[ 0 ][ CONTROL_POSITION ] );
  
  if( fabs( controller->positionsMaxList[ 0 ] ) < 0.0001 ) controller->positionsMaxList[ 0 ] = 0.0001;
  
  double jointPosition = jointMeasuresTable[ 0 ][ CONTROL_POSITION ] / controller->positionsMaxList[ 0 ];
  size_t jointSector = 0;
  double jointSectorLength = 2.0 / 3;
  
  if( jointPosition <= -1.0 + jointSectorLength ) jointSector = 0;
  else if( jointPosition > -1.0 + jointSectorLength && jointPosition < 1.0 - jointSectorLength ) jointSector = 1;
  else if( jointPosition >= 1.0 - jointSectorLength ) jointSector = 2;
  
  //fprintf( stderr, "pos: %.5f, max: %.5f, norm: %.5f, sector: %lu (%s)\r", jointMeasuresTable[ 0 ][ CONTROL_POSITION ], controller->positionsMaxList[ 0 ], 
  //                                                                         jointPosition, jointSector, enabled ? "enabled" : "disabled" );
  //DataLogging.RegisterValues( controller->logID, 1, jointSector );
  
  double proportionalGain = PROPORTIONAL_GAINS[ jointSector ];
  double derivativeGain = DERIVATIVE_GAINS[ jointSector ];
  
  double positionError = jointSetpointsTable[ 0 ][ CONTROL_POSITION ] - jointMeasuresTable[ 0 ][ CONTROL_POSITION ];
  double velocityError = jointSetpointsTable[ 0 ][ CONTROL_VELOCITY ] - jointMeasuresTable[ 0 ][ CONTROL_VELOCITY ];
  double result = 1.7863 * ( jointMeasuresTable[ 0 ][ CONTROL_ACCELERATION ] + proportionalGain * positionError + derivativeGain * velocityError ) 
                  + 0.1981 * jointMeasuresTable[ 0 ][ CONTROL_VELOCITY ] + 0.1111;
  
  result = proportionalGain * positionError;
  
  jointSetpointsTable[ 0 ][ CONTROL_FORCE ] = result;
  
  if( !enabled ) jointSetpointsTable[ 0 ][ CONTROL_FORCE ] = 0.0; 
}
