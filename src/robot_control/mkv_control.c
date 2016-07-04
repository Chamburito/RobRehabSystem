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
  bool enabled;
  int logID;
}
ControlData;

DECLARE_MODULE_INTERFACE( ROBOT_CONTROL_INTERFACE ) 

//const double PROPORTIONAL_GAINS[ 3 ] = { 44.127, -171.0346, -68.9648 };
//const double DERIVATIVE_GAINS[ 3 ] = { 0.0, 0.0, 0.0 };//{ 26.8266, 53.0479, -32.0771 };
const double PROPORTIONAL_GAINS[ 3 ] = { 9.7873, 10.0387, 10.0633 };
const double DERIVATIVE_GAINS[ 3 ] = { 1.1964, 3.2189, 4.2336 };//{ 26.8266, 53.0479, -32.0771 };

Controller InitController( const char* data )
{
  ControlData* newController = (ControlData*) malloc( sizeof(ControlData) );
  
  newController->positionsMaxList[ 0 ] = 0.0;
  
  double stateProbabilityData[ 3 ][ 3 ] = { { 0.9317, 0.0595, 0.0088 }, { 0.0175, 0.9708, 0.0117 }, { 0.0088, 0.04, 0.9512 } };
  newController->statesProbability = Matrices.Create( (double*) stateProbabilityData, 3, 3 );
  
  newController->proportionalGain = newController->derivativeGain = 0.0;
  
  newController->enabled = false;
  
  newController->logID = DataLogging.InitLog( "test", 1, 1000 );
  
  return (Controller) newController;
}

void EndController( Controller genericController )
{
  if( genericController == NULL ) return;
  
  ControlData* controller = (ControlData*) genericController;
  
  Matrices.Discard( controller->statesProbability );
  
  DataLogging.EndLog( controller->logID );
  
  free( controller );
}

size_t GetJointsNumber( Controller genericController )
{
  return DOFS_NUMBER;
}

char** GetJointNamesList( Controller genericController )
{
  return (char**) DOF_NAMES;
}

size_t GetAxesNumber( Controller genericController )
{
  return DOFS_NUMBER;
}

char** GetAxisNamesList( Controller genericController )
{
  return (char**) DOF_NAMES;
}

void SetControlState( Controller genericController, enum ControlState controlState )
{
  if( genericController == NULL ) return;
  
  ControlData* controller = (ControlData*) genericController;
  
  fprintf( stderr, "control state changed: %d", controlState );
  
  controller->enabled = ( controlState == CONTROL_OPERATION ) ? true : false; 
}

void RunControlStep( Controller genericController, double** jointMeasuresTable, double** axisMeasuresTable, double** jointSetpointsTable, double** axisSetpointsTable )
{
  if( genericController == NULL ) return;
  
  ControlData* controller = (ControlData*) genericController;
  
  axisMeasuresTable[ 0 ][ CONTROL_POSITION ] = jointMeasuresTable[ 0 ][ CONTROL_POSITION ];
  axisMeasuresTable[ 0 ][ CONTROL_VELOCITY ] = jointMeasuresTable[ 0 ][ CONTROL_VELOCITY ];
  axisMeasuresTable[ 0 ][ CONTROL_ACCELERATION ] = jointMeasuresTable[ 0 ][ CONTROL_ACCELERATION ];
  axisMeasuresTable[ 0 ][ CONTROL_FORCE ] = jointMeasuresTable[ 0 ][ CONTROL_FORCE ];
  axisMeasuresTable[ 0 ][ CONTROL_STIFFNESS ] = jointMeasuresTable[ 0 ][ CONTROL_STIFFNESS ];
  axisMeasuresTable[ 0 ][ CONTROL_DAMPING ] = jointMeasuresTable[ 0 ][ CONTROL_DAMPING ];
  
  if( fabs( jointMeasuresTable[ 0 ][ CONTROL_POSITION ] ) > controller->positionsMaxList[ 0 ] ) 
    controller->positionsMaxList[ 0 ] = fabs( jointMeasuresTable[ 0 ][ CONTROL_POSITION ] );
  
  if( fabs( controller->positionsMaxList[ 0 ] ) < 0.0001 ) controller->positionsMaxList[ 0 ] = 0.0001;
  
  double jointPosition = jointMeasuresTable[ 0 ][ CONTROL_POSITION ] / controller->positionsMaxList[ 0 ];
  size_t jointSector = 0;
  double jointSectorLength = 2.0 / 3;
  
  if( jointPosition <= -1.0 + jointSectorLength ) jointSector = 0;
  else if( jointPosition > -1.0 + jointSectorLength && jointPosition < 1.0 - jointSectorLength ) jointSector = 1;
  else if( jointPosition >= 1.0 - jointSectorLength ) jointSector = 2;
  
  fprintf( stderr, "pos: %.5f, max: %.5f, norm: %.5f, sector: %lu (%s)\r", jointMeasuresTable[ 0 ][ CONTROL_POSITION ], controller->positionsMaxList[ 0 ], 
                                                                           jointPosition, jointSector, controller->enabled ? "enabled" : "disabled" );
  //DataLogging.RegisterValues( controller->logID, 1, jointSector );
  
  double proportionalGain = PROPORTIONAL_GAINS[ jointSector ];
  double derivativeGain = DERIVATIVE_GAINS[ jointSector ];
  
  double positionError = jointSetpointsTable[ 0 ][ CONTROL_POSITION ] - jointMeasuresTable[ 0 ][ CONTROL_POSITION ];
  double velocityError = jointSetpointsTable[ 0 ][ CONTROL_VELOCITY ] - jointMeasuresTable[ 0 ][ CONTROL_VELOCITY ];
  double result = 1.7863 * ( jointMeasuresTable[ 0 ][ CONTROL_ACCELERATION ] + proportionalGain * positionError + derivativeGain * velocityError ) 
                  + 0.1981 * jointMeasuresTable[ 0 ][ CONTROL_VELOCITY ] + 0.1111;
  
  jointSetpointsTable[ 0 ][ CONTROL_FORCE ] = result;
  
  if( !controller->enabled ) jointSetpointsTable[ 0 ][ CONTROL_FORCE ] = 0.0; 
}
