#include "robot_control/interface.h"

#include <math.h>

#include "matrices.h"
#include "debug/data_logging.h"

#define DOFS_NUMBER 1

const char* DOF_NAMES[ DOFS_NUMBER ] = { "angle" };

typedef struct _ControlData
{
  ControlVariables measuresMaxList[ DOFS_NUMBER ];
  Matrix statesProbability;
  double proportionalGain, derivativeGain;
  int logID;
}
ControlData;

DECLARE_MODULE_INTERFACE( ROBOT_CONTROL_INTERFACE ) 

const double PROPORTIONAL_GAINS[ 3 ] = { 44.127, -171.0346, -68.9648 };
const double DERIVATIVE_GAINS[ 3 ] = { 26.8266, 53.0479, -32.0771 };

Controller InitController( const char* data )
{
  ControlData* newController = (ControlData*) malloc( sizeof(ControlData) );
  
  newController->measuresMaxList[ 0 ].position = 1.0;
  
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

void RunControlStep( Controller ref_controller, ControlVariables** jointMeasuresList, ControlVariables** axisMeasuresList, ControlVariables** jointSetpointsList, ControlVariables** axisSetpointsList )
{
  if( ref_controller == NULL ) return;
  
  ControlData* controller = (ControlData*) ref_controller;
  
  if( fabs( jointMeasuresList[ 0 ]->position ) > controller->measuresMaxList[ 0 ].position ) controller->measuresMaxList[ 0 ].position = fabs( jointMeasuresList[ 0 ]->position );
  if( fabs( controller->measuresMaxList[ 0 ].position ) < 0.0001 ) controller->measuresMaxList[ 0 ].position = 0.0001;
  
  double jointPosition = jointMeasuresList[ 0 ]->position / controller->measuresMaxList[ 0 ].position;
  size_t jointSector = 0;
  double jointSectorLength = 2.0 / 3;
  
  if( jointPosition <= -1.0 + jointSectorLength ) jointSector = 0;
  else if( jointPosition > -1.0 + jointSectorLength && jointPosition < 1.0 - jointSectorLength ) jointSector = 1;
  else if( jointPosition >= 1.0 - jointSectorLength ) jointSector = 2;
  
  DataLogging.RegisterValues( controller->logID, 1, jointSector );
  
  double proportionalGain = PROPORTIONAL_GAINS[ jointSector ];
  double derivativeGain = DERIVATIVE_GAINS[ jointSector ];
  
  double positionError = jointSetpointsList[ 0 ]->position - jointMeasuresList[ 0 ]->position;
  double velocityError = jointSetpointsList[ 0 ]->velocity - jointMeasuresList[ 0 ]->velocity;
  double result = 1.7863 * ( jointMeasuresList[ 0 ]->acceleration + proportionalGain * positionError + derivativeGain * velocityError ) + 0.1981 * jointMeasuresList[ 0 ]->velocity + 0.1111;
  
  //double probability = Matrices.GetElement( controller->statesProbability, jointSector, jointSector );
  
  jointSetpointsList[ 0 ]->force = result;
  
  if( jointSetpointsList[ 0 ]->stiffness == 0.0 ) jointSetpointsList[ 0 ]->force = 0.0; 
}
