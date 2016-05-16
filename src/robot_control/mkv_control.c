#include "robot_control/interface.h"

#include <math.h>

#include "matrices.h"

#define DOFS_NUMBER 1

const char* DOF_NAMES[ DOFS_NUMBER ] = { "angle" };

typedef struct _ControlData
{
  Matrix statesProbability;
}
ControlData;

DECLARE_MODULE_INTERFACE( ROBOT_CONTROL_INTERFACE ) 


Controller InitController( const char* data )
{
  ControlData* newController = (ControlData*) malloc( sizeof(ControlData) );
  
  double stateProbabilityData[ 3 ][ 3 ] = { { 0.6, 0.3, 0.1 }, { 0.1, 0.6, 0.3 }, { 0.3, 0.1, 0.6 } };
  newController->statesProbability = Matrices.Create( (double*) stateProbabilityData, 3, 3 );
  
  return (Controller) newController;
}

void EndController( Controller ref_controller )
{
  if( ref_controller == NULL ) return;
  
  ControlData* controller = (ControlData*) ref_controller;
  
  Matrices.Discard( controller->statesProbability );
  
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
  
  double jointPosition = jointMeasuresList[ 0 ]->position;
  size_t jointSector = 0;
  double jointSectorLength = 2.0 / 3;
  
  if( jointPosition <= -1.0 + jointSectorLength ) jointSector = 0;
  else if( jointPosition > -1.0 + jointSectorLength && jointPosition < 1.0 - jointSectorLength ) jointSector = 1;
  else if( jointPosition >= 1.0 - jointSectorLength ) jointSector = 2;
  
  double probability = Matrices.GetElement( controller->statesProbability, jointSector, jointSector );
  
  
}
