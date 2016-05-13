#include "robot_control/interface.h"

#include <math.h>

#define DOFS_NUMBER 1

const char* DOF_NAMES[ DOFS_NUMBER ] = { "angle" };

DECLARE_MODULE_INTERFACE( ROBOT_CONTROL_INTERFACE ) 

Controller InitController( const char* data )
{
  return NULL;
}

void EndController( Controller controller )
{
  
}

size_t GetJointsNumber( Controller controller )
{
  return DOFS_NUMBER;
}

char** GetJointNamesList( Controller controller )
{
  return (char**) DOF_NAMES;
}

size_t GetAxesNumber( Controller controller )
{
  return DOFS_NUMBER;
}

char** GetAxisNamesList( Controller controller )
{
  return (char**) DOF_NAMES;
}

void RunControlStep( Controller controller, ControlVariables** jointMeasuresList, ControlVariables** axisMeasuresList, ControlVariables** jointSetpointsList, ControlVariables** axisSetpointsList )
{
  axisMeasuresList[ 0 ]->position = jointMeasuresList[ 0 ]->position;
  axisMeasuresList[ 0 ]->velocity = jointMeasuresList[ 0 ]->velocity;
  axisMeasuresList[ 0 ]->acceleration = jointMeasuresList[ 0 ]->acceleration;
  axisMeasuresList[ 0 ]->force = jointMeasuresList[ 0 ]->force;
  axisMeasuresList[ 0 ]->stiffness = jointMeasuresList[ 0 ]->stiffness;
  axisMeasuresList[ 0 ]->damping = jointMeasuresList[ 0 ]->damping;
  
  jointSetpointsList[ 0 ]->position = axisSetpointsList[ 0 ]->position;
  jointSetpointsList[ 0 ]->velocity = axisSetpointsList[ 0 ]->velocity;
  jointSetpointsList[ 0 ]->acceleration = axisSetpointsList[ 0 ]->acceleration;
  jointSetpointsList[ 0 ]->force = axisSetpointsList[ 0 ]->force;
  jointSetpointsList[ 0 ]->stiffness = axisSetpointsList[ 0 ]->stiffness;
  jointSetpointsList[ 0 ]->damping = axisSetpointsList[ 0 ]->damping;
}
