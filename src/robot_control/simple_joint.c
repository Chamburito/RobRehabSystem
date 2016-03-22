#include "robot_control/interface.h"

#include <math.h>

IMPLEMENT_INTERFACE( ROBOT_CONTROL_FUNCTIONS )

const size_t DOFS_NUMBER = 1;
const char* DOF_NAMES[ DOFS_NUMBER ] = { { "angle" } };

Controller InitController( const char* data )
{
  return NULL;
}

void EndController( Controller controller )
{
  
}

size_t GetDoFsNumber( Controller controller )
{
  return DOFS_NUMBER;
}

const char** GetJointNamesList( Controller controller )
{
  return (const char**) DOF_NAMES;
}

const char** GetAxisNamesList( Controller controller )
{
  return (const char**) DOF_NAMES;
}

void RunStep( Controller controller, double** jointMeasuresTable, double** axisMeasuresTable, double** jointSetpointsTable, double** axisSetpointsTable )
{
  axisMeasuresTable[ 0 ][ CONTROL_POSITION ] = jointMeasuresTable[ 0 ][ CONTROL_POSITION ];
	axisMeasuresTable[ 0 ][ CONTROL_VELOCITY ] = jointMeasuresTable[ 0 ][ CONTROL_VELOCITY ];
	axisMeasuresTable[ 0 ][ CONTROL_ACCELERATION ] = jointMeasuresTable[ 0 ][ CONTROL_ACCELERATION ];
  axisMeasuresTable[ 0 ][ CONTROL_FORCE ] = jointMeasuresTable[ 0 ][ CONTROL_FORCE ];
  axisMeasuresTable[ 0 ][ CONTROL_STIFFNESS ] = jointMeasuresTable[ 0 ][ CONTROL_STIFFNESS ];
  axisMeasuresTable[ 0 ][ CONTROL_DAMPING ] = jointMeasuresTable[ 0 ][ CONTROL_DAMPING ];
  
  jointSetpointsTable[ 0 ][ CONTROL_POSITION ] = axisSetpointsTable[ 0 ][ CONTROL_POSITION ];
	jointSetpointsTable[ 0 ][ CONTROL_VELOCITY ] = axisSetpointsTable[ 0 ][ CONTROL_VELOCITY ];
	jointSetpointsTable[ 0 ][ CONTROL_ACCELERATION ] = axisSetpointsTable[ 0 ][ CONTROL_ACCELERATION ];
  jointSetpointsTable[ 0 ][ CONTROL_FORCE ] = axisSetpointsTable[ 0 ][ CONTROL_FORCE ];
  jointSetpointsTable[ 0 ][ CONTROL_STIFFNESS ] = axisSetpointsTable[ 0 ][ CONTROL_STIFFNESS ];
  jointSetpointsTable[ 0 ][ CONTROL_DAMPING ] = axisSetpointsTable[ 0 ][ CONTROL_DAMPING ];
}
