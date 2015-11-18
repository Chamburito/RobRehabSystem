#include "robot_mechanics_interface.h"

IMPLEMENT_INTERFACE( ROBOT_MECHANICS_FUNCTIONS )

const size_t DOFS_NUMBER = 1;

size_t GetDoFsNumber( void )
{
  return DOFS_NUMBER;
}

void GetForwardKinematics( double** jointValuesTable, double* dofValuesList, size_t dofIndex )
{
  //DEBUG_PRINT( "calculating forward kinematics for dof %lu", dofIndex );
  
  dofValuesList[ ROBOT_POSITION ] = jointValuesTable[ 0 ][ ROBOT_POSITION ];
  dofValuesList[ ROBOT_VELOCITY ] = jointValuesTable[ 0 ][ ROBOT_VELOCITY ];
  dofValuesList[ ROBOT_ACCELERATION ] = jointValuesTable[ 0 ][ ROBOT_ACCELERATION ];
}

void GetInverseKinematics( double** dofValuesTable, double* jointValuesList, size_t jointIndex )
{
  //DEBUG_PRINT( "calculating inverse kinematics for joint %lu", jointIndex );
  
  jointValuesList[ ROBOT_POSITION ] = dofValuesTable[ 0 ][ ROBOT_POSITION ];
  jointValuesList[ ROBOT_VELOCITY ] = dofValuesTable[ 0 ][ ROBOT_VELOCITY ];
  jointValuesList[ ROBOT_ACCELERATION ] = dofValuesTable[ 0 ][ ROBOT_ACCELERATION ];
}

void GetForwardDynamics( double** jointValuesTable, double* dofValuesList, size_t dofIndex )
{
  //DEBUG_PRINT( "calculating forward dynamics for dof %lu", dofIndex );
  
  dofValuesList[ ROBOT_POSITION ] = jointValuesTable[ 0 ][ ROBOT_POSITION ];
  dofValuesList[ ROBOT_VELOCITY ] = jointValuesTable[ 0 ][ ROBOT_VELOCITY ];
  dofValuesList[ ROBOT_FORCE ] = jointValuesTable[ 0 ][ ROBOT_FORCE ];
}

void GetInverseDynamics( double** dofValuesTable, double* jointValuesList, size_t jointIndex )
{
  //DEBUG_PRINT( "calculating inverse dynamics for joint %lu", jointIndex );
  
  jointValuesList[ ROBOT_POSITION ] = dofValuesTable[ 0 ][ ROBOT_POSITION ];
  jointValuesList[ ROBOT_VELOCITY ] = dofValuesTable[ 0 ][ ROBOT_VELOCITY ];
  jointValuesList[ ROBOT_FORCE ] = dofValuesTable[ 0 ][ ROBOT_FORCE ];
}
