#include "robot_mechanics_interface.h"

IMPLEMENT_INTERFACE( ROBOT_MECHANICS_FUNCTIONS )

const size_t DOFS_NUMBER = 1;
const char* DOF_NAMES_LIST[ DOFS_NUMBER ] = { "Theta" };

const char** GetDoFsList( size_t* ref_dofsCount )
{
  if( ref_dofsCount != NULL ) *ref_dofsCount = DOFS_NUMBER;
  
  return (const char**) DOF_NAMES_LIST;
}

void GetForwardKinematics( const double** jointValuesTable, double** dofValuesTable )
{
  DEBUG_PRINT( "calculating forward kinematics for table %p", jointValuesTable );
  
  dofValuesTable[ 0 ][ ROBOT_POSITION ] = jointValuesTable[ 0 ][ ROBOT_POSITION ];
  dofValuesTable[ 0 ][ ROBOT_VELOCITY ] = jointValuesTable[ 0 ][ ROBOT_VELOCITY ];
  dofValuesTable[ 0 ][ ROBOT_ACCELERATION ] = jointValuesTable[ 0 ][ ROBOT_ACCELERATION ];
}

void GetInverseKinematics( const double** dofValuesTable, double** jointValuesTable )
{
  DEBUG_PRINT( "calculating inverse kinematics for table %p", dofValuesTable );
  
  jointValuesTable[ 0 ][ ROBOT_POSITION ] = dofValuesTable[ 0 ][ ROBOT_POSITION ];
  jointValuesTable[ 0 ][ ROBOT_VELOCITY ] = dofValuesTable[ 0 ][ ROBOT_VELOCITY ];
  jointValuesTable[ 0 ][ ROBOT_ACCELERATION ] = dofValuesTable[ 0 ][ ROBOT_ACCELERATION ];
}

void GetForwardDynamics( const double** jointValuesTable, double** dofValuesTable )
{
  //DEBUG_PRINT( "calculating forward dynamics for table %p", jointValuesTable );
  
  dofValuesTable[ 0 ][ ROBOT_POSITION ] = jointValuesTable[ 0 ][ ROBOT_POSITION ];
  dofValuesTable[ 0 ][ ROBOT_VELOCITY ] = jointValuesTable[ 0 ][ ROBOT_VELOCITY ];
  dofValuesTable[ 0 ][ ROBOT_FORCE ] = jointValuesTable[ 0 ][ ROBOT_FORCE ];
}

void GetInverseDynamics( const double** dofValuesTable, double** jointValuesTable )
{
  DEBUG_PRINT( "calculating inverse dynamics: %g %g %g", dofValuesTable[ 0 ][ ROBOT_POSITION ], dofValuesTable[ 0 ][ ROBOT_VELOCITY ], dofValuesTable[ 0 ][ ROBOT_FORCE ] );
  
  jointValuesTable[ 0 ][ ROBOT_POSITION ] = dofValuesTable[ 0 ][ ROBOT_POSITION ];
  jointValuesTable[ 0 ][ ROBOT_VELOCITY ] = dofValuesTable[ 0 ][ ROBOT_VELOCITY ];
  jointValuesTable[ 0 ][ ROBOT_FORCE ] = dofValuesTable[ 0 ][ ROBOT_FORCE ];
}
