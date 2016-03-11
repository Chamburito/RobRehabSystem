#include "mechanical_interface.h"

#include <math.h>

IMPLEMENT_INTERFACE( MECHANICS_FUNCTIONS )

const size_t DOFS_NUMBER = 1;

MechanicalModel InitModel( const char* data )
{
  return NULL;
}

void EndModel( MechanicalModel model )
{
  
}

size_t GetDoFsNumber( MechanicalModel model )
{
  return DOFS_NUMBER;
}

void SolveForwardKinematics( MechanicalModel model, double** jointValuesTable, double** axisValuesTable )
{
	axisValuesTable[ 0 ][ DOF_POSITION ] = jointValuesTable[ 0 ][ DOF_POSITION ];
	axisValuesTable[ 0 ][ DOF_VELOCITY ] = jointValuesTable[ 0 ][ DOF_VELOCITY ];
	axisValuesTable[ 0 ][ DOF_ACCELERATION ] = jointValuesTable[ 0 ][ DOF_ACCELERATION ];
}

void SolveInverseKinematics( MechanicalModel model, double** axisValuesTable, double** jointValuesTable )
{
	jointValuesTable[ 0 ][ DOF_POSITION ] = axisValuesTable[ 0 ][ DOF_POSITION ];
	jointValuesTable[ 0 ][ DOF_VELOCITY ] = axisValuesTable[ 0 ][ DOF_VELOCITY ];
	jointValuesTable[ 0 ][ DOF_ACCELERATION ] = axisValuesTable[ 0 ][ DOF_ACCELERATION ];
}

void SolveForwardDynamics( MechanicalModel model, double** jointValuesTable, double** axisValuesTable )
{
	axisValuesTable[ 0 ][ DOF_FORCE ] = jointValuesTable[ 0 ][ DOF_FORCE ];
}

void SolveInverseDynamics( MechanicalModel model, double** axisValuesTable, double** jointValuesTable )
{
	jointValuesTable[ 0 ][ DOF_FORCE ] = axisValuesTable[ 0 ][ DOF_FORCE ];
}

void GetJointControlActions( MechanicalModel model, double** jointMeasuresTable, double** jointSetpointsTable )
{
	return;
}
