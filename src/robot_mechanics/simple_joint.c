#include "robot_mechanics_interface.h"

size_t GetMaxDOFs( void )
{
  return 1;
}

double* GetForwardKinematics( double** jointValuesTable, size_t dofsNumber )
{
  return jointValuesTable[ 0 ];
}

        function_init( double*, interface, GetInverseKinematics, double*, size_t ) \
        function_init( double*, interface, GetForwardDynamics, double*, size_t ) \
        function_init( double*, interface, GetInverseDynamics, double*, size_t )
