#ifndef ROBOT_MECHANICS_INTERFACE_H
#define ROBOT_MECHANICS_INTERFACE_H

#ifdef __cplusplus
    extern "C" {
#endif

#include "interfaces.h"

enum { DOF_POSITION, DOF_VELOCITY, DOF_ACCELERATION, DOF_FORCE = DOF_ACCELERATION, DOF_VARS_NUMBER };

#define ROBOT_MECHANICS_FUNCTIONS( interface, function_init ) \
        function_init( int, interface, Init, const char* ) \
        function_init( void, interface, End, int ) \ 
        function_init( size_t, interface, GetDoFsNumber, int ) \
        function_init( void, interface, SolveForwardKinematics, int, double**, double** ) \
        function_init( void, interface, SolveInverseKinematics, int, double**, double** ) \
        function_init( void, interface, SolveForwardDynamics, int, double**, double** ) \
        function_init( void, interface, SolveInverseDynamics, int, double**, double** ) \
        function_init( void, interface, GetJointControlActions, int, double**, double** )

DEFINE_INTERFACE( RobotMechanics, ROBOT_MECHANICS_FUNCTIONS )

#ifdef __cplusplus
    }
#endif

#endif  // ROBOT_MECHANICS_INTERFACE_H
