#ifndef ROBOT_MECHANICS_INTERFACE_H
#define ROBOT_MECHANICS_INTERFACE_H

#ifdef __cplusplus
    extern "C" {
#endif

#include "interface.h"

enum { ROBOT_POSITION, ROBOT_VELOCITY, ROBOT_ACCELERATION, ROBOT_FORCE = ROBOT_ACCELERATION, ROBOT_MECHANICS_VARS_NUMBER };

#define ROBOT_MECHANICS_FUNCTIONS( interface, function_init ) \
        function_init( size_t, interface, GetDoFsNumber, void ) \
        function_init( void, interface, GetForwardKinematics, const double**, double*, size_t ) \
        function_init( void, interface, GetInverseKinematics, const double**, double*, size_t ) \
        function_init( void, interface, GetForwardDynamics, const double**, double*, size_t ) \
        function_init( void, interface, GetInverseDynamics, const double**, double*, size_t )
        //function_init( void, interface, GetJointForces, double* const, double* ) \
        //function_init( void, interface, GetJointAccelerations, double* const, double* )

DEFINE_INTERFACE( RobotMechanics, ROBOT_MECHANICS_FUNCTIONS )

#ifdef __cplusplus
    }
#endif

#endif  // ROBOT_MECHANICS_INTERFACE_H
