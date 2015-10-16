#ifndef ROBOT_MECHANICS_INTERFACE_H
#define ROBOT_MECHANICS_INTERFACE_H

#ifdef __cplusplus
    extern "C" {
#endif

#include "interface.h"

#define ROBOT_MECHANICS_FUNCTIONS( interface, function_init ) \
        function_init( size_t, interface, GetMaxDOFs, void ) \
        function_init( double*, interface, GetForwardKinematics, double*, size_t ) \
        function_init( double*, interface, GetInverseKinematics, double*, size_t ) \
        function_init( double*, interface, GetForwardDynamics, double*, size_t ) \
        function_init( double*, interface, GetInverseDynamics, double*, size_t )

DEFINE_INTERFACE( RobotMechanics, ROBOT_MECHANICS_FUNCTIONS )

#ifdef __cplusplus
    }
#endif

#endif  // ROBOT_MECHANICS_INTERFACE_H
