#ifndef MECHANICS_INTERFACE_H
#define MECHANICS_INTERFACE_H

#ifdef __cplusplus
    extern "C" {
#endif

#include "interfaces.h"

#define MECHANICS_INVALID_MODEL NULL
      
typedef void* MechanicalModel;
      
enum { DOF_POSITION, DOF_VELOCITY, DOF_ACCELERATION, DOF_FORCE, DOF_STIFFNESS, DOF_DAMPING, DOF_VARS_NUMBER };

#define MECHANICS_FUNCTIONS( interface, function_init ) \
        function_init( MechanicalModel, interface, InitModel, const char* ) \
        function_init( void, interface, EndModel, MechanicalModel ) \ 
        function_init( size_t, interface, GetDoFsNumber, MechanicalModel ) \
        function_init( const char**, interface, GetJointNames, MechanicalModel ) \
        function_init( const char**, interface, GetAxisNames, MechanicalModel ) \
        function_init( void, interface, SolveForwardMechanics, MechanicalModel, double**, double** ) \
        function_init( void, interface, SolveInverseMechanics, MechanicalModel, double**, double** ) \
        function_init( void, interface, GetJointControlActions, MechanicalModel, double**, double** )

DEFINE_INTERFACE( Mechanics, MECHANICS_FUNCTIONS )

#ifdef __cplusplus
    }
#endif

#endif  // ROBOT_MECHANICS_INTERFACE_H
