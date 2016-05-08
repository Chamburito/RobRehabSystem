#ifndef MOTORS_H
#define MOTORS_H

#include "namespaces.h"
      
      
typedef struct _MotorData MotorData;
typedef MotorData* Motor;

#define MOTOR_INTERFACE( Namespace, INIT_FUNCTION ) \
        INIT_FUNCTION( Motor, Namespace, Init, const char* ) \
        INIT_FUNCTION( void, Namespace, End, Motor ) \
        INIT_FUNCTION( void, Namespace, Enable, Motor ) \
        INIT_FUNCTION( void, Namespace, Disable, Motor ) \
        INIT_FUNCTION( void, Namespace, Reset, Motor ) \
        INIT_FUNCTION( void, Namespace, SetOffset, Motor, double ) \
        INIT_FUNCTION( bool, Namespace, IsEnabled, Motor ) \
        INIT_FUNCTION( bool, Namespace, HasError, Motor ) \
        INIT_FUNCTION( void, Namespace, WriteControl, Motor, double )

DECLARE_NAMESPACE_INTERFACE( Motors, MOTOR_INTERFACE )


#endif  // MOTORS_H
