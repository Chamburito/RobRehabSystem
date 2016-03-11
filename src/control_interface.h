#ifndef CONTROL_INTERFACE_H
#define CONTROL_INTERFACE_H

#include "interfaces.h"

#include "debug/async_debug.h"

#define CONTROLLER_INVALID NULL

typedef void* Controller;

// Control used values enumeration
enum ControlVariables { CONTROL_POSITION, CONTROL_VELOCITY, CONTROL_FORCE, CONTROL_ACCELERATION = CONTROL_FORCE, CONTROL_VARS_NUMBER };

//typedef double* (*ControlFunction)( double[ CONTROL_VARS_NUMBER ], double[ CONTROL_VARS_NUMBER ], double, double* );

#define CONTROL_FUNCTIONS( interface, function_init ) \
        function_init( Controller, interface, InitController, void ) \
        function_init( double*, interface, RunStep, Controller, double[ CONTROL_VARS_NUMBER ], double[ CONTROL_VARS_NUMBER ], double, double* ) \
        function_init( void, interface, EndController, Controller )

DEFINE_INTERFACE( Control, CONTROL_FUNCTIONS )

#endif  // CONTROL_INTERFACE_H
