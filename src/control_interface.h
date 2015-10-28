#ifndef CONTROL_INTERFACE_H
#define CONTROL_INTERFACE_H

#include "interface.h"

#include "debug/async_debug.h"

#include <stdbool.h>
#include <math.h>

// Control used values enumerations

enum ControlVariables { CONTROL_POSITION, CONTROL_VELOCITY, CONTROL_FORCE, CONTROL_ACCELERATION = CONTROL_FORCE, CONTROL_VARS_NUMBER };

typedef double* (*ControlFunction)( double[ CONTROL_VARS_NUMBER ], double[ CONTROL_VARS_NUMBER ], double, double* );

#define CONTROL_FUNCTIONS( interface, function_init ) \
        function_init( double*, interface, Run, double[ CONTROL_VARS_NUMBER ], double[ CONTROL_VARS_NUMBER ], double, double* )

DEFINE_INTERFACE( Control, CONTROL_FUNCTIONS )

#endif  // CONTROL_INTERFACE_H
