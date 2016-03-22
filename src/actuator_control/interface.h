#ifndef ACTUATOR_CONTROL_INTERFACE_H
#define ACTUATOR_CONTROL_INTERFACE_H

#include "interfaces.h"
#include "control_definitions.h"

#include "debug/async_debug.h"

#define ACTUATOR_CONTROL_FUNCTIONS( interface, function_init ) \
        function_init( Controller, interface, InitController, void ) \
        function_init( double*, interface, RunStep, Controller, double[ CONTROL_VARS_NUMBER ], double[ CONTROL_VARS_NUMBER ], double, double* ) \
        function_init( void, interface, EndController, Controller )

DEFINE_INTERFACE( ActuatorControl, ACTUATOR_CONTROL_FUNCTIONS )

#endif  // ACTUATOR_CONTROL_INTERFACE_H
