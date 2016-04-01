#ifndef ACTUATOR_CONTROL_INTERFACE_H
#define ACTUATOR_CONTROL_INTERFACE_H

#include "interfaces.h"
#include "control_definitions.h"

#include "debug/async_debug.h"

#define ACTUATOR_CONTROL_INTERFACE( interface, function_init ) \
        function_init( Controller, interface, InitController, void ) \
        function_init( double*, interface, RunControlStep, Controller, double*, double*, double, double* ) \
        function_init( void, interface, EndController, Controller )

#endif  // ACTUATOR_CONTROL_INTERFACE_H
