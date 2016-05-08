#ifndef ACTUATOR_CONTROL_INTERFACE_H
#define ACTUATOR_CONTROL_INTERFACE_H

#include "modules.h"
#include "control_definitions.h"

#define ACTUATOR_CONTROL_INTERFACE( Namespace, INIT_FUNCTION ) \
        INIT_FUNCTION( Controller, Namespace, InitController, void ) \
        INIT_FUNCTION( double*, Namespace, RunControlStep, Controller, double*, double*, double, double* ) \
        INIT_FUNCTION( void, Namespace, EndController, Controller )

#endif  // ACTUATOR_CONTROL_INTERFACE_H
