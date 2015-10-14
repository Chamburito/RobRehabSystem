#ifndef AXIS_INTERFACE_H
#define AXIS_INTERFACE_H

#ifndef M_PI
#define M_PI   3.14159265358979323846	/* pi */
#endif

#include "interface.h"

#include <stdbool.h>

enum AxisVariables { AXIS_ENCODER, AXIS_RPS, AXIS_CURRENT, AXIS_ANALOG, AXIS_VARS_NUMBER };

#define AXIS_INTERFACE_FUNCTIONS( interface, function_init ) \
        function_init( int, interface, Connect, unsigned int ) \
        function_init( void, interface, Disconnect, int ) \
        function_init( void, interface, Enable, int ) \
        function_init( void, interface, Disable, int ) \
        function_init( void, interface, Reset, int ) \
        function_init( bool, interface, IsEnabled, int ) \
        function_init( bool, interface, HasError, int ) \
        function_init( bool, interface, ReadMeasures, int, double[ AXIS_VARS_NUMBER ] ) \
        function_init( bool, interface, WriteSetpoint, int, double ) \
        function_init( void, interface, SetOperationMode, int, enum AxisVariables )

DEFINE_INTERFACE( Axis, AXIS_INTERFACE_FUNCTIONS )

#endif // AXIS_INTERFACE_H
