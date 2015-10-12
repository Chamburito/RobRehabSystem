#ifndef AXIS_INTERFACE_H
#define AXIS_INTERFACE_H

#ifndef M_PI
#define M_PI   3.14159265358979323846	/* pi */
#endif

#include "interface.h"

#include <stdbool.h>

enum AxisVariables { AXIS_ENCODER, AXIS_RPS, AXIS_CURRENT, AXIS_ANALOG, AXIS_VARS_NUMBER };

#define Axis( function_init ) \
        function_init( int, CANEPOS_Connect, unsigned int ) \
        function_init( void, CANEPOS_Disconnect, int ) \
        function_init( void, CANEPOS_Enable, int ) \
        function_init( void, CANEPOS_Disable, int ) \
        function_init( void, CANEPOS_Reset, int ) \
        function_init( bool, CANEPOS_IsEnabled, int ) \
        function_init( bool, CANEPOS_HasError, int ) \
        function_init( bool, CANEPOS_ReadMeasures, int, double[ AXIS_VARS_NUMBER ] ) \
        function_init( bool, CANEPOS_WriteSetpoint, int, double ) \
        function_init( void, CANEPOS_SetOperationMode, int, enum AxisVariables )

DEFINE_INTERFACE( Axis )

#endif // AXIS_INTERFACE_H
