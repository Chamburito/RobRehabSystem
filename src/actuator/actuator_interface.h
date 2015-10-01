#ifndef ACTUATOR_INTERFACE_H
#define ACTUATOR_INTERFACE_H

#include "interface.h"

#include "axis/axis_motor.h"
#include "axis/axis_sensor.h"

#define Actuator( function_init ) \
        function_init( int, Init, const char* ) \
        function_init( void, End, int ) \
        function_init( void, Enable, int ) \
        function_init( void, Disable, int ) \
        function_init( void, Reset, int ) \
        function_init( void, Calibrate, int ) \
        function_init( bool, IsEnabled, int ) \
        function_init( bool, HasError, int ) \
        function_init( double*, ReadAxes, int ) \
        function_init( void, SetSetpoint, int, double ) \
        function_init( void, SetOperationMode, int, enum AxisVariables )
        
/*typedef struct _ActuatorOperations { ACTUATOR_INTERFACE( INIT_NAMESPACE_POINTER, Actuator ) } ActuatorOperations;

typedef ActuatorOperations* ActuatorInterface;*/

DEFINE_INTERFACE( Actuator )

#endif // ACTUATOR_INTERFACE_H
