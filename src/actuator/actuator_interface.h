#ifndef ACTUATOR_INTERFACE_H
#define ACTUATOR_INTERFACE_H

#include "interface.h"

#include "axis/axis_motor.h"
#include "axis/axis_sensor.h"

enum ActuatorVariables { ACTUATOR_POSITION, ACTUATOR_VELOCITY, ACTUATOR_FORCE, ACTUATOR_VARS_NUMBER };

#define ACTUATOR_INTERFACE( FUNCTION_INIT, namespace ) \
        FUNCTION_INIT( int, namespace, Init, const char* ) \
        FUNCTION_INIT( void, namespace, End, int ) \
        FUNCTION_INIT( void, namespace, Enable, int ) \
        FUNCTION_INIT( void, namespace, Disable, int ) \
        FUNCTION_INIT( void, namespace, Reset, int ) \
        FUNCTION_INIT( void, namespace, Calibrate, int ) \
        FUNCTION_INIT( bool, namespace, IsEnabled, int ) \
        FUNCTION_INIT( bool, namespace, HasError, int ) \
        FUNCTION_INIT( double*, namespace, ReadAxes, int ) \
        FUNCTION_INIT( void, namespace, SetSetpoint, int, double ) \
        FUNCTION_INIT( void, namespace, SetOperationMode, int, enum ActuatorVariables )
        
typedef struct _ActuatorOperations { ACTUATOR_INTERFACE( INIT_NAMESPACE_POINTER, Actuator ) } ActuatorOperations;

typedef ActuatorOperations* ActuatorInterface;

#endif // ACTUATOR_INTERFACE_H
