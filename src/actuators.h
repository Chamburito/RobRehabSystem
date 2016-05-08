#ifndef ACTUATOR_H
#define ACTUATOR_H 

#include "interfaces.h"

#include "control_definitions.h"

typedef struct _ActuatorData ActuatorData;
typedef ActuatorData* Actuator;


#define ACTUATOR_INTERFACE( namespace, function_init ) \
        function_init( Actuator, namespace, Init, const char* ) \
        function_init( void, namespace, End, Actuator ) \
        function_init( void, namespace, Enable, Actuator ) \
        function_init( void, namespace, Disable, Actuator ) \
        function_init( void, namespace, Reset, Actuator ) \
        function_init( void, namespace, SetOffset, Actuator ) \
        function_init( void, namespace, Calibrate, Actuator ) \
        function_init( bool, namespace, IsEnabled, Actuator ) \
        function_init( bool, namespace, HasError, Actuator ) \
        function_init( double, namespace, SetSetpoint, Actuator, enum ControlVariables, double ) \
        function_init( double*, namespace, UpdateMeasures, Actuator, double* ) \
        function_init( double, namespace, RunControl, Actuator, double*, double* )

DECLARE_NAMESPACE_INTERFACE( Actuators, ACTUATOR_INTERFACE )


#endif // ACTUATOR_H
