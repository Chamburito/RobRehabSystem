#ifndef ACTUATOR_H
#define ACTUATOR_H 

#include "namespaces.h"

#include "control_definitions.h"


typedef struct _ActuatorData ActuatorData;
typedef ActuatorData* Actuator;

#define ACTUATOR_INTERFACE( Namespace, INIT_FUNCTION ) \
        INIT_FUNCTION( Actuator, Namespace, Init, const char* ) \
        INIT_FUNCTION( void, Namespace, End, Actuator ) \
        INIT_FUNCTION( void, Namespace, Enable, Actuator ) \
        INIT_FUNCTION( void, Namespace, Disable, Actuator ) \
        INIT_FUNCTION( bool, Namespace, Reset, Actuator ) \
        INIT_FUNCTION( bool, Namespace, ToggleOffset, Actuator ) \
        INIT_FUNCTION( bool, Namespace, ToggleCalibration, Actuator ) \
        INIT_FUNCTION( bool, Namespace, IsEnabled, Actuator ) \
        INIT_FUNCTION( bool, Namespace, HasError, Actuator ) \
        INIT_FUNCTION( double, Namespace, SetSetpoint, Actuator, enum ControlVariable, double ) \
        INIT_FUNCTION( ControlVariables*, Namespace, UpdateMeasures, Actuator, ControlVariables* ) \
        INIT_FUNCTION( double, Namespace, RunControl, Actuator, ControlVariables*, ControlVariables* )

DECLARE_NAMESPACE_INTERFACE( Actuators, ACTUATOR_INTERFACE )


#endif // ACTUATOR_H
