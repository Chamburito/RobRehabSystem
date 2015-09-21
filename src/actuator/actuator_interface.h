#ifndef ACTUATOR_INTERFACE_H
#define ACTUATOR_INTERFACE_H

#include "axis/axis_motor.h"
#include "axis/axis_sensor.h"

/*typedef struct _Axis
{
  int ID;
  AxisInterface interface;
  double gearConversionFactor;
}
Axis;*/

enum ActuatorVariables { ACTUATOR_POSITION, ACTUATOR_VELOCITY, ACTUATOR_FORCE, ACTUATOR_VARS_NUMBER };

#define ACTUATOR_INTERFACE( namespace ) \
        ACTUATOR_FUNCTION( int, namespace, Init, const char* ) \
        ACTUATOR_FUNCTION( void, namespace, End, int ) \
        ACTUATOR_FUNCTION( void, namespace, Enable, int ) \
        ACTUATOR_FUNCTION( void, namespace, Disable, int ) \
        ACTUATOR_FUNCTION( void, namespace, Reset, int ) \
        ACTUATOR_FUNCTION( void, namespace, Calibrate, int ) \
        ACTUATOR_FUNCTION( bool, namespace, IsEnabled, int ) \
        ACTUATOR_FUNCTION( bool, namespace, HasError, int ) \
        ACTUATOR_FUNCTION( double*, namespace, ReadAxes, int ) \
        ACTUATOR_FUNCTION( void, namespace, SetSetpoint, int, double ) \
        ACTUATOR_FUNCTION( void, namespace, SetOperationMode, int, enum ActuatorVariables )
        
#define ACTUATOR_FUNCTION( rvalue, namespace, name, ... ) rvalue (*name)( __VA_ARGS__ );

typedef struct _ActuatorOperations
{
  /*int (*Init)( const char* );
  void (*End)( int );
  void (*Enable)( int );
  void (*Disable)( int );
  void (*Reset)( int );
  void (*Calibrate)( int );
  bool (*IsEnabled)( int );
  bool (*HasError)( int );
  double* (*ReadAxes)( int );
  void (*SetSetpoint)( int, double );*/
  ACTUATOR_INTERFACE( ACTUATOR )
}
ActuatorOperations;

typedef ActuatorOperations* ActuatorInterface;

#endif // ACTUATOR_INTERFACE_H
