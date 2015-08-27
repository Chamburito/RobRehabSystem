#ifndef ACTUATOR_INTERFACE_H
#define ACTUATOR_INTERFACE_H

#include "axis/axis_types.h"

typedef struct _Axis
{
  int ID;
  AxisInterface interface;
  double gearConversionFactor;
}
Axis;

typedef struct _ActuatorOperations
{
  int (*Init)( const char* );
  void (*End)( int );
  void (*Enable)( int );
  void (*Disable)( int );
  void (*Reset)( int );
  bool (*IsEnabled)( int );
  bool (*HasError)( int );
  void (*ReadAxes)( int );
  double (*GetMeasure)( int, enum AxisDimensions );
  void (*SetSetpoint)( int, double );
}
ActuatorOperations;

typedef ActuatorOperations* ActuatorInterface;

#endif // ACTUATOR_INTERFACE_H
