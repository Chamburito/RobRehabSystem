#ifndef ACTUATOR_INTERFACE_H
#define ACTUATOR_INTERFACE_H

#include "axis_interface.h"
#include "ni_can_epos_axis/axis_interface.h"

typedef struct _Axis
{
  AxisInterface* interface;
  int interfaceID;
  double measuresList[ AXIS_DIMENSIONS_NUMBER ];
  double measureRatiosList[ AXIS_DIMENSIONS_NUMBER ];
  int operationMode;
}
Axis;

typedef struct _ActuatorInterface
{
  int (*Init)( const char* );
  void (*End)( int );
  void (*Enable)( int );
  void (*Disable)( int );
  void (*Reset)( int );
  bool (*IsEnabled)( int );
  bool (*HasError)( int );
  void (*SetOption)( int, const char*, void* );
  double (*GetMeasure)( int, enum AxisDimensions );
  void (*SetSetpoint)( int, double );
  void (*ReadAxes)( int );
  void (*WriteControl)( int );
  void (*SetOperationMode)( int, enum AxisDimensions );
}
ActuatorInterface;

#endif // ACTUATOR_INTERFACE_H
