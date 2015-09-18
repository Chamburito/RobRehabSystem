#ifndef AXIS_INTERFACE_H
#define AXIS_INTERFACE_H

#include <stdbool.h>

enum AxisMeasures { AXIS_MEASURE_ENCODER, AXIS_MEASURE_VELOCITY, AXIS_MEASURE_CURRENT, AXIS_MEASURE_TENSION, AXIS_MEASURES_NUMBER };
enum AxisSetpoints { AXIS_SETPOINT_ENCODER, AXIS_SETPOINT_VELOCITY, AXIS_SETPOINT_CURRENT, AXIS_SETPOINTS_NUMBER };

typedef struct _AxisOperations
{  
  int (*Connect)( const char* );
  void (*Disconnect)( int );
  void (*Enable)( int );
  void (*Disable)( int );
  void (*Reset)( int );
  bool (*IsEnabled)( int );
  bool (*HasError)( int );
  bool (*ReadMeasures)( int, double[ AXIS_MEASURES_NUMBER ] );
  void (*WriteSetpoints)( int, double[ AXIS_SETPOINTS_NUMBER ] );
}
AxisOperations;

typedef AxisOperations* AxisInterface;

#endif // AXIS_INTERFACE_H
