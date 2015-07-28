#ifndef AXIS_INTERFACE_H
#define AXIS_INTERFACE_H

enum AxisDimensions { AXIS_POSITION, AXIS_VELOCITY, AXIS_TORQUE, AXIS_DIMENSIONS_NUMBER };

typedef struct _AxisInterface
{  
  int (*Connect)( int );
  void (*Disconnect)( int );
  void (*Enable)( int );
  void (*Disable)( int );
  void (*Reset)( int );
  bool (*IsEnabled)( int );
  bool (*HasError)( int );
  bool (*ReadMeasures)( int, double[ AXIS_DIMENSIONS_NUMBER ] );
  void (*WriteControl)( int, double[ AXIS_DIMENSIONS_NUMBER ] );
  void (*SetOperationMode)( int, enum AxisOperationModes );
}
AxisInterface;

#endif // AXIS_INTERFACE_H