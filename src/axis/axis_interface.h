#ifndef AXIS_INTERFACE_H
#define AXIS_INTERFACE_H

enum AxisDimensions { AXIS_POSITION, AXIS_VELOCITY, AXIS_FORCE, AXIS_DIMENSIONS_NUMBER };

typedef struct _AxisMethods
{  
  int (*Connect)( const char* );
  void (*Disconnect)( int );
  void (*Enable)( int );
  void (*Disable)( int );
  void (*Reset)( int );
  bool (*IsEnabled)( int );
  bool (*HasError)( int );
  bool (*ReadMeasures)( int, double[ AXIS_DIMENSIONS_NUMBER ] );
  void (*WriteControl)( int, double );
}
AxisMethods;

typedef AxisInterfaceMethods* AxisInterface;

#endif // AXIS_INTERFACE_H