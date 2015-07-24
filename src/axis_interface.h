#ifndef AXIS_INTERFACE_H
#define AXIS_INTERFACE_H

typedef struct _AxisInterface
{
  void (*Enable)( int );
  void (*Disable)( int );
  void (*Reset)( int );
  bool (*IsEnabled)( int );
  bool (*HasError)( int );
  double (*GetMeasure)( int, int );
  void (*SetParameter)( int, int, double );
  double (*GetParameter)( int, int );
}
AxisInterface;

#endif // AXIS_INTERFACE_H
