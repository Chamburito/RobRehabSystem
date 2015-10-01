#ifndef AXIS_INTERFACE_H
#define AXIS_INTERFACE_H

#ifndef M_PI
#define M_PI   3.14159265358979323846	/* pi */
#endif

#include <stdbool.h>

enum AxisVariables { AXIS_ENCODER, AXIS_RPS, AXIS_CURRENT, AXIS_ANALOG, AXIS_VARS_NUMBER };

#define AXIS_INTERFACE( namespace ) \
        AXIS_FUNCTION( int, namespace, Connect, unsigned int ) \
        AXIS_FUNCTION( void, namespace, Disconnect, int ) \
        AXIS_FUNCTION( void, namespace, Enable, int ) \
        AXIS_FUNCTION( void, namespace, Disable, int ) \
        AXIS_FUNCTION( void, namespace, Reset, int ) \
        AXIS_FUNCTION( bool, namespace, IsEnabled, int ) \
        AXIS_FUNCTION( bool, namespace, HasError, int ) \
        AXIS_FUNCTION( bool, namespace, ReadMeasures, int, double[ AXIS_VARS_NUMBER ] ) \
        AXIS_FUNCTION( void, namespace, WriteSetpoint, int, double ) \
        AXIS_FUNCTION( void, namespace, SetOperationMode, int, enum AxisVariables )
        
#define AXIS_FUNCTION( rvalue, namespace, name, ... ) rvalue (*name)( __VA_ARGS__ );

typedef struct _AxisOperations
{
  /*int (*Connect)( const char* );
  void (*Disconnect)( int );
  void (*Enable)( int );
  void (*Disable)( int );
  void (*Reset)( int );
  bool (*IsEnabled)( int );
  bool (*HasError)( int );
  bool (*ReadMeasures)( int, double[ AXIS_MEASURES_NUMBER ] );
  void (*WriteSetpoint)( int, double );*/
  AXIS_INTERFACE( AXIS )
}
AxisOperations;

#undef AXIS_FUNCTION

typedef AxisOperations* AxisInterface;

#endif // AXIS_INTERFACE_H
