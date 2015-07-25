enum AxisMeasures { AXIS_MEASURES_POSITION, AXIS_MEASURES_VELOCITY, AXIS_MEASURES_ACCELERATION, AXIS_MEASURES_TORQUE, AXIS_MEASURES_NUMBER };
enum AxisOperationModes { AXIS_OP_MODES_POSITION, AXIS_OP_MODES_VELOCITY, AXIS_OP_MODES_NUMBER };

typedef struct _AxisControlInterface
{  
  int (*Connect)( int );
  void (*Disconnect)( int );
  void (*Enable)( int );
  void (*Disable)( int );
  void (*Reset)( int );
  bool (*IsEnabled)( int );
  bool (*HasError)( int );
  bool (*ReadMeasures)( int, double[ AXIS_MEASURES_NUMBER ] );
  void (*WriteControl)( int, double[ AXIS_OP_MODES_NUMBER ] );
  void (*SetOperationMode)( int, int );
}
AxisControlInterface;
