const struct { size_t POSITION, VELOCITY, ACCELERATION, TORQUE, NUMBER; } AxisMeasures = { 0, 1, 2, 3, 4 };
const struct { size_t POSITION, VELOCITY, NUMBER; } AxisOperationModes = { 0, 1, 2 };

typedef struct _AxisControlInterface
{  
  int (*Connect)( int );
  void (*Disconnect)( int );
  void (*Enable)( int );
  void (*Disable)( int );
  double* (*ReadMeasures)( int );
  void (*WriteControl)( int, double );
  void (*SetOperationMode)( int, int );
}
AxisControlInterface;
