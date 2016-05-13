#ifndef CONTROL_DEFINITIONS_H
#define CONTROL_DEFINITIONS_H

#define CONTROLLER_INVALID_HANDLE NULL

#define CONTROL_PASS_INTERVAL 0.005

typedef void* Controller;

// Control used values structure (ONLY ADD/REMOVE DOUBLES !!)
// As this struct only contains 64 bits (8 bytes) values, we can access its members like an array as well
// Because data alignment in memory is always done with power-of-2 number of bytes, so we should have no issues with structure padding 
typedef struct
{
  double position;
  double velocity;
  double force;
  double acceleration;
  double stiffness;
  double damping;
}
ControlVariables;

//#define CONTROL_VARS_NUMBER sizeof(ControlVariables)/sizeof(double)

// Control used values enumeration
enum ControlVariable { CONTROL_POSITION, CONTROL_VELOCITY, CONTROL_FORCE, CONTROL_ACCELERATION, CONTROL_MODES_NUMBER,
                       CONTROL_STIFFNESS = CONTROL_MODES_NUMBER, CONTROL_DAMPING, CONTROL_VARS_NUMBER }; 

#endif // CONTROL_DEFINITIONS_H
