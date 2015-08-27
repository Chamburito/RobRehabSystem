#ifndef IMPEDANCE_CONTROL_H
#define IMPEDANCE_CONTROL_H

#include <math.h>

// Control used values enumerations
enum { CONTROL_POSITION, CONTROL_VELOCITY, CONTROL_ACCELERATION, CONTROL_FORCE, CONTROL_ERROR, CONTROL_MEASURES_NUMBER };
enum { CONTROL_REFERENCE, CONTROL_STIFFNESS, CONTROL_DAMPING, CONTROL_SETPOINTS_NUMBER };

typedef double (*ImpedanceControlFunction)( double[ CONTROL_MEASURES_NUMBER ], double[ CONTROL_SETPOINTS_NUMBER ], double );

// Control algorhitms
static double RunDefaultControl( double[ CONTROL_MEASURES_NUMBER ], double[ CONTROL_SETPOINTS_NUMBER ], double );
static double RunForcePIControl( double[ CONTROL_MEASURES_NUMBER ], double[ CONTROL_SETPOINTS_NUMBER ], double );
static double RunVelocityControl( double[ CONTROL_MEASURES_NUMBER ], double[ CONTROL_SETPOINTS_NUMBER ], double );

// Available precompiled control methods
struct IndexedFunction
{
  char* name;                                   // Identifier string
  ImpedanceControlFunction ref_RunControl;      // Control pass algorithm (function pointer)
}
controlFunctionsList[] = { { "Default", RunDefaultControl }, { "Impedance_Force_PI", RunForcePIControl }, { "Impedance_Velocity", RunVelocityControl } };

static size_t functionsNumber = sizeof(controlFunctionsList) / sizeof(struct IndexedFunction);

ImpedanceControlFunction ImpedanceControl_GetFunction( const char* functionName )
{
  for( size_t functionID = 0; functionID < functionsNumber; functionID++ )
  {
    if( strcmp( functionName, controlFunctionsList[ functionID ].name ) == 0 )
      return controlFunctionsList[ functionID ].ref_RunControl;
  }
  
  return NULL;
}

/////////////////////////////////////////////////////////////////////////////////
/////                            HIPS CONSTANTS                             /////
/////////////////////////////////////////////////////////////////////////////////

const double TNS_2_POS_RATIO = 0.0168; // mm/mV
const double TNS_2_POS_OFFSET = 2.5749; // mm

const double HIPS_FORCE_SENSOR_STIFFNESS = 78.9; // N/mm
const double HIPS_ACTUATOR_STEP = 3e-3; // Linear effector advance per screw rotation (in meters/rotation)

// Orthosis structure bars and diagonals lengths (in meters)
enum { A, B, C, D, C2D, A2B };
//const double HIPS_STRUCT_DIMS[] = { 0.063, 0.060, 0.284, 0.117, sqrt( 0.284 * 0.284 + 0.117 * 0.117 ), sqrt( 0.063 * 0.063 + 0.060 * 0.060 ) };
const double HIPS_STRUCT_DIMS[] = { 0.063, 0.060, 0.284, 0.117, 0.307, 0.087 };
// Actuator length with effector at initial position (in meters)
const double HIPS_EFFECTOR_BASE_LENGTH = 0.348;

enum { ALPHA, BETA };
//const double HIPS_STRUCT_ANGLES[] = { atan( HIPS_STRUCT_DIMS[ A ] / HIPS_STRUCT_DIMS[ B ] ), atan( HIPS_STRUCT_DIMS[ C ] / HIPS_STRUCT_DIMS[ D ] ) };
const double HIPS_STRUCT_ANGLES[] = { 0.810, 1.180 };

/////////////////////////////////////////////////////////////////////////////////
/////                    HIPS CONTROL DEFAULT VALUES                        /////
/////////////////////////////////////////////////////////////////////////////////

// Sampling
const int HIPS_CONTROL_SAMPLING_NUMBER = 6;

/////////////////////////////////////////////////////////////////////////////////
/////                         HIPS CONTROL FUNCTION                         /////
/////////////////////////////////////////////////////////////////////////////////

static double RunVelocityControl( double measuresList[ CONTROL_MEASURES_NUMBER ], double parametersList[ CONTROL_SETPOINTS_NUMBER ], double deltaTime )
{
  /*static double sensorTension_0, sensorPosition_0;

  // Sampling of last voltage signal values for filtering (average filter)
  static double analogSamples[ HIPS_CONTROL_SAMPLING_NUMBER ];

  static double positionInSetpoint;
    
  if( sensorTension_0 == 0 )
  {
    sensorTension_0 = MotorDrive_GetMeasure( hipsControl->sensor, AXIS_TENSION ); // mV
    sensorPosition_0 = ( TNS_2_POS_RATIO * sensorTension_0 + TNS_2_POS_OFFSET ); // mm  
  }
  
  double sensorTension = 0;
  for( int i = 0; i < HIPS_CONTROL_SAMPLING_NUMBER; i ++ )
  {
    analogSamples[ i ] = ( i == 0 ) ? MotorDrive_GetMeasure( hipsControl->sensor, AXIS_TENSION ) : analogSamples[ i - 1 ]; // mV
    sensorTension += analogSamples[ i ] / HIPS_CONTROL_SAMPLING_NUMBER;
  }

  double sensorPosition = ( TNS_2_POS_RATIO * sensorTension + TNS_2_POS_OFFSET );	// mm

  double actuatorForce = -HIPS_FORCE_SENSOR_STIFFNESS * ( sensorPosition_0 - sensorPosition ); // N

  //Motor_SetSetpoint( hipsControl->actuator, MOTOR_POSITION_SETPOINT, hipsControl->setpoint / ( 2 * PI ) );
  //positionInSetpoint = Motor_GetSetpoint( hipsControl->actuator, MOTOR_POSITION_SETPOINT ) * ( 2 * PI );

  // Relation between axis rotation and effector linear displacement
  double actuatorEncoderPosition = controlData->position; //MotorDrive_GetMeasure( hipsControl->sensor, AXIS_POSITION );
  double effectorDeltaLength = actuatorEncoderPosition * HIPS_ACTUATOR_STEP + ( sensorPosition_0 - sensorPosition ) / 1000; // m

  double effectorLength = HIPS_EFFECTOR_BASE_LENGTH + effectorDeltaLength; // m

  double cos_eta = ( pow( effectorLength, 2 ) + pow( HIPS_STRUCT_DIMS[ A2B ], 2 ) - pow( HIPS_STRUCT_DIMS[ C2D ], 2 ) ) / ( 2 * effectorLength * HIPS_STRUCT_DIMS[ A2B ] );
  
  double sin_eta = sqrt( (double)( 1 - pow( cos_eta, 2 ) ) );

  double gama = acos( ( pow( HIPS_STRUCT_DIMS[ C2D ], 2 ) + pow( HIPS_STRUCT_DIMS[ A2B ], 2 ) - pow( effectorLength, 2 ) ) / ( 2 * HIPS_STRUCT_DIMS[ C2D ] * HIPS_STRUCT_DIMS[ A2B ] ) );

  //double theta = HIPS_STRUCT_ANGLES[ ALPHA ] + HIPS_STRUCT_ANGLES[ BETA ] + gama - PI;

  // Current theta derivative with respect to the effector total length (base + delta)
  double lengthToThetaRatio = sqrt( pow( HIPS_STRUCT_DIMS[ C2D ], 2 ) + pow( HIPS_STRUCT_DIMS[ A2B ], 2 ) - 2 * HIPS_STRUCT_DIMS[ C2D ] * HIPS_STRUCT_DIMS[ A2B ] * cos( gama ) )
								                  / ( HIPS_STRUCT_DIMS[ C2D ] * HIPS_STRUCT_DIMS[ A2B ] * sin( gama ) );

  // Impedance Control Virtual Stiffness
  if( controlData->stiffness < 1.0 ) controlData->stiffness = 1.0;
  // Impedance Control Virtual Damping
  if( controlData->damping < 10.0 ) controlData->damping = 10.0;

  double actuatorStiffness = controlData->stiffness * lengthToThetaRatio * ( 1 / ( HIPS_STRUCT_DIMS[ A2B ] * sin_eta ) );
  double actuatorDamping = controlData->damping * lengthToThetaRatio * ( 1 / ( HIPS_STRUCT_DIMS[ A2B ] * sin_eta ) );

  double Fv_q = ( controlData->stiffness * positionInSetpoint ) / ( HIPS_STRUCT_DIMS[ A2B ] * sin_eta );

  // Control law
  double velocitySetpoint = ( Fv_q - actuatorStiffness * ( actuatorEncoderPosition * HIPS_ACTUATOR_STEP )
						                + ( ( actuatorStiffness - HIPS_FORCE_SENSOR_STIFFNESS ) / HIPS_FORCE_SENSOR_STIFFNESS ) * actuatorForce ) / actuatorDamping;

  int velocityRPMSetpoint = ( velocitySetpoint / HIPS_ACTUATOR_STEP ) * 60 / ( 2 * PI ); // rpm

  //Motor_SetSetpoint( hipsControl->actuator, MOTOR_VELOCITY_SETPOINT, velocityRPMSetpoint );
  return velocityRPMSetpoint;*/
  return 0.0;
}


/////////////////////////////////////////////////////////////////////////////////
/////                            KNEE CONSTANTS                             /////
/////////////////////////////////////////////////////////////////////////////////

// Torque Filter
const double a2 = -0.9428;
const double a3 = 0.3333;
const double b1 = 0.0976;
const double b2 = 0.1953;
const double b3 = 0.0976;

// Angular Velocity Filter
const double c2 = -1.889;
const double c3 = 0.8949;
const double d1 = 0.0015;
const double d2 = 0.0029;
const double d3 = 0.0015;

/////////////////////////////////////////////////////////////////////////////////
/////                          KNEE CONTROL FUNCTION                        /////
/////////////////////////////////////////////////////////////////////////////////

static double RunForcePIControl( double measuresList[ CONTROL_MEASURES_NUMBER ], double parametersList[ CONTROL_SETPOINTS_NUMBER ], double deltaTime )
{
  static double position, positionSetpoint, positionError, positionErrorSum, positionSetpointSum;
  static double velocity[3], velocityFiltered[3], velocitySetpoint[3];
  static double force[3], forceFiltered[3], forceError[3];
  
  static double error;
  
  if( measuresList[ CONTROL_ERROR ] == 0.0 )
  {
    /*error = ( positionSetpointSum > 0.0 ) ? positionErrorSum / positionSetpointSum : 1.0;
    if( error > 1.0 )*/ error = 1.0;
    
    positionErrorSum = 0.0;
    positionSetpointSum = 0.0;
  }

  // Impedance Control Vitual Stiffness
  parametersList[ CONTROL_STIFFNESS ] *= error;

  velocity[0] = ( measuresList[ CONTROL_POSITION ] - position ) / deltaTime;
  position = measuresList[ CONTROL_POSITION ];

  velocityFiltered[0] = -c2 * velocityFiltered[1] - c3 * velocityFiltered[2] + d1 * velocity[0] + d2 * velocity[1] + d3 * velocity[2];
  
  positionSetpoint = parametersList[ CONTROL_REFERENCE ];
  positionError = position - positionSetpoint;
  positionErrorSum += deltaTime * positionError * positionError;
  positionSetpointSum += deltaTime * positionSetpoint * positionSetpoint;
  
  measuresList[ CONTROL_ERROR ] = positionError / positionSetpoint;
  
  double forceSetpoint = -parametersList[ CONTROL_STIFFNESS ] * positionError - parametersList[ CONTROL_DAMPING ] * velocityFiltered[0];

  force[0] = measuresList[ CONTROL_FORCE ];
  forceFiltered[0] = -a2 * forceFiltered[1] - a3 * forceFiltered[2] + b1 * force[0] + b2 * force[1] + b3 * force[2];
  
  forceError[0] = forceSetpoint - forceFiltered[0];

  velocitySetpoint[0] += 370.0 * ( forceError[0] - forceError[1] ) + 3.5 * deltaTime * forceError[0];
  
  //velocitySetpoint[0] = 0.9822 * velocitySetpoint[1] + 0.01407 * velocitySetpoint[2] + 338.6 * forceError[1] - 337.4 * forceError[2]; //5ms
  
  for( int i = 2; i > 0; i-- )
  {
    forceError[ i ] = forceError[ i - 1 ];
    velocity[ i ] = velocity[ i - 1 ];
    velocityFiltered[ i ] = velocityFiltered[ i - 1 ];
    forceFiltered[ i ] = forceFiltered[ i - 1 ];
    velocitySetpoint[ i ] = velocitySetpoint[ i - 1 ];
  }
  
  return velocitySetpoint[0];
}

/////////////////////////////////////////////////////////////////////////////////
/////                        DEFAULT CONTROL FUNCTION                       /////
/////////////////////////////////////////////////////////////////////////////////

static double RunDefaultControl( double measuresList[ CONTROL_MEASURES_NUMBER ], double parametersList[ CONTROL_SETPOINTS_NUMBER ], double deltaTime )
{
  return parametersList[ CONTROL_MEASURES_NUMBER ];
}

#endif  /* IMPEDANCE_CONTROL_H */
