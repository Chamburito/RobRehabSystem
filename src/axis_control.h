#ifndef CONTROL_H
#define CONTROL_H

#include "axis.h"

#include "async_debug.h"

#include <math.h>
  
enum Joint { ANKLE, KNEE, HIPS, AXIS_CONTROL_N_JOINTS };  // Control algorhitms/axes indexes

/////////////////////////////////////////////////////////////////////////////////
/////                            CONTROL DEVICES                            /////
/////////////////////////////////////////////////////////////////////////////////

//CAN database addressing
const char* CAN_DATABASE = "database";
const char* CAN_CLUSTER = "NETCAN";

static Motor* motorList[ AXIS_CONTROL_N_JOINTS ];           // EPOS active axes list
static Encoder* sensorList[ AXIS_CONTROL_N_JOINTS ];        // EPOS passive (measurement) axes list

// Control algorhitms
static void AnkleControl( Motor* ankleActuator, Encoder* ankleSensor );
static void KneeControl( Motor*, Encoder* );
static void HipsControl( Motor* hipsActuator, Encoder* hipsSensor );

typedef void (*ControlFunction)( Motor*, Encoder* ); // Use "async_function" type as "void (*)( Motor*, Encoder* )" type
// Array of function pointers. Equivalent to void (*control_functions[ AXIS_CONTROL_N_JOINTS ])( Motor*, Encoder* )
static ControlFunction controlFunctions[ AXIS_CONTROL_N_JOINTS ] = { AnkleControl, KneeControl, HipsControl }; 

static void* AsyncControl( void* ); 

// EPOS devices and CAN network initialization
void AxisControl_Init()
{
  DEBUG_EVENT( 0, "Initializing Axis Control on thread %x", THREAD_ID );
  
  EposNetwork_Start( CAN_DATABASE, CAN_CLUSTER );
  
  // Start CAN network transmission
  //for( uint8_t nodeID = 1; nodeID <= AXIS_CONTROL_N_JOINTS; nodeID++ )
  //  EposNetwork_InitNode( nodeID );
  
  for( size_t jointID = 0; jointID < AXIS_CONTROL_N_JOINTS; jointID++ )
  {
    motorList[ jointID ] = NULL;
    sensorList[ jointID ] = NULL;
  }
  
  /* Specific initialization code */
  // motorList[ ANKLE ] = Motor_Connect( "Ankle", 4, 4096 ); // ?
  //motorList[ KNEE ] = Motor_Connect( "Knee", 2, 4096 );
  //sensorList[ KNEE ] = Encoder_Connect( 1, 2000 );
  //motorList[ HIPS ] = Motor_Connect( "Hips", 3, 4096 );
  
  motorList[ ANKLE ] = Motor_Connect( "Teste", 1, 4096 );
  
  // Start a control thread for each motor
  for( size_t controlMode = 0; controlMode < AXIS_CONTROL_N_JOINTS; controlMode++ )
    (void) Thread_Start( AsyncControl, (void*) controlMode, DETACHED );
  
  Timing_Delay( 200 );
  Motor_Enable( motorList[ ANKLE ] );
  Motor_SetParameter( motorList[ ANKLE ], POSITION_SETPOINT, 0.6 );
  
  DEBUG_EVENT( 0, "Axis Control initialized on thread %x", THREAD_ID );
}

// EPOS devices and CAN network shutdown
void AxisControl_End()
{
  DEBUG_EVENT( 0, "Ending Axis Control on thread %x", THREAD_ID );

  Timing_Delay( 2000 );
  
  // Destroy axes data structures
  for( size_t axisID = 0; axisID < AXIS_CONTROL_N_JOINTS; axisID++ )
  {
    Motor_Disconnect( motorList[ axisID ] );
    motorList[ axisID ] = NULL;
    Encoder_Disconnect( sensorList[ axisID ] );
    sensorList[ axisID ] = NULL;
  }
  
  // End CAN network transmission
  EposNetwork_Stop();
  
  DEBUG_EVENT( 0, "Axis Control ended on thread %x", THREAD_ID );
}

extern inline void AxisControl_EnableMotor( size_t motorID )
{
  if( motorID < 0 || motorID >= AXIS_CONTROL_N_JOINTS ) return;
  
  if( motorList[ motorID ] != NULL )
    Motor_Enable( motorList[ motorID ] );
}

extern inline void AxisControl_DisableMotor( size_t motorID )
{
  if( motorID < 0 || motorID >= AXIS_CONTROL_N_JOINTS ) return;
  
  if( motorList[ motorID ] != NULL )
    Motor_Disable( motorList[ motorID ] );
}

extern inline void AxisControl_ResetMotor( size_t motorID )
{
  if( motorID < 0 || motorID >= AXIS_CONTROL_N_JOINTS ) return;
  
  if( motorList[ motorID ] != NULL )
    Motor_Reset( motorList[ motorID ] );
}

extern inline void AxisControl_ConfigMotor( size_t motorID, double* parametersList )
{
  if( motorID < 0 || motorID >= AXIS_CONTROL_N_JOINTS ) return;
  
  if( motorList[ motorID ] == NULL ) return;
  
  for( size_t parameterIndex = 0; parameterIndex < AXIS_N_PARAMS; parameterIndex++ )
  {
    if( parametersList[ parameterIndex ] != 0.0 )
      Motor_SetParameter( motorList[ motorID ], parameterIndex, parametersList[ parameterIndex ] );
  }
}

bool* AxisControl_GetMotorStatus( size_t motorID )
{
  static bool statesList[ AXIS_N_STATES ];
  
  if( motorID < 0 || motorID >= AXIS_CONTROL_N_JOINTS ) return NULL;
  
  if( motorList[ motorID ] == NULL ) return NULL;
  
  for( size_t stateIndex = 0; stateIndex < AXIS_N_STATES; stateIndex++ )
    statesList[ stateIndex ] = Encoder_CheckState( motorList[ motorID ]->encoder, stateIndex );
  
  return statesList;
}

extern inline double* AxisControl_GetMotorMeasures( size_t motorID )
{
  static double measuresList[ AXIS_N_DIMS ];
  
  if( motorID < 0 || motorID >= AXIS_CONTROL_N_JOINTS ) return NULL;
  
  if( motorList[ motorID ] == NULL ) return NULL;
  
  for( size_t dimensionIndex = 0; dimensionIndex < AXIS_N_DIMS; dimensionIndex++ )
    measuresList[ dimensionIndex ] = Encoder_GetMeasure( motorList[ motorID ]->encoder, dimensionIndex );
  
  return measuresList;
}

extern inline const char* AxisControl_GetMotorName( size_t motorID )
{
  if( motorID < 0 || motorID >= AXIS_CONTROL_N_JOINTS ) return NULL;
  
  if( motorList[ motorID ] == NULL ) return NULL;
  
  return motorList[ motorID ]->name;
}

/////////////////////////////////////////////////////////////////////////////////
/////                           COMMON CONSTANTS                            /////
/////////////////////////////////////////////////////////////////////////////////

#ifndef PI
const double PI = 3.141592;        // Duh...
#endif

const double CONTROL_SAMPLING_INTERVAL = 0.005;           // Sampling interval

// Method that runs the control functions asyncronously
static void* AsyncControl( void* args )
{
  unsigned long controlMode = (unsigned long) args;
  
  ControlFunction ref_AxisControl = controlFunctions[ controlMode ];
  
  unsigned int execTime, elapsedTime;
  
  // Verify and try to correct errors
  if( motorList[ controlMode ] != NULL )
  {
    Motor_Reset( motorList[ controlMode ] );
    Motor_SetOperationMode( motorList[ controlMode ], POSITION_MODE );
  }
  
  DEBUG_EVENT( 0, "starting to run control for Axis %s", motorList[ controlMode ]->name );
  
  while( motorList[ controlMode ] != NULL )
  {
    DEBUG_UPDATE( "running control for Axis %s", motorList[ controlMode ]->name );
    execTime = Timing_GetExecTimeMilliseconds();

    Encoder_ReadValues( motorList[ controlMode ]->encoder );
    
    // If the motor is being actually controlled, call control pass algorhitm
    if( motorList[ controlMode ]->active ) ref_AxisControl( motorList[ controlMode ], sensorList[ controlMode ] );

    Motor_WriteConfig( motorList[ controlMode ] );
    
    elapsedTime = Timing_GetExecTimeMilliseconds() - execTime;
    DEBUG_UPDATE( "control pass for Axis %s (before delay): elapsed time: %u", motorList[ controlMode ]->name, elapsedTime );
    
    if( elapsedTime < (int) ( 1000 * CONTROL_SAMPLING_INTERVAL ) ) Timing_Delay( 1000 * CONTROL_SAMPLING_INTERVAL - elapsedTime );
    
    DEBUG_UPDATE( "control pass for Axis %s (after delay): elapsed time: %u", motorList[ controlMode ]->name, Timing_GetExecTimeMilliseconds() - execTime );
  }
  
  Thread_Exit( 0 );
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

// Virtual impedance gains
double Kq = 10.0; // Stiffness
double Bq = 10.0; // Damping

// Sampling
const int HIPS_CONTROL_SAMPLING_NUMBER = 6;

/////////////////////////////////////////////////////////////////////////////////
/////                         HIPS CONTROL FUNCTION                         /////
/////////////////////////////////////////////////////////////////////////////////

static void HipsControl( Motor* hipsActuator, Encoder* hipsSensor )
{
  static double sensorTension_0, sensorPosition_0;

  // Sampling of last voltage signal values for filtering (average filter)
  static double analogSamples[ HIPS_CONTROL_SAMPLING_NUMBER ];

  static double angPositionInSetpoint;
    
  if( sensorTension_0 == 0 )
  {
    sensorTension_0 = Encoder_GetMeasure( hipsActuator->encoder, TENSION ); // mV
    sensorPosition_0 = ( TNS_2_POS_RATIO * sensorTension_0 + TNS_2_POS_OFFSET ); // mm  
  }
  
  double sensorTension = 0;
  for( int i = 0; i < HIPS_CONTROL_SAMPLING_NUMBER; i ++ )
  {
    analogSamples[ i ] = ( i == 0 ) ? Encoder_GetMeasure( hipsActuator->encoder, TENSION ) : analogSamples[ i - 1 ]; // mV
    sensorTension += analogSamples[ i ] / HIPS_CONTROL_SAMPLING_NUMBER;
  }

  double sensorPosition = ( TNS_2_POS_RATIO * sensorTension + TNS_2_POS_OFFSET );	// mm

  double actuatorForce = -HIPS_FORCE_SENSOR_STIFFNESS * ( sensorPosition_0 - sensorPosition ); // N

  angPositionInSetpoint = Motor_GetParameter( hipsActuator, POSITION_SETPOINT );
  printf( "Angle setpoint: %g\n", angPositionInSetpoint );

  // Relation between axis rotation and effector linear displacement
  double actuatorEncoderPosition = Encoder_GetMeasure( hipsActuator->encoder, POSITION );
  double effectorDeltaLength = ( actuatorEncoderPosition /*/ hipsActuator->encoder->resolution*/ ) * HIPS_ACTUATOR_STEP + ( sensorPosition_0 - sensorPosition ) / 1000; // m

  double effectorLength = HIPS_EFFECTOR_BASE_LENGTH + effectorDeltaLength; // m

  double cos_eta = ( pow( effectorLength, 2 ) + pow( HIPS_STRUCT_DIMS[ A2B ], 2 ) - pow( HIPS_STRUCT_DIMS[ C2D ], 2 ) )
				   / ( 2 * effectorLength * HIPS_STRUCT_DIMS[ A2B ] );
  double sin_eta = sqrt( (double)( 1 - pow( cos_eta, 2 ) ) );

  double gama = acos( ( pow( HIPS_STRUCT_DIMS[ C2D ], 2 ) + pow( HIPS_STRUCT_DIMS[ A2B ], 2 ) - pow( effectorLength, 2 ) )
					  / ( 2 * HIPS_STRUCT_DIMS[ C2D ] * HIPS_STRUCT_DIMS[ A2B ] ) );

  double theta = HIPS_STRUCT_ANGLES[ ALPHA ] + HIPS_STRUCT_ANGLES[ BETA ] + gama - PI;

  // Current theta derivative with respect to the effector total length (base + delta)
  double lengthToThetaRatio = sqrt( pow( HIPS_STRUCT_DIMS[ C2D ], 2 ) + pow( HIPS_STRUCT_DIMS[ A2B ], 2 )
									  - 2 * HIPS_STRUCT_DIMS[ C2D ] * HIPS_STRUCT_DIMS[ A2B ] * cos( gama ) )
								/ ( HIPS_STRUCT_DIMS[ C2D ] * HIPS_STRUCT_DIMS[ A2B ] * sin( gama ) );

  Kq = Motor_GetParameter( hipsActuator, PROPORTIONAL_GAIN );
  Bq = Motor_GetParameter( hipsActuator, DERIVATIVE_GAIN );

  if( (int) Kq <= 0 ) Kq = 1.0;
  if( (int) Bq <= 0 ) Bq = 1.0;

  double actuatorStiffness = Kq * lengthToThetaRatio * ( 1 / ( HIPS_STRUCT_DIMS[ A2B ] * sin_eta ) );
  double actuatorDamping = Bq * lengthToThetaRatio * ( 1 / ( HIPS_STRUCT_DIMS[ A2B ] * sin_eta ) );

  double Fv_q = ( Kq * angPositionInSetpoint ) / ( HIPS_STRUCT_DIMS[ A2B ] * sin_eta );

  // Control law
  double velocitySetpoint = ( Fv_q - actuatorStiffness * ( ( actuatorEncoderPosition /*/ hipsActuator->encoder->resolution*/ ) * HIPS_ACTUATOR_STEP )
						  + ( ( actuatorStiffness - HIPS_FORCE_SENSOR_STIFFNESS ) / HIPS_FORCE_SENSOR_STIFFNESS ) * actuatorForce ) / actuatorDamping;

  int velocityRPMSetpoint = ( velocitySetpoint / HIPS_ACTUATOR_STEP ) * 60 / ( 2 * PI ); // rpm

  Motor_SetParameter( hipsActuator, VELOCITY_SETPOINT, velocityRPMSetpoint );
}


/////////////////////////////////////////////////////////////////////////////////
/////                            KNEE CONSTANTS                             /////
/////////////////////////////////////////////////////////////////////////////////

const int KNEE_JOINT_REDUCTION = 150;       // Joint gear reduction ratio
const int KNEE_BASE_STIFFNESS = 104;        // Elastic constant for knee joint spring

const double a2 = -0.9428;
const double a3 = 0.3333;
const double b1 = 0.0976;
const double b2 = 0.1953;
const double b3 = 0.0976;

const double c2 = -1.889;
const double c3 = 0.8949;
const double d1 = 0.0015;
const double d2 = 0.0029;
const double d3 = 0.0015;

/////////////////////////////////////////////////////////////////////////////////
/////                     KNEE CONTROL DEFAULT VALUES                       /////
/////////////////////////////////////////////////////////////////////////////////

// PID control gains
const double kp = 370; // Proportional
const double ki = 3.5; // Integrated
const double kd = 0.0; // Derivative

// Virtual impedance gains
double Kv = 0.0; // Stiffness
double Bv = 5.0; // Damping

/////////////////////////////////////////////////////////////////////////////////
/////                          KNEE CONTROL FUNCTION                        /////
/////////////////////////////////////////////////////////////////////////////////

static void KneeControl( Motor* kneeIn, Encoder* kneeOut )
{
  static double angPositionOut[2];
  
  static double angVelocityOut[3];
  static double angVelOutWeighted[3];
  static double torque[3];
  static double torqueWeighted[3];
  static double error[3];

  static double angPositionIn, angPosInReduced, angPositionInSetpoint, angVelocityIn, angVelocityInSetpoint;

  //angPositionInSetpoint = ( Encoder_GetMeasure( kneeIn, POSITION_SETPOINT ) * 2 * PI ); // / kneeIn->encoder->resolution;

  angPositionIn = ( ( Encoder_GetMeasure( kneeIn->encoder, POSITION ) /*- angPositionInSetpoint*/ ) * 2 * PI ); // / kneeIn->encoder->resolution;
  angPosInReduced = angPositionIn / KNEE_JOINT_REDUCTION;

  angVelocityIn = Encoder_GetMeasure( kneeIn->encoder, VELOCITY );

  // Impedance control

  Kv = Motor_GetParameter( kneeIn, PROPORTIONAL_GAIN );
  Bv = Motor_GetParameter( kneeIn, DERIVATIVE_GAIN );

  //printf( "Kv: %g - Bv: %g\n", Kv,  Bv );

  angVelocityOut[0] = ( angPositionOut[0] - angPositionOut[1] ) / CONTROL_SAMPLING_INTERVAL;
  angPositionOut[1] = angPositionOut[0];

  angVelOutWeighted[0] = -c2 * angVelOutWeighted[1] - c3 * angVelOutWeighted[2] + d1 * angVelocityOut[0] + d2 * angVelocityOut[1] + d3 * angVelocityOut[2];

  angPositionOut[0] = ( ( -Encoder_GetMeasure( kneeOut, POSITION ) /*- Encoder_GetMeasure( knee_out, POSITION_SETPOINT )*/ ) * 2 * PI ); // / kneeOut->resolution;

  double torqueSetpoint = -Kv * ( angPositionOut[0] - angPositionInSetpoint ) - Bv * ( angVelOutWeighted[0] * angVelocityInSetpoint );

  //Motor_SetParameter( knee_in, ANGLE, (int) ( 1000 * ang_position_out[0] ) );

  torque[0] = KNEE_BASE_STIFFNESS * ( angPosInReduced - angPositionOut[0] );
  //Motor_SetParameter( knee_in, FORCE, (int) torque[0] );

  torqueWeighted[0] = -a2 * torqueWeighted[1] - a3 * torqueWeighted[2] + b1 * torque[0] + b2 * torque[1] + b3 * torque[2];

  error[0] = ( torqueSetpoint - torqueWeighted[0] );

  angVelocityInSetpoint += kp * ( error[0] - error[1] ) + ki * CONTROL_SAMPLING_INTERVAL * error[0] + ( kd / CONTROL_SAMPLING_INTERVAL ) * ( error[0] - 2 * error[1] + error[2] );

  for( int i = 1; i <= 2; i++ )
  {
    error[ i ] = error[ i - 1 ];
    angVelocityOut[ i ] = angVelocityOut[ i - 1 ];
    angVelOutWeighted[ i ] = angVelOutWeighted[ i - 1 ];
    torqueWeighted[ i ] = torqueWeighted[ i - 1 ];
  }

  Motor_SetParameter( kneeIn, VELOCITY_SETPOINT, (int) angVelocityInSetpoint );
}

/////////////////////////////////////////////////////////////////////////////////
/////                         ANKLE CONTROL FUNCTION                        /////
/////////////////////////////////////////////////////////////////////////////////

static void AnkleControl( Motor* ankleActuator, Encoder* ankleSensor )
{
  //static int velocitySetpoint;  

  //if( Motor_GetParameter( ankleActuator, POSITION_SETPOINT ) > 0.5 ) Motor_SetParameter( ankleActuator, POSITION_SETPOINT, 0.5 );
  //else if( Motor_GetParameter( ankleActuator, POSITION_SETPOINT ) < -0.5 ) Motor_SetParameter( ankleActuator, POSITION_SETPOINT, -0.5 );
  
  if( Encoder_GetMeasure( ankleActuator->encoder, POSITION ) > 0.5 ) Motor_SetParameter( ankleActuator, POSITION_SETPOINT, -0.6 );
  else if( Encoder_GetMeasure( ankleActuator->encoder, POSITION ) < -0.5 ) Motor_SetParameter( ankleActuator, POSITION_SETPOINT, 0.6 );
  
  //velocitySetpoint = -( Encoder_GetMeasure( ankleActuator->encoder, POSITION ) - Encoder_GetMeasure( ankleActuator->encoder, POSITION_SETPOINT ) ) * 100;
  
  //Motor_SetParameter( ankleActuator, VELOCITY_SETPOINT, 100 );
}

#endif /* CONTROL_H */ 
