#ifndef CONTROL_H
#define CONTROL_H

#include "axis.h"

#include "async_debug.h"

#include <math.h>
  
enum Joint { ANKLE, KNEE, HIPS, CONTROL_N_JOINTS };  // Control algorhitms/axes indexes

/////////////////////////////////////////////////////////////////////////////////
/////                            CONTROL DEVICES                            /////
/////////////////////////////////////////////////////////////////////////////////

//CAN database addressing
const char* CAN_DATABASE = "database";
const char* CAN_CLUSTER = "NETCAN";

static Motor* motorList[ CONTROL_N_JOINTS ];           // EPOS active axes list
static Encoder* sensorList[ CONTROL_N_JOINTS ];        // EPOS passive (measurement) axes list

// Control algorhitms
static void ankle_control( Motor*, Encoder* );
static void knee_control( Motor*, Encoder* );
static void hips_control( Motor*, Encoder* );

typedef void (*ControlFunction)( Motor*, Encoder* ); // Use "async_function" type as "void (*)( Motor*, Encoder* )" type
// Array of function pointers. Equivalent to void (*control_functions[ CONTROL_N_JOINTS ])( Motor*, Encoder* )
static ControlFunction control_functions[ CONTROL_N_JOINTS ] = { ankle_control, knee_control, hips_control }; 

static void* async_control( void* ); 

// EPOS devices and CAN network initialization
void AxisControl_Init()
{
  DEBUG_EVENT( 0, "Initializing Axis Control on thread %x", CmtGetCurrentThreadID() );
  
  // Start CAN network transmission
  for( uint8_t nodeID = 1; nodeID <= CONTROL_N_JOINTS; nodeID++ )
    EposNetwork_Start( CAN_DATABASE, CAN_CLUSTER, nodeID );
  
  for( size_t jointID = 0; jointID < CONTROL_N_JOINTS; jointID++ )
  {
    motorList[ jointID ] = NULL;
    sensorList[ jointID ] = NULL;
  }
  
  /* Specific initialization code */
  // motorList[ ANKLE ] = motor_create( "Ankle", 4, 4096 ); // ?
  motorList[ KNEE ] = Motor_Connect( "Knee", 2, 4096 );
  sensorList[ KNEE ] = Encoder_Connect( 1, 2000 );
  motorList[ HIPS ] = Motor_Connect( "Hips", 3, 4096 );
  
  // Start a control thread for each motor
  for( size_t control_mode = 0; control_mode < CONTROL_N_JOINTS; control_mode++ )
    (void) Thread_Start( async_control, (void*) control_mode, DETACHED );
  
  DEBUG_EVENT( 0, "Axis Control initialized on thread %x", CmtGetCurrentThreadID() );
}

// EPOS devices and CAN network shutdown
void AxisControl_End()
{
  DEBUG_EVENT( 0, "Ending Axis Control on thread %x", CmtGetCurrentThreadID() );
  
  // End CAN network transmission
  EposNetwork_Stop( 1 );

  Timing_Delay( 2000 );
  
  // Destroy axes data structures
  for( size_t axisID = 0; axisID < CONTROL_N_JOINTS; axisID++ )
  {
    Motor_Disconnect( motorList[ axisID ] );
    Encoder_Disconnect( sensorList[ axisID ] );
  }
  
  DEBUG_EVENT( 0, "Axis Control ended on thread %x", CmtGetCurrentThreadID() );
}

extern inline void AxisControl_EnableMotor( size_t motorID )
{
  if( motorID < 0 || motorID >= CONTROL_N_JOINTS ) return;
  
  if( motorList[ motorID ] != NULL )
    Motor_Enable( motorList[ motorID ] );
}

extern inline void AxisControl_DisableMotor( size_t motorID )
{
  if( motorID < 0 || motorID >= CONTROL_N_JOINTS ) return;
  
  if( motorList[ motorID ] != NULL )
    Motor_Disable( motorList[ motorID ] );
}

extern inline void AxisControl_ResetMotor( size_t motorID )
{
  if( motorID < 0 || motorID >= CONTROL_N_JOINTS ) return;
  
  if( motorList[ motorID ] != NULL )
    Encoder_Reset( motorList[ motorID ]->encoder );
}

extern inline void AxisControl_ConfigMotor( size_t motorID, double* parametersList )
{
  if( motorID < 0 || motorID >= CONTROL_N_JOINTS ) return;
  
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
  
  if( motorID < 0 || motorID >= CONTROL_N_JOINTS ) return NULL;
  
  if( motorList[ motorID ] == NULL ) return NULL;
  
  for( size_t stateIndex = 0; stateIndex < AXIS_N_STATES; stateIndex++ )
    statesList[ stateIndex ] = Encoder_CheckState( motorList[ motorID ]->encoder, stateIndex );
  
  return statesList;
}

extern inline double* AxisControl_GetMotorMeasures( size_t motorID )
{
  static double measuresList[ AXIS_N_DIMS ];
  
  if( motorID < 0 || motorID >= CONTROL_N_JOINTS ) return NULL;
  
  if( motorList[ motorID ] == NULL ) return NULL;
  
  for( size_t dimensionIndex = 0; dimensionIndex < AXIS_N_DIMS; dimensionIndex++ )
    measuresList[ dimensionIndex ] = Encoder_GetMeasure( motorList[ motorID ]->encoder, dimensionIndex );
  
  return measuresList;
}

extern inline const char* AxisControl_GetMotorName( size_t motorID )
{
  if( motorID < 0 || motorID >= CONTROL_N_JOINTS ) return NULL;
  
  if( motorList[ motorID ] == NULL ) return NULL;
  
  return motorList[ motorID ]->name;
}

/////////////////////////////////////////////////////////////////////////////////
/////                           COMMON CONSTANTS                            /////
/////////////////////////////////////////////////////////////////////////////////

#ifndef PI
const double PI = 3.141592;        // Duh...
#endif

const double TS = 0.005;           // Sampling interval

// Method that runs the control functions asyncronously
static void* async_control( void* args )
{
  unsigned long control_mode = (unsigned long) args;
  
  ControlFunction control_function = control_functions[ control_mode ];
  
  unsigned int exec_time, elapsed_time;
  
  while( motorList[ control_mode ] != NULL )
  {
    exec_time = Timing_GetExecTimeMilliseconds();

    Encoder_Read( motorList[ control_mode ]->encoder );
	
  	// Verify and try to correct errors
  	if( Encoder_CheckState( motorList[ control_mode ]->encoder, FAULT ) == true )
  	  Encoder_Reset( motorList[ control_mode ]->encoder );
  	if( sensorList[ control_mode ] != NULL )
  	{
  	  if( Encoder_CheckState( sensorList[ control_mode ], FAULT ) == true ) 
          Encoder_Reset( sensorList[ control_mode ] );
  	}
    
    // If the motor is being actually controlled, call control pass algorhitm
    if( motorList[ control_mode ]->active ) control_function( motorList[ control_mode ], sensorList[ control_mode ] );

    Motor_WriteConfig( motorList[ control_mode ] );
    
    elapsed_time = Timing_GetExecTimeMilliseconds() - exec_time;
    DEBUG_UPDATE( "control pass for Axis %s (before delay): elapsed time: %u", motorList[ control_mode ]->name, elapsed_time );
    
    if( elapsed_time < (int) ( 1000 * TS ) ) Timing_Delay( 1000 * TS - elapsed_time );
    
    DEBUG_UPDATE( "control pass for Axis %s (after delay): elapsed time: %u", motorList[ control_mode ]->name, Timing_GetExecTimeMilliseconds() - exec_time );
  }
  
  Thread_Exit( 0 );
  return NULL;
}

extern inline void control_mode_start( enum Joint mode )
{
  if( mode >= 0 && mode < CONTROL_N_JOINTS )
    Motor_Enable( motorList[ mode ] );
}

extern inline void control_mode_stop( enum Joint mode )
{
  if( mode >= 0 && mode < CONTROL_N_JOINTS )
    Motor_Disable( motorList[ mode ] );
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

static void hips_control( Motor* hips_actuator, Encoder* hips_sensor )
{
  static double sensor_tension_zero, sensor_position_zero;

  // Sampling of last voltage signal values for filtering (average filter)
  static double analog_samples[ HIPS_CONTROL_SAMPLING_NUMBER ];

  static double ang_position_in_ref;
    
  if( sensor_tension_zero == 0 )
  {
    Motor_SetOperationMode( hips_actuator, VELOCITY_MODE );
    
    sensor_tension_zero = Encoder_GetMeasure( hips_actuator->encoder, TENSION ); // mV
    sensor_position_zero = ( TNS_2_POS_RATIO * sensor_tension_zero + TNS_2_POS_OFFSET ); // mm  
  }
  
  double sensor_tension = 0;
  for( int i = 0; i < HIPS_CONTROL_SAMPLING_NUMBER; i ++ )
  {
    analog_samples[ i ] = ( i == 0 ) ? Encoder_GetMeasure( hips_actuator->encoder, TENSION ) : analog_samples[ i - 1 ]; // mV
    sensor_tension += analog_samples[ i ] / HIPS_CONTROL_SAMPLING_NUMBER;
  }

  double sensor_position = ( TNS_2_POS_RATIO * sensor_tension + TNS_2_POS_OFFSET );	// mm

  double actuator_force = -HIPS_FORCE_SENSOR_STIFFNESS * ( sensor_position_zero - sensor_position ); // N

  ang_position_in_ref = Motor_GetParameter( hips_actuator, POSITION_SETPOINT );
  printf( "Angle setpoint: %g\n", ang_position_in_ref );

  // Relation between axis rotation and effector linear displacement
  double actuator_encoder_position = Encoder_GetMeasure( hips_actuator->encoder, POSITION );
  double effector_delta_length = ( actuator_encoder_position / hips_actuator->encoder->resolution ) * HIPS_ACTUATOR_STEP + ( sensor_position_zero - sensor_position ) / 1000; // m

  double effector_length = HIPS_EFFECTOR_BASE_LENGTH + effector_delta_length; // m

  double cos_eta = ( pow( effector_length, 2 ) + pow( HIPS_STRUCT_DIMS[ A2B ], 2 ) - pow( HIPS_STRUCT_DIMS[ C2D ], 2 ) )
				   / ( 2 * effector_length * HIPS_STRUCT_DIMS[ A2B ] );
  double sin_eta = sqrt( (double)( 1 - pow( cos_eta, 2 ) ) );

  double gama = acos( ( pow( HIPS_STRUCT_DIMS[ C2D ], 2 ) + pow( HIPS_STRUCT_DIMS[ A2B ], 2 ) - pow( effector_length, 2 ) )
					  / ( 2 * HIPS_STRUCT_DIMS[ C2D ] * HIPS_STRUCT_DIMS[ A2B ] ) );

  double theta = HIPS_STRUCT_ANGLES[ ALPHA ] + HIPS_STRUCT_ANGLES[ BETA ] + gama - PI;

  // Current theta derivative with respect to the effector total length (base + delta)
  double length_2_theta_ratio = sqrt( pow( HIPS_STRUCT_DIMS[ C2D ], 2 ) + pow( HIPS_STRUCT_DIMS[ A2B ], 2 )
									  - 2 * HIPS_STRUCT_DIMS[ C2D ] * HIPS_STRUCT_DIMS[ A2B ] * cos( gama ) )
								/ ( HIPS_STRUCT_DIMS[ C2D ] * HIPS_STRUCT_DIMS[ A2B ] * sin( gama ) );

  Kq = Motor_GetParameter( hips_actuator, PROPORTIONAL_GAIN );
  Bq = Motor_GetParameter( hips_actuator, DERIVATIVE_GAIN );

  if( (int) Kq <= 0 ) Kq = 1.0;
  if( (int) Bq <= 0 ) Bq = 1.0;

  double actuator_stiffness = Kq * length_2_theta_ratio * ( 1 / ( HIPS_STRUCT_DIMS[ A2B ] * sin_eta ) );
  double actuator_damping = Bq * length_2_theta_ratio * ( 1 / ( HIPS_STRUCT_DIMS[ A2B ] * sin_eta ) );

  double Fv_q = ( Kq * ang_position_in_ref ) / ( HIPS_STRUCT_DIMS[ A2B ] * sin_eta );

  // Control law
  double velocity_ref = ( Fv_q - actuator_stiffness * ( ( actuator_encoder_position / hips_actuator->encoder->resolution ) * HIPS_ACTUATOR_STEP )
						  + ( ( actuator_stiffness - HIPS_FORCE_SENSOR_STIFFNESS ) / HIPS_FORCE_SENSOR_STIFFNESS ) * actuator_force ) / actuator_damping;

  int velocity_ref_rpm = ( velocity_ref / HIPS_ACTUATOR_STEP ) * 60 / ( 2 * PI ); // rpm

  //motor_set_reference( hips, ANGLE, 180 * theta / PI );
  Motor_SetParameter( hips_actuator, VELOCITY_SETPOINT, velocity_ref_rpm );
  //motor_set_reference( hips, FORCE, actuator_force );
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

static void knee_control( Motor* knee_in, Encoder* knee_out )
{
  static double ang_position_out[2];
  
  static double ang_velocity_out[3];
  static double ang_vel_out_weighted[3];
  static double torque[3];
  static double torque_weighted[3];
  static double error[3];

  static double ang_position_in, ang_pos_in_reduced, ang_position_in_ref, ang_velocity_in, ang_velocity_in_ref;
  
  if( ang_position_in == 0 ) Motor_SetOperationMode( knee_in, VELOCITY_MODE );

  //ang_position_in_ref = ( Encoder_GetMeasure( knee_in, POSITION_SETPOINT ) * 2 * PI ) / ENCODER_IN_RES;

  ang_position_in = ( ( Encoder_GetMeasure( knee_in->encoder, POSITION ) /*- ang_position_in_ref*/ ) * 2 * PI ) / knee_in->encoder->resolution;
  ang_pos_in_reduced = ang_position_in / KNEE_JOINT_REDUCTION;

  ang_velocity_in = Encoder_GetMeasure( knee_in->encoder, VELOCITY );

  // Impedance control

  Kv = Motor_GetParameter( knee_in, PROPORTIONAL_GAIN );
  Bv = Motor_GetParameter( knee_in, DERIVATIVE_GAIN );

  //printf( "Kv: %g - Bv: %g\n", Kv,  Bv );

  ang_velocity_out[0] = ( ang_position_out[0] - ang_position_out[1] ) / TS;
  ang_position_out[1] = ang_position_out[0];

  ang_vel_out_weighted[0] = -c2 * ang_vel_out_weighted[1] - c3 * ang_vel_out_weighted[2] + d1 * ang_velocity_out[0] + d2 * ang_velocity_out[1] + d3 * ang_velocity_out[2];

  ang_position_out[0] = ( ( -Encoder_GetMeasure( knee_out, POSITION ) /*- Encoder_GetMeasure( knee_out, POSITION_SETPOINT )*/ ) * 2 * PI ) / knee_out->resolution;

  double torque_ref = -Kv * ( ang_position_out[0] - ang_position_in_ref ) - Bv * ( ang_vel_out_weighted[0] * ang_velocity_in_ref );

  //Motor_SetParameter( knee_in, ANGLE, (int) ( 1000 * ang_position_out[0] ) );

  torque[0] = KNEE_BASE_STIFFNESS * ( ang_pos_in_reduced - ang_position_out[0] );
  //Motor_SetParameter( knee_in, FORCE, (int) torque[0] );

  torque_weighted[0] = -a2 * torque_weighted[1] - a3 * torque_weighted[2] + b1 * torque[0] + b2 * torque[1] + b3 * torque[2];

  error[0] = ( torque_ref - torque_weighted[0] );

  ang_velocity_in_ref += kp * ( error[0] - error[1] ) + ki * TS * error[0] + ( kd / TS ) * ( error[0] - 2 * error[1] + error[2] );

  for( int i = 1; i <= 2; i++ )
  {
    error[ i ] = error[ i - 1 ];
    ang_velocity_out[ i ] = ang_velocity_out[ i - 1 ];
    ang_vel_out_weighted[ i ] = ang_vel_out_weighted[ i - 1 ];
    torque_weighted[ i ] = torque_weighted[ i - 1 ];
  }

  //Motor_SetParameter( knee_in, ANGLE, 180 * ang_position_out[0] / PI );
  //Motor_SetParameter( knee_in, ANGLE_SETPOINT, 180 * ang_position_in_ref / PI );
  Motor_SetParameter( knee_in, VELOCITY_SETPOINT, (int) ang_velocity_in_ref );
}

/////////////////////////////////////////////////////////////////////////////////
/////                         ANKLE CONTROL FUNCTION                        /////
/////////////////////////////////////////////////////////////////////////////////

static void ankle_control( Motor* ankle_actuator, Encoder* ankle_sensor )
{
  static int velocity_ref;  
    
  if( velocity_ref == 0 ) Motor_SetOperationMode( ankle_actuator, VELOCITY_MODE );

  if( Encoder_GetMeasure( ankle_actuator->encoder, POSITION_SETPOINT ) > 2000 ) Motor_SetParameter( ankle_actuator, POSITION_SETPOINT, 2000 );
  else if( Encoder_GetMeasure( ankle_actuator->encoder, POSITION_SETPOINT ) < -2000 ) Motor_SetParameter( ankle_actuator, POSITION_SETPOINT, -2000 );
  
  velocity_ref = -( Encoder_GetMeasure( ankle_actuator->encoder, POSITION ) - Encoder_GetMeasure( ankle_actuator->encoder, POSITION_SETPOINT ) ) / 20;
  
  Motor_SetParameter( ankle_actuator, VELOCITY_SETPOINT, velocity_ref );
}

#endif /* CONTROL_H */ 
