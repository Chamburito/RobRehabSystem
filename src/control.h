#ifndef CONTROL_H
#define CONTROL_H

#include "axis.h"

#ifdef _CVI_DLL_
  #include "threads_realtime.h"
#elif WIN32
  #include "threads_windows.h"
#else
  #include "threads_unix.h"
#endif

#include <cmath>
#include <iostream>

using namespace std;
  
enum Joint { ANKLE, KNEE, HIPS, N_JOINTS, KNEE_AUX = 3 };  // Control algorhitms/axes indexes

/////////////////////////////////////////////////////////////////////////////////
/////                            CONTROL DEVICES                            /////
/////////////////////////////////////////////////////////////////////////////////

//CAN database addressing
const char* CAN_DATABASE = "database";
const char* CAN_CLUSTER = "NETCAN";

// EPOS active axes list
static Motor* motor_list[ N_JOINTS ];
// EPOS passive (measurement) axes list
static Axis_Sensor* sensor_list[ N_JOINTS ];

// Control algorhitms
static void ankle_control( void );
static void knee_control( void );
static void hips_control( void );

typedef void (*Control_Function)( void ); // Use "async_function" type as "void* (*)( void )" type
static Control_Function control_functions[ N_JOINTS ]; // Array of function pointers. Equivalent to void* (*control_functions[ N_JOINTS ])( void )

// EPOS devices and CAN network initialization
void control_init()
{
  // Start CAN network transmission
  for( int node_id = 1; node_id <= N_AXES; node_id++ )
    epos_network_start( CAN_DATABASE, CAN_CLUSTER, node_id );
  
  // motor_list[ ANKLE ] = motor_create( 4 ); // ?
  motor_list[ KNEE ] = motor_create( 2, 4096 );
  sensor_list[ KNEE ] = encoder_create( 1, 2000 );
  motor_list[ HIPS ] = motor_create( 3, 4096 );  
  
  control_functions[ ANKLE ] = ankle_control;
  control_functions[ KNEE ] = knee_control;
  control_functions[ HIPS ] = hips_control;
}

// EPOS devices and CAN network shutdown
void control_end()
{
  // End CAN network transmission
  epos_network_stop( 1 );

  delay( 2000 );
  
  for( size_t motor_id = 0; motor_id < N_AXES; motor_id++ )
    motor_destroy( motor_list[ motor_id ] );
    
  // End threading subsystem
  end_threading();
}

/////////////////////////////////////////////////////////////////////////////////
/////                           COMMON CONSTANTS                            /////
/////////////////////////////////////////////////////////////////////////////////

const double PI = 3.141592;
const double Ts = 0.005; // sampling interval

static void* async_control( void* args )
{
  unsigned long motor_id = (unsigned long) args;
  
  motor_enable( motor_list[ motor_id ] );
  
  unsigned int exec_time, elapsed_time;
  
  while( motor_list[ motor_id ] != NULL )
  {
    exec_time = get_exec_time_milisseconds();

    motor_read_values( motor_list[ motor_id ] );
    
    if( motor_list[ motor_id ]->active ) (control_functions[ motor_id ])();
	
	motor_write_values( motor_list[ motor_id ] );
    
    elapsed_time = get_exec_time_milisseconds() - exec_time;
    printf( "async_control (Axis %u): elapsed time: %u\n", motor_id, elapsed_time );
    if( elapsed_time < (int) ( 1000 * Ts ) ) delay( 1000 * Ts - elapsed_time );
    printf( "async_control (Axis %u): elapsed time: %u\n\n", motor_id, get_exec_time_milisseconds() - exec_time );
  }
  
  exit_thread( 0 );
  return NULL;
}

extern inline void control_mode_start( Joint mode )
{
  if( mode >= 0 && mode < N_JOINTS )
    motor_enable( motor_list[ mode ] );

    //run_thread( async_control, (void*) mode, DETACHED );
}

extern inline void control_mode_stop( Joint mode )
{
  if( mode >= 0 && mode < N_JOINTS )
    motor_disable( motor_list[ mode ] );
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
const double HIPS_STRUCT_DIMS[] = { 0.063, 0.060, 0.284, 0.117, sqrt( 0.284 * 0.284 + 0.117 * 0.117 ), sqrt( 0.063 * 0.063 + 0.060 * 0.060 ) };
// Actuator length with effector at initial position (in meters)
const double HIPS_EFFECTOR_BASE_LENGTH = 0.348;

enum { ALPHA, BETA };
const double HIPS_STRUCT_ANGLES[] = { atan( HIPS_STRUCT_DIMS[ A ] / HIPS_STRUCT_DIMS[ B ] ), atan( HIPS_STRUCT_DIMS[ C ] / HIPS_STRUCT_DIMS[ D ] ) };

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

static void hips_control( void )
{
  Motor* hips = motor_list[ HIPS ];

  static double sensor_tension_zero, sensor_position_zero;

  // Sampling of last voltage signal values for filtering (average filter)
  static double analog_samples[ HIPS_CONTROL_SAMPLING_NUMBER ];

  static double ang_position_in_ref; // ?
    
  if( sensor_tension_zero == 0 )
  {
	motor_set_operation_mode( hips, VELOCITY_MODE );
	  
	double sensor_tension_zero = encoder_get_measure( hips->encoder, TENSION ); // mV
  	double sensor_position_zero = ( TNS_2_POS_RATIO * sensor_tension_zero + TNS_2_POS_OFFSET ); // mm  
  }
  
  double sensor_tension = 0;
  for( int i = 0; i < HIPS_CONTROL_SAMPLING_NUMBER; i ++ )
  {
	  analog_samples[ i ] = ( i == 0 ) ? encoder_get_measure( hips->encoder, TENSION ) : analog_samples[ i - 1 ]; // mV
	  sensor_tension += analog_samples[ i ] / HIPS_CONTROL_SAMPLING_NUMBER;
  }

  double sensor_position = ( TNS_2_POS_RATIO * sensor_tension + TNS_2_POS_OFFSET );	// mm

  double actuator_force = -HIPS_FORCE_SENSOR_STIFFNESS * ( sensor_position_zero - sensor_position ); // N

  ang_position_in_ref = motor_get_parameter( hips, POSITION_SETPOINT );
  printf( "Angle setpoint: %g\n", ang_position_in_ref );

  // Relation between axis rotation and effector linear displacement
  double actuator_encoder_position = encoder_get_measure( hips, POSITION );
  double effector_delta_length = ( actuator_encoder_position / hips->encoder->resolution ) * HIPS_ACTUATOR_STEP + ( sensor_position_zero - sensor_position ) / 1000; // m

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

  Kq = motor_get_parameter( hips, PROPORTIONAL_GAIN );
  Bq = motor_get_parameter( hips, DERIVATIVE_GAIN );

  if( (int) Kq <= 0 ) Kq = 1.0;
  if( (int) Bq <= 0 ) Bq = 1.0;

  double actuator_stiffness = Kq * length_2_theta_ratio * ( 1 / ( HIPS_STRUCT_DIMS[ A2B ] * sin_eta ) );
  double actuator_damping = Bq * length_2_theta_ratio * ( 1 / ( HIPS_STRUCT_DIMS[ A2B ] * sin_eta ) );

  double Fv_q = ( Kq * ang_position_in_ref ) / ( HIPS_STRUCT_DIMS[ A2B ] * sin_eta );

  // Control law
  double velocity_ref = ( Fv_q - actuator_stiffness * ( ( actuator_encoder_position / hips->encoder->resolution ) * HIPS_ACTUATOR_STEP )
						  + ( ( actuator_stiffness - HIPS_FORCE_SENSOR_STIFFNESS ) / HIPS_FORCE_SENSOR_STIFFNESS ) * actuator_force ) / actuator_damping;

  int velocity_ref_rpm = ( velocity_ref / HIPS_ACTUATOR_STEP ) * 60 / ( 2 * PI ); // rpm

  //motor_set_parameter( hips, ANGLE, 180 * theta / PI );
  motor_set_parameter( hips, VELOCITY_SETPOINT, velocity_ref_rpm );
  //motor_set_parameter( hips, FORCE, actuator_force );
}


/////////////////////////////////////////////////////////////////////////////////
/////                            KNEE CONSTANTS                             /////
/////////////////////////////////////////////////////////////////////////////////

const int KNEE_JOINT_REDUCTION = 150;       // Reducao da junta
const int KNEE_BASE_STIFFNESS = 104;        // Constante elástica

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
/////                     KNEE CONTROL DEFAULT VALUES                        /////
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

static void knee_control()
{
  Encoder* knee_out = sensor_list[ KNEE ];
  Motor* knee_in = motor_list[ KNEE ];

  motor_set_operation_mode( knee_in, VELOCITY_MODE );

  static double ang_position_out[2];
  
  static double ang_velocity_out[3];
  static double ang_vel_out_weighted[3];
  static double torque[3];
  static double torque_weighted[3];
  static double error[3];

  static double ang_position_in_ref, ang_velocity_in_ref; // ?

  //ang_position_in_ref = ( encoder_get_measure( knee_in, POSITION_SETPOINT ) * 2 * PI ) / ENCODER_IN_RES;

  double ang_position_in = ( ( encoder_get_measure( knee_in->encoder, POSITION ) /*- ang_position_in_ref*/ ) * 2 * PI ) / knee_in->encoder->resolution;
  double ang_pos_in_reduced = ang_position_in / KNEE_JOINT_REDUCTION;

  double ang_velocity_in = encoder_get_measure( knee_in->encoder, VELOCITY );

  // Impedance control

  Kv = motor_get_parameter( knee_in, PROPORTIONAL_GAIN );
  Bv = motor_get_parameter( knee_in, DERIVATIVE_GAIN );

  //printf( "Kv: %g - Bv: %g\n", Kv,  Bv );

  ang_velocity_out[0] = ( ang_position_out[0] - ang_position_out[1] ) / Ts;
  ang_position_out[1] = ang_position_out[0];

  ang_vel_out_weighted[0] = -c2 * ang_vel_out_weighted[1] - c3 * ang_vel_out_weighted[2] + d1 * ang_velocity_out[0] + d2 * ang_velocity_out[1] + d3 * ang_velocity_out[2];

  ang_position_out[0] = ( ( -encoder_get_measure( knee_out, POSITION ) /*- encoder_get_measure( knee_out, POSITION_SETPOINT )*/ ) * 2 * PI ) / knee_out->resolution;

  double torque_ref = -Kv * ( ang_position_out[0] - ang_position_in_ref ) - Bv * ( ang_vel_out_weighted[0] * ang_velocity_in_ref );

  motor_set_parameter( knee_in, ANGLE, (int) ( 1000 * ang_position_out[0] ) );

  torque[0] = KNEE_BASE_STIFFNESS * ( ang_pos_in_reduced - ang_position_out[0] );
  motor_set_parameter( knee_in, FORCE, (int) torque[0] );

  torque_weighted[0] = -a2 * torque_weighted[1] - a3 * torque_weighted[2] + b1 * torque[0] + b2 * torque[1] + b3 * torque[2];

  error[0] = ( torque_ref - torque_weighted[0] );

  ang_velocity_in_ref += kp * ( error[0] - error[1] ) + ki * Ts * error[0] + ( kd / Ts ) * ( error[0] - 2 * error[1] + error[2] );

  for( int i = 1; i <= 2; i++ )
  {
	  error[ i ] = error[ i - 1 ];
	  ang_velocity_out[ i ] = ang_velocity_out[ i - 1 ];
	  ang_vel_out_weighted[ i ] = ang_vel_out_weighted[ i - 1 ];
	  torque_weighted[ i ] = torque_weighted[ i - 1 ];
  }

  //motor_set_parameter( knee_in, ANGLE, 180 * ang_position_out[0] / PI );
  //motor_set_parameter( knee_in, ANGLE_SETPOINT, 180 * ang_position_in_ref / PI );
  motor_set_parameter( knee_in, VELOCITY_SETPOINT, (int) ang_velocity_in_ref );
  motor_write_values( knee_in );

  exit_thread( 0 );
  return NULL;
}

/////////////////////////////////////////////////////////////////////////////////
/////                         ANKLE CONTROL FUNCTION                        /////
/////////////////////////////////////////////////////////////////////////////////

static void* ankle_control( void* args )
{
  Motor* ankle = motor_list[ 0 ]; // Hack. Should be ANKLE
  
  motor_enable( ankle );
  
  int exec_time, elapsed_time;
  
  motor_set_operation_mode( ankle, VELOCITY_MODE );

  int velocity_ref;

  while( ankle->active ) 
  {
     exec_time = get_exec_time_milisseconds();

    if( encoder_get_measure( ankle, POSITION_SETPOINT ) > 2000 ) motor_set_parameter( ankle, POSITION_SETPOINT, 2000 );
    else if( encoder_get_measure( ankle, POSITION_SETPOINT ) < -2000 ) motor_set_parameter( ankle, POSITION_SETPOINT, -2000 );

    velocity_ref = -( encoder_get_measure( ankle, POSITION ) - encoder_get_measure( ankle, POSITION_SETPOINT ) ) / 20;

    motor_set_parameter( ankle, VELOCITY_SETPOINT, velocity_ref );
     motor_write_values( ankle );

    elapsed_time = get_exec_time_milisseconds() - exec_time;
     //cout << "ankleControl: elapsed time: " << elapsed_time << endl;
     if( elapsed_time < (int) ( 1000 * Ts ) ) delay( 1000 * Ts - elapsed_time );
     //cout << "ankleControl: elapsed time + sleep: " << get_exec_time_milisseconds() - exec_time << endl;
     //cout << endl;
  }

  motor_set_parameter( ankle, VELOCITY_SETPOINT, 0 );
  motor_write_values( ankle );

  exit_thread( 0 );
  return NULL;
}

#endif /* CONTROL_H */ 
