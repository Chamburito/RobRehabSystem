#ifndef CONTROL_H
#define CONTROL_H

#include "axis.h"

#ifdef WIN32
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

// EPOS axes list
const int N_AXES = 4;
static Axis* axis_list[ N_AXES ];

// Control algorhitms
static void* ankle_control( void* );
static void* knee_control( void* );
static void* hips_control( void* );

typedef void* (*Async_Function)( void* ); // Use "async_function" type as "void* (*)( void* )" type
static Async_Function control_functions[ N_JOINTS ]; // Array of function pointers. Equivalent to void* (*control_functions[ N_JOINTS ])( void* )

// EPOS devices and CAN network initialization
void control_init()
{
  // Start CAN network transmission
  for( int node_id = 1; node_id <= N_AXES; node_id++ )
    epos_network_start( CAN_DATABASE, CAN_CLUSTER, node_id );
  
  // axis_list[ ANKLE ] = axis_create( 4 ); // ?
  axis_list[ KNEE ] = axis_create( 2 );
  axis_list[ KNEE_AUX ] = axis_create( 1 );
  axis_list[ HIPS ] = axis_create( 3 );  
  
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
  
  for( size_t axis_id = 0; axis_id < N_AXES; axis_id++ )
    axis_destroy( axis_list[ axis_id ] );
    
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
  unsigned long axis_id = (unsigned long) args;
  
  axis_enable( axis_list[ axis_id ] );
  axis_set_operation_mode( axis_list[ axis_id ], VELOCITY_MODE );
  
  unsigned int exec_time, elapsed_time;
  
  while( axis_list[ axis_id ] != NULL )
  {
    exec_time = get_exec_time_milisseconds();
    
    if( axis_list[ axis_id ]->active ) (control_functions[ axis_id ])( NULL );
    
    elapsed_time = get_exec_time_milisseconds() - exec_time;
    printf( "async_control (Axis %u): elapsed time: %u\n", axis_id, elapsed_time );
    if( elapsed_time < (int) ( 1000 * Ts ) ) delay( 1000 * Ts - elapsed_time );
    printf( "async_control (Axis %u): elapsed time: %u\n\n", axis_id, get_exec_time_milisseconds() - exec_time );
  }

  exit_thread( 0 );
  return NULL;
}

extern inline void run_control( Joint mode )
{
  if( mode >= 0 && mode < N_JOINTS )
    run_thread( async_control, (void*) mode, DETACHED );
}

extern inline void stop_control( Joint mode )
{
  if( mode >= 0 && mode < N_JOINTS )
    axis_disable( axis_list[ mode ] );
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

static void* hips_control( void* args )
{
  Axis* hips = axis_list[ HIPS ];
  
  axis_enable( hips );

  double tension_ref = axis_get_measure( hips, TENSION ); // mV
  double position_ref = ( TNS_2_POS_RATIO * tension_ref + TNS_2_POS_OFFSET ); // mm

  // Sampling of last voltage signal values for filtering (average filter)
  static double analog_samples[ HIPS_CONTROL_SAMPLING_NUMBER ];
  
  int exec_time, elapsed_time;

  axis_set_operation_mode( hips, VELOCITY_MODE );

  static double ang_position_in_ref; // ?

  while( hips->active/*axis_get_status( hips, SWITCHED_ON )*/ )
  {
    exec_time = get_exec_time_milisseconds();
    
    double tension = 0;
    for( int i = 0; i < HIPS_CONTROL_SAMPLING_NUMBER; i ++ )
    {
      analog_samples[ i ] = ( i == 0 ) ? axis_get_measure( hips, TENSION ) : analog_samples[ i - 1 ]; // mV
      tension += analog_samples[ i ] / HIPS_CONTROL_SAMPLING_NUMBER;
    } 

    double position = ( TNS_2_POS_RATIO * tension + TNS_2_POS_OFFSET );	// mm

    double actuator_force = -HIPS_FORCE_SENSOR_STIFFNESS * ( position_ref - position ); // N

    ang_position_in_ref = axis_get_measure( hips, POSITION_SETPOINT );
    printf( "Angle setpoint: %g\n", ang_position_in_ref );

    // Relation  between axis rotation and effector linear displacement
    int actuator_encoder_position = axis_get_measure( hips, POSITION );
    double effector_delta_length = ( actuator_encoder_position / 4096 ) * HIPS_ACTUATOR_STEP + ( position_ref - position ) / 1000; // m

    double effector_length = HIPS_EFFECTOR_BASE_LENGTH + effector_delta_length; // m

    double cos_eta = ( pow( effector_length, 2 ) + pow( HIPS_STRUCT_DIMS[ A2B ], 2 ) - pow( HIPS_STRUCT_DIMS[ C2D ], 2 ) ) 
                         / ( 2 * effector_length * HIPS_STRUCT_DIMS[ A2B ] );
    double sin_eta = sqrt( (double)( 1 - pow( cos_eta, 2 ) ) );
    
    double gama = acos( ( pow( HIPS_STRUCT_DIMS[ C2D ], 2 ) + pow( HIPS_STRUCT_DIMS[ A2B ], 2 ) - pow( effector_length, 2 ) ) 
                            / ( 2 * HIPS_STRUCT_DIMS[ C2D ] * HIPS_STRUCT_DIMS[ A2B ] ) );

    double theta = HIPS_STRUCT_ANGLES[ ALPHA ] + HIPS_STRUCT_ANGLES[ BETA ] + gama - PI;

    // Current theta derivative with respect to the effector total length (base + delta)
    double theta_2_length_ratio = sqrt( pow( HIPS_STRUCT_DIMS[ C2D ], 2 ) + pow( HIPS_STRUCT_DIMS[ A2B ], 2 ) 
                                        - 2 * HIPS_STRUCT_DIMS[ C2D ] * HIPS_STRUCT_DIMS[ A2B ] * cos( gama ) ) 
                                      / ( HIPS_STRUCT_DIMS[ C2D ] * HIPS_STRUCT_DIMS[ A2B ] * sin( gama ) );

    Kq = axis_get_parameter( hips, KP );
    Bq = axis_get_parameter( hips, KD );

    if( (int) Kq == 0 ) Kq = 1.0;
    if( (int) Bq == 0 ) Bq = 1.0;
  
    double actuator_stiffness = Kq * theta_2_length_ratio * ( 1 / ( HIPS_STRUCT_DIMS[ A2B ] * sin_eta ) );
    double actuator_damping = Bq * theta_2_length_ratio * ( 1 / ( HIPS_STRUCT_DIMS[ A2B ] * sin_eta ) );

    double Fv_q = ( Kq * ang_position_in_ref ) / ( HIPS_STRUCT_DIMS[ A2B ] * sin_eta );

    // Control law
    double velocity_ref = ( Fv_q - actuator_stiffness * ( ( actuator_encoder_position / 4096 ) * HIPS_ACTUATOR_STEP ) 
                            + ( ( actuator_stiffness - HIPS_FORCE_SENSOR_STIFFNESS ) / HIPS_FORCE_SENSOR_STIFFNESS ) * actuator_force ) / actuator_damping;

    int velocity_ref_rpm = ( velocity_ref / HIPS_ACTUATOR_STEP ) * 60 / ( 2 * PI ); // rpm
    
    //cout << "ang_position: " << theta << "  vel ref: " << velocity_ref_rpm << "  force: " << actuator_force << endl;  

    axis_set_reference( hips, ANGLE, 180 * theta / PI );
    axis_set_reference( hips, ANGLE_SETPOINT, 180 * ang_position_in_ref / PI );
    axis_set_reference( hips, VELOCITY_SETPOINT, velocity_ref_rpm );
    axis_set_reference( hips, FORCE, actuator_force );
    axis_write_values( hips );
    
    elapsed_time = get_exec_time_milisseconds() - exec_time;
    printf( "hips_control: elapsed time: %d\n", elapsed_time );
    if( elapsed_time < (int) ( 1000 * Ts ) ) delay( 1000 * Ts - elapsed_time );
    printf( "hips_control: elapsed time: %d\n\n", get_exec_time_milisseconds() - exec_time );
  }

  axis_set_reference( hips, VELOCITY_SETPOINT, 0 );
  axis_write_values( hips );

  exit_thread( 0 );
  return NULL;
}


/////////////////////////////////////////////////////////////////////////////////
/////                             KNEE CONSTANTS                             /////
/////////////////////////////////////////////////////////////////////////////////

const int KNEE_JOINT_REDUCTION = 150;  // Redução da junta
const int ENCODER_IN_RES = 4096;               // Resolução do encoder do motor
const int ENCODER_OUT_RES = 2000;           // Resolução do encoder de saída
const int KNEE_BASE_STIFFNESS = 104;       // Constante elástica

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

static void* knee_control( void* args )
{
  Axis* knee_out = axis_list[ KNEE_AUX ];
  Axis* knee_in = axis_list[ KNEE ];
  
  axis_enable( knee_in );
  
  int exec_time, elapsed_time;

  axis_set_operation_mode( knee_in, VELOCITY_MODE );

  static double ang_position_out[2];
  
  static double ang_velocity_out[3];
  static double ang_vel_out_weighted[3];
  static double torque[3];
  static double torque_weighted[3];
  static double error[3];

  static double ang_position_in_ref, ang_velocity_in_ref; // ?

  while( knee_in->active )
  {
    exec_time = get_exec_time_milisseconds();

    //ang_position_in_ref = ( axis_get_measure( knee_in, POSITION_SETPOINT ) * 2 * PI ) / ENCODER_IN_RES;
    
    double ang_position_in = ( ( axis_get_measure( knee_in, POSITION ) /*- ang_position_in_ref*/ ) * 2 * PI ) / ENCODER_IN_RES;
    double ang_pos_in_reduced = ang_position_in / KNEE_JOINT_REDUCTION;    
        
    double ang_velocity_in = axis_get_measure( knee_in, VELOCITY );

    // Impedance control
      
    Kv = axis_get_parameter( knee_in, KP );
    Bv = axis_get_parameter( knee_in, KD );

    //printf( "Kv: %g - Bv: %g\n", Kv,  Bv );
	  
    ang_velocity_out[0] = ( ang_position_out[0] - ang_position_out[1] ) / Ts;
    ang_position_out[1] = ang_position_out[0];
    
    ang_vel_out_weighted[0] = -c2 * ang_vel_out_weighted[1] - c3 * ang_vel_out_weighted[2] + d1 * ang_velocity_out[0] + d2 * ang_velocity_out[1] + d3 * ang_velocity_out[2];

    ang_position_out[0] = ( ( -axis_get_measure( knee_out, POSITION ) /*- axis_get_measure( knee_out, POSITION_SETPOINT )*/ ) * 2 * PI ) / ENCODER_OUT_RES;

    double torque_ref = -Kv * ( ang_position_out[0] - ang_position_in_ref ) - Bv * ( ang_vel_out_weighted[0] * ang_velocity_in_ref );

    axis_set_reference( knee_in, ANGLE, (int) ( 1000 * ang_position_out[0] ) );

    torque[0] = KNEE_BASE_STIFFNESS * ( ang_pos_in_reduced - ang_position_out[0] );
    axis_set_reference( knee_in, FORCE, (int) torque[0] );

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

    axis_set_reference( knee_in, ANGLE, 180 * ang_position_out[0] / PI );
    axis_set_reference( knee_in, ANGLE_SETPOINT, 180 * ang_position_in_ref / PI );
    axis_set_reference( knee_in, VELOCITY_SETPOINT, (int) ang_velocity_in_ref );
    axis_write_values( knee_in );

    elapsed_time = get_exec_time_milisseconds() - exec_time;
    printf( "knee_control: control value: %g\n", ang_velocity_in_ref );
    printf( "knee_control: elapsed time: %d\n", elapsed_time );
    if( elapsed_time < (int) ( 1000 * Ts ) ) delay( 1000 * Ts - elapsed_time );
    printf( "knee_control: elapsed time: %d\n\n", get_exec_time_milisseconds() - exec_time );
  }
  
  axis_set_reference( knee_in, VELOCITY_SETPOINT, 0 );
  axis_write_values( knee_in );

  exit_thread( 0 );
  return NULL;
}

/////////////////////////////////////////////////////////////////////////////////
/////                         ANKLE CONTROL FUNCTION                        /////
/////////////////////////////////////////////////////////////////////////////////

static void* ankle_control( void* args )
{
  Axis* ankle = axis_list[ 0 ]; // Hack. Should be ANKLE
  
  axis_enable( ankle );
  
  int exec_time, elapsed_time;
  
  axis_set_operation_mode( ankle, VELOCITY_MODE );

  int velocity_ref;

  while( ankle->active ) 
  {
     exec_time = get_exec_time_milisseconds();

    if( axis_get_measure( ankle, POSITION_SETPOINT ) > 2000 ) axis_set_reference( ankle, POSITION_SETPOINT, 2000 );
    else if( axis_get_measure( ankle, POSITION_SETPOINT ) < -2000 ) axis_set_reference( ankle, POSITION_SETPOINT, -2000 );

    velocity_ref = -( axis_get_measure( ankle, POSITION ) - axis_get_measure( ankle, POSITION_SETPOINT ) ) / 20;

    axis_set_reference( ankle, VELOCITY_SETPOINT, velocity_ref );
     axis_write_values( ankle );

    elapsed_time = get_exec_time_milisseconds() - exec_time;
     //cout << "ankleControl: elapsed time: " << elapsed_time << endl;
     if( elapsed_time < (int) ( 1000 * Ts ) ) delay( 1000 * Ts - elapsed_time );
     //cout << "ankleControl: elapsed time + sleep: " << get_exec_time_milisseconds() - exec_time << endl;
     //cout << endl;
  }

  axis_set_reference( ankle, VELOCITY_SETPOINT, 0 );
  axis_write_values( ankle );

  exit_thread( 0 );
  return NULL;
}

#endif /* CONTROL_H */ 
