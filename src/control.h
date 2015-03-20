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

static Motor* motor_list[ CONTROL_N_JOINTS ];           // EPOS active axes list
static Encoder* sensor_list[ CONTROL_N_JOINTS ];        // EPOS passive (measurement) axes list

// Control algorhitms
static void ankle_control( Motor*, Encoder* );
static void knee_control( Motor*, Encoder* );
static void hips_control( Motor*, Encoder* );

typedef void (*Control_Function)( Motor*, Encoder* ); // Use "async_function" type as "void (*)( Motor*, Encoder* )" type
// Array of function pointers. Equivalent to void (*control_functions[ CONTROL_N_JOINTS ])( Motor*, Encoder* )
static Control_Function control_functions[ CONTROL_N_JOINTS ] = { ankle_control, knee_control, hips_control }; 

// EPOS devices and CAN network initialization
void control_init()
{
  // Start CAN network transmission
  for( size_t node_id = 1; node_id <= CONTROL_N_JOINTS; node_id++ )
    epos_network_start( CAN_DATABASE, CAN_CLUSTER, node_id );
  
  for( size_t joint_id = 0; joint_id < CONTROL_N_JOINTS; joint_id++ )
    motor_list[ joint_id ] = sensor_list[ joint_id ] = NULL;
  
  /* Specific initialization code */
  // motor_list[ ANKLE ] = motor_create( "Ankle", 4, 4096 ); // ?
  motor_list[ KNEE ] = motor_connect( "Knee", 2, 4096 );
  sensor_list[ KNEE ] = encoder_connect( 1, 2000 );
  motor_list[ HIPS ] = motor_connect( "Hips", 3, 4096 );
  
  // Start a control thread for each motor
  for( size_t control_mode = 0; control_mode < CONTROL_N_JOINTS; control_mode++ )
    (void) thread_start( async_control, (void*) control_mode, DETACHED );
}

// EPOS devices and CAN network shutdown
void control_end()
{
  // End CAN network transmission
  epos_network_stop( 1 );

  delay( 2000 );
  
  // Destroy axes data structures
  for( size_t axis_id = 0; axis_id < CONTROL_N_JOINTS; axis_id++ )
  {
    motor_disconnect( motor_list[ axis_id ] );
    encoder_disconnect( sensor_list[ axis_id ] );
  }
}

extern inline void control_enable_motor( size_t motor_id )
{
  if( motor_id < 0 || motor_id >= CONTROL_N_JOINTS ) return;
  
  if( motor_list[ motor_id ] != NULL )
    motor_enable( motor_list[ motor_id ] );
}

extern inline void control_disable_motor( size_t motor_id )
{
  if( motor_id < 0 || motor_id >= CONTROL_N_JOINTS ) return;
  
  if( motor_list[ motor_id ] != NULL )
    motor_disable( motor_list[ motor_id ] );
}

extern inline void control_reset_motor( size_t motor_id )
{
  if( motor_id < 0 || motor_id >= CONTROL_N_JOINTS ) return;
  
  if( motor_list[ motor_id ] != NULL )
    encoder_reset( motor_list[ motor_id ]->encoder );
}

extern inline void control_config_motor( size_t motor_id, double parameters_list )
{
  if( motor_id < 0 || motor_id >= CONTROL_N_JOINTS ) return;
  
  if( motor_list[ motor_id ] == NULL ) return;
  
  for( size_t parameter_index = 0; parameter_index < AXIS_N_PARAMS; parameter_index++ )
  {
    if( parameters_list[ parameter_index ] != 0.0 )
      motor_set_parameter( motor_list[ motor_id ], parameter_index, parameters_list[ parameter_index ] );
  }
}

bool* control_get_motor_status( size_t motor_id )
{
  static bool states_list[ AXIS_N_STATES ];
  
  if( motor_id < 0 || motor_id >= CONTROL_N_JOINTS ) return NULL;
  
  if( motor_list[ motor_id ] == NULL ) return NULL;
  
  for( size_t state_index = 0; state_index < AXIS_N_STATES; state_index++ )
    states_list[ state_index ] = encoder_check_state( motor_list[ motor_id ]->encoder, state_index );
  
  return states_list;
}

extern inline double* control_get_motor_measures( size_t motor_id )
{
  static double measures_list[ AXIS_N_DIMS ];
  
  if( motor_id < 0 || motor_id >= CONTROL_N_JOINTS ) return NULL;
  
  if( motor_list[ motor_id ] == NULL ) return NULL;
  
  for( size_t dimension_index = 0; dimension_index < AXIS_N_DIMS; dimension_index++ )
    measures_list[ dimension_index ] = motor_get_measure( motor_list[ motor_id ], dimension_index );
  
  return measures_list;
}

extern inline const char* control_get_motor_name( size_t motor_id )
{
  if( motor_id < 0 || motor_id >= CONTROL_N_JOINTS ) return NULL;
  
  if( motor_list[ motor_id ] == NULL ) return NULL;
  
  return motor_list[ motor_id ]->name;
}

/////////////////////////////////////////////////////////////////////////////////
/////                           COMMON CONSTANTS                            /////
/////////////////////////////////////////////////////////////////////////////////

const double PI = 3.141592;        // Duh...
const double Ts = 0.005;           // Sampling interval

// Method that runs the control functions asyncronously
static void* async_control( void* args )
{
  unsigned long control_mode = (unsigned long) args;
  
  Control_Function control_function = control_functions[ control_mode ];
  
  unsigned int exec_time, elapsed_time;
  
  while( motor_list[ control_mode ] != NULL )
  {
    exec_time = get_exec_time_milisseconds();

    encoder_read( motor_list[ control_mode ]->encoder );
	
  	// Verify and try to correct errors
  	if( encoder_get_status( motor_list[ control_mode ]->encoder, FAULT ) == true )
  	  encoder_reset( motor_list[ control_mode ]->encoder );
  	if( sensor_list[ control_mode ] != NULL )
  	{
  	  if( encoder_get_status( sensor_list[ control_mode ], FAULT ) == true ) 
          encoder_reset( sensor_list[ control_mode ] );
  	}
    
    // If the motor is being actually controlled, call control pass algorhitm
    if( motor_list[ control_mode ]->active ) control_function( motor_list[ control_mode ], sensor_list[ control_mode ] );

    motor_control_write( motor_list[ control_mode ] );
    
    elapsed_time = get_exec_time_milisseconds() - exec_time;
    printf( "async_control (Axis %u): elapsed time: %u\n", control_mode, elapsed_time );
    if( elapsed_time < (int) ( 1000 * Ts ) ) delay( 1000 * Ts - elapsed_time );
    printf( "async_control (Axis %u): elapsed time: %u\n\n", control_mode, get_exec_time_milisseconds() - exec_time );
  }
  
  thread_exit( 0 );
  return NULL;
}

extern inline void control_mode_start( Joint mode )
{
  if( mode >= 0 && mode < CONTROL_N_JOINTS )
    motor_enable( motor_list[ mode ] );
}

extern inline void control_mode_stop( Joint mode )
{
  if( mode >= 0 && mode < CONTROL_N_JOINTS )
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

static void hips_control( Motor* hips_actuator, Encoder* hips_sensor )
{
  static double sensor_tension_zero, sensor_position_zero;

  // Sampling of last voltage signal values for filtering (average filter)
  static double analog_samples[ HIPS_CONTROL_SAMPLING_NUMBER ];

  static double ang_position_in_ref;
    
  if( sensor_tension_zero == 0 )
  {
    motor_set_operation_mode( hips_actuator, VELOCITY_MODE );
    
    sensor_tension_zero = encoder_get_measure( hips_actuator->encoder, TENSION ); // mV
    sensor_position_zero = ( TNS_2_POS_RATIO * sensor_tension_zero + TNS_2_POS_OFFSET ); // mm  
  }
  
  double sensor_tension = 0;
  for( int i = 0; i < HIPS_CONTROL_SAMPLING_NUMBER; i ++ )
  {
    analog_samples[ i ] = ( i == 0 ) ? encoder_get_measure( hips_actuator->encoder, TENSION ) : analog_samples[ i - 1 ]; // mV
    sensor_tension += analog_samples[ i ] / HIPS_CONTROL_SAMPLING_NUMBER;
  }

  double sensor_position = ( TNS_2_POS_RATIO * sensor_tension + TNS_2_POS_OFFSET );	// mm

  double actuator_force = -HIPS_FORCE_SENSOR_STIFFNESS * ( sensor_position_zero - sensor_position ); // N

  ang_position_in_ref = motor_get_parameter( hips_actuator, POSITION_SETPOINT );
  printf( "Angle setpoint: %g\n", ang_position_in_ref );

  // Relation between axis rotation and effector linear displacement
  double actuator_encoder_position = encoder_get_measure( hips_actuator, POSITION );
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

  Kq = motor_get_parameter( hips_actuator, PROPORTIONAL_GAIN );
  Bq = motor_get_parameter( hips_actuator, DERIVATIVE_GAIN );

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
  motor_set_parameter( hips_actuator, VELOCITY_SETPOINT, velocity_ref_rpm );
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
  
  if( ang_position_in == 0 ) motor_set_operation_mode( knee_in, VELOCITY_MODE );

  //ang_position_in_ref = ( encoder_get_measure( knee_in, POSITION_SETPOINT ) * 2 * PI ) / ENCODER_IN_RES;

  ang_position_in = ( ( encoder_get_measure( knee_in->encoder, POSITION ) /*- ang_position_in_ref*/ ) * 2 * PI ) / knee_in->encoder->resolution;
  ang_pos_in_reduced = ang_position_in / KNEE_JOINT_REDUCTION;

  ang_velocity_in = encoder_get_measure( knee_in->encoder, VELOCITY );

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
}

/////////////////////////////////////////////////////////////////////////////////
/////                         ANKLE CONTROL FUNCTION                        /////
/////////////////////////////////////////////////////////////////////////////////

static void ankle_control( Motor* ankle_actuator, Encoder* ankle_sensor )
{
  static int velocity_ref;  
    
  if( velocity_ref == 0 ) motor_set_operation_mode( ankle_actuator, VELOCITY_MODE );

  if( encoder_get_measure( ankle_actuator, POSITION_SETPOINT ) > 2000 ) motor_set_parameter( ankle_actuator, POSITION_SETPOINT, 2000 );
  else if( encoder_get_measure( ankle_actuator, POSITION_SETPOINT ) < -2000 ) motor_set_parameter( ankle_actuator, POSITION_SETPOINT, -2000 );
  
  velocity_ref = -( encoder_get_measure( ankle_actuator, POSITION ) - encoder_get_measure( ankle_actuator, POSITION_SETPOINT ) ) / 20;
  
  motor_set_parameter( ankle_actuator, VELOCITY_SETPOINT, velocity_ref );
}

#endif /* CONTROL_H */ 
