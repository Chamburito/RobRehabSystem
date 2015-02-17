#ifndef CONTROL_H
#define CONTROL_H

#include "axis.h"
#include "epos_network.h"

#ifdef WIN32
  #include "threads_windows.h"
#else
  #include "threads_unix.h"
#endif

#include <cmath>
#include <iostream>

using namespace std;
  
enum Joint { HIPS = 2, KNEE = 1, ANKLE = 0 };  // Control algorhitms indexes

/////////////////////////////////////////////////////////////////////////////////
/////                            CONTROL DEVICES                            /////
/////////////////////////////////////////////////////////////////////////////////

//CAN database addressing
const char* CAN_DATABASE = "database";
const char* CAN_CLUSTER = "NETCAN";

// EPOS axes list
const int N_AXES = 3;
Axis* axis_list[ N_AXES ];

// EPOS devices and CAN network initialization
void control_init()
{
  // Start CAN network transmission
  for( int node_id = 1; node_id <= N_AXES; node_id++ )
    epos_network_start( CAN_DATABASE, CAN_CLUSTER, node_id );
  
  // Create axes structures
  for( int axis_id = 0; axis_id < N_AXES; axis_id++ )
    axis_list[ axis_id ] = axis_create();
}

// EPOS devices and CAN network shutdown
void control_end()
{
  // End CAN network transmission
  epos_network_stop( 1 );

  delay( 2000 );
    
	// End threading subsystem
  end_threading();
}

/////////////////////////////////////////////////////////////////////////////////
/////                           COMMON CONSTANTS                            /////
/////////////////////////////////////////////////////////////////////////////////

const double PI = 3.141592;
const double Ts = 0.005;				  // sampling interval


/////////////////////////////////////////////////////////////////////////////////
/////                            HIPS CONSTANTS                             /////
/////////////////////////////////////////////////////////////////////////////////

const double TNS_2_POS_RATIO = 0.0168;
const double TNS_2_POS_OFFSET = 2.5749;

const double HIPS_FORCE_SENSOR_STIFFNESS = 78.9; // N/m
const double HIPS_ACTUATOR_STEP = 3e-3;

enum { A, B, C, D, C2D, A2B };
const double HIPS_STRUCT_DIMS[] = { 0.063, 0.060, 0.284, 0.117, sqrt( 0.284 * 0.284 + 0.117 * 0.117 ), sqrt( 0.063 * 0.063 + 0.060 * 0.060 ) };

const double HIPS_EFFECTOR_BASE_LENGTH = 0.348;

enum { ALPHA, BETA };
const double HIPS_STRUCT_ANGLES[] = { atan( HIPS_STRUCT_DIMS[ A ] / HIPS_STRUCT_DIMS[ B ] ), atan( HIPS_STRUCT_DIMS[ C ] / HIPS_STRUCT_DIMS[ D ] ) };

/////////////////////////////////////////////////////////////////////////////////
/////                    HIPS CONTROL DEFAULT VALUES                        /////
/////////////////////////////////////////////////////////////////////////////////

// GainsHIPS_FORCE_SENSOR_STIFFNESS
double Kq = 10.0;
double Bq = 10.0;

// Sampling
const int HIPS_CONTROL_SAMPLING_NUMBER = 6;

/////////////////////////////////////////////////////////////////////////////////
/////                         HIPS CONTROL FUNCTION                         /////
/////////////////////////////////////////////////////////////////////////////////

static void* hips_control( void* args )
{
  Axis* hips = axis_list[ 2 ];

  double tension_ref = axis_get_measure( hips, TENSION ); // mV
  double position_ref = ( TNS_2_POS_RATIO * tension_ref + TNS_2_POS_OFFSET ); // mm

  // Sampling of last voltage signal values for filtering (average filter)
  static double analog_samples[ HIPS_CONTROL_SAMPLING_NUMBER ];
  
  int exec_time, elapsed_time;

  axis_set_operation_mode( hips, VELOCITY_MODE );

  static double theta_ref; // ?

  while( hips->active/*axis_get_status( hips, SWITCHED_ON )*/ )
  {
    exec_time = get_exec_time_milisseconds();

    axis_read_values( hips );
    
    double tension = 0;
    for( int i = 0; i < HIPS_CONTROL_SAMPLING_NUMBER; i ++ )
    {
      if( i == 0 ) analog_samples[ i ] = axis_get_measure( hips, TENSION ); // mV
      else analog_samples[ i ] = analog_samples[ i - 1 ];
    
      tension += analog_samples[ i ] / HIPS_CONTROL_SAMPLING_NUMBER; // mV;
    } 

    double position = ( TNS_2_POS_RATIO * tension + TNS_2_POS_OFFSET );	// mm

    double actuator_force = -HIPS_FORCE_SENSOR_STIFFNESS * ( position_ref - position ); // N

    theta_ref = axis_get_measure( hips, POSITION_SETPOINT );
    printf( "Angle setpoint: %g\n", theta_ref );

    // 4-bar (A, B, C, D, in meters) mechanism

    int actuator_encoder_position = axis_get_measure( hips, POSITION );
    double effector_delta_length = ( actuator_encoder_position / 4096 ) * HIPS_ACTUATOR_STEP + ( position_ref - position ) / 1000; // m

    double effector_length = HIPS_EFFECTOR_BASE_LENGTH + effector_delta_length; // m

    double cos_eta = ( effector_length * effector_length + pow( HIPS_STRUCT_DIMS[ A2B ], 2 ) - pow( HIPS_STRUCT_DIMS[ C2D ], 2 ) ) 
                         / ( 2 * effector_length * HIPS_STRUCT_DIMS[ A2B ] );
    double sin_eta = sqrt( (double)( 1 - cos_eta * cos_eta ) );
    
    double gama = acos( ( pow( HIPS_STRUCT_DIMS[ C2D ], 2 ) + pow( HIPS_STRUCT_DIMS[ A2B ], 2 ) - effector_length * effector_length ) 
                            / ( 2 * HIPS_STRUCT_DIMS[ C2D ] * HIPS_STRUCT_DIMS[ A2B ] ) );

    double theta = HIPS_STRUCT_ANGLES[ ALPHA ] + HIPS_STRUCT_ANGLES[ BETA ] + gama - PI;

    double theta_2_length_ratio = sqrt( pow( HIPS_STRUCT_DIMS[ C2D ], 2 ) + pow( HIPS_STRUCT_DIMS[ A2B ], 2 ) 
                                        - 2 * HIPS_STRUCT_DIMS[ C2D ] * HIPS_STRUCT_DIMS[ A2B ] * cos( gama ) ) 
                                      / ( HIPS_STRUCT_DIMS[ C2D ] * HIPS_STRUCT_DIMS[ A2B ] * sin( gama ) );

    Kq = axis_get_parameter( hips, KP );
    Bq = axis_get_parameter( hips, KD );

    if( (int) Kq == 0 ) Kq = 1.0;
    if( (int) Bq == 0 ) Bq = 1.0;
  
    double actuator_stiffness = Kq * theta_2_length_ratio * ( 1 / ( HIPS_STRUCT_DIMS[ A2B ] * sin_eta ) );
    double actuator_damping = Bq * theta_2_length_ratio * ( 1 / ( HIPS_STRUCT_DIMS[ A2B ] * sin_eta ) );

    double Fv_q = ( Kq * theta_ref ) / ( HIPS_STRUCT_DIMS[ A2B ] * sin_eta );

    // Control law

    double velocity_ref = ( Fv_q - actuator_stiffness * ( ( actuator_encoder_position / 4096 ) * HIPS_ACTUATOR_STEP ) 
                            + ( ( actuator_stiffness - HIPS_FORCE_SENSOR_STIFFNESS ) / HIPS_FORCE_SENSOR_STIFFNESS ) * actuator_force ) / actuator_damping;

    int velocity_ref_rpm = ( velocity_ref / HIPS_ACTUATOR_STEP ) * 60 / ( 2 * PI ); // rpm
    
    //cout << "angle: " << theta << "  vel ref: " << velocity_ref_rpm << "  force: " << actuator_force << endl;  

    axis_set_reference( hips, ANGLE, 180 * theta / PI );
    axis_set_reference( hips, ANGLE_SETPOINT, 180 * theta_ref / PI );
    axis_set_reference( hips, VELOCITY_SETPOINT, velocity_ref_rpm );
    axis_set_reference( hips, FORCE, actuator_force );
    axis_write_values( hips );
    
    elapsed_time = get_exec_time_milisseconds() - exec_time;
    printf( "hips_control: elapsed time: %d\n", elapsed_time );
    if( elapsed_time < (int) ( 1000 * Ts ) ) delay( 1000 * Ts - elapsed_time );
    printf( "hips_control: elapsed time: %d\n", get_exec_time_milisseconds() - exec_time );
    cout << endl;
  }

  axis_set_reference( hips, VELOCITY_SETPOINT, 0 );
  axis_write_values( hips );

  exit_thread( 0 );
  return NULL;
}


/////////////////////////////////////////////////////////////////////////////////
////////// CONSTANTES JOELHO ////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

const int N = 150;						            // Redução da junta
const int ENCODER_IN_RES = 4096;			    // Resolução do encoder do motor
const int ENCODER_OUT_RES = 2000;		      // Resolução do encoder de saída
const int Ks = 104;						            // Constante elástica

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

const double kp = 370;				    // Ganho proporcional
const double ki = 3.5;					// Ganho integrativo
const double kd = 0.0;					// Ganho derivativo

/////////////////////////////////////////////////////////////////////////////////
////////// GANHOS JOELHO ////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

double Kv = 0.0;
double Bv = 5.0;

/////////////////////////////////////////////////////////////////////////////////
////////// FUNÇÃO JOELHO ////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

static void* kneeControl( void* args )
{
  Axis* epos = (Axis*) args;
  
  int execTime, elapsedTime;

  epos->VCS_SetOperationMode( VELOCITY_MODE );

  //epos->SetControlValue( KP, Kv );
  //epos->SetControlValue( KD, Bv );

  static double omega_l[3];
  static double omega_lf[3];
  static double theta_l[2];
  static double torque_l[3];
  static double torque_lf[3];
  static double control[2];
  static double error[3];

  static double theta_ld, omega_ld; // ?

  //epos->ReadSetpoints( "theta_j.txt" );

  while( epos->active/*epos->PDOgetStatusWord( SWITCHED_ON )*/ )
  {
    execTime = get_exec_time_milisseconds();

    double Im = epos->PDOgetValue( CURRENT );
    //double theta_c = ( ( epos->PDOgetValue( POSITION ) - epos->PDOgetValue( POSITION_SETPOINT ) ) * 2 * PI ) / ( ENCODER_IN_RES * N );
    //double theta_m = ( ( epos->PDOgetValue( POSITION ) - epos->PDOgetValue( POSITION_SETPOINT ) ) * 2 * PI ) / ENCODER_IN_RES;
    double theta_c = ( ( epos->PDOgetValue( POSITION ) ) * 2 * PI ) / ( ENCODER_IN_RES * N );
    double theta_m = ( ( epos->PDOgetValue( POSITION ) ) * 2 * PI ) / ENCODER_IN_RES;
        
    double omega_m = epos->PDOgetValue( VELOCITY );

    //theta_ld = epos->GetSetpoint();
    //theta_ld = epos->PDOgetValue( POSITION_SETPOINT );
    //omega_ld = epos->PDOgetValue( VELOCITY_SETPOINT );

    //---------------Controle de Impedância--------------------//
      
    Kv = epos->GetControlValue( KP );
    Bv = epos->GetControlValue( KD );

    cout << "Kv: " << Kv << " - Bv: " << Bv << endl;
	  
    omega_l[0] = ( theta_l[0] - theta_l[1] ) / Ts;
    omega_lf[0] = -c2 * omega_lf[1] - c3 * omega_lf[2] + d1 * omega_l[0] + d2 * omega_l[1] + d3 * omega_l[2];

    //theta_l[0] = ( ( -EPOS[0].PDOgetValue( POSITION ) - EPOS[0].PDOgetValue( POSITION_SETPOINT ) ) * 2 * PI ) / ENCODER_OUT_RES;
    theta_l[0] = ( ( -EPOS[0].PDOgetValue( POSITION ) ) * 2 * PI ) / ENCODER_OUT_RES;

    double torque_d = -Kv * ( theta_l[0] - theta_ld ) - Bv * ( omega_lf[0] - omega_ld );

    epos->PDOsetValue( ANGLE, (int) ( 1000 * theta_l[0] ) );
    //cout << "epos 2: theta_l: " << theta_l[0] << endl;

    torque_l[0] = Ks * ( theta_c - theta_l[0] );
    epos->PDOsetValue( FORCE, (int) torque_l[0] );

    torque_lf[0] = -a2 * torque_lf[1] - a3 * torque_lf[2] + b1 * torque_l[0] + b2 * torque_l[1] + b3 * torque_l[2];

    error[0] = ( torque_d - torque_lf[0] );

    control[0] = control[1] + kp * ( error[0] - error[1] ) + ki * Ts * error[0] + ( kd / Ts ) * ( error[0] - 2 * error[1] + error[2] );
    //controle = (0.9872*controle2_ant + 0.009265*controle2_ant2 + 333.3*erro_ant - 332*erro_ant2);

    error[2] = error[1];
    error[1] = error[0];

    theta_l[1] = theta_l[0];

    control[1] = control[0];

    omega_l[2] = omega_l[1];
    omega_l[1] = omega_l[0];

    omega_lf[2] = omega_lf[1];
    omega_lf[1] = omega_lf[0];

    torque_l[2] = torque_l[1];
    torque_l[1] = torque_l[0];

    torque_lf[2] = torque_lf[1];
    torque_lf[1] = torque_lf[0];

    epos->PDOsetValue( ANGLE, 180 * theta_l[0] / PI );
    epos->PDOsetValue( ANGLE_SETPOINT, 180 * theta_ld / PI );
    epos->PDOsetValue( VELOCITY_SETPOINT, (int) control[0] );
    epos->WritePDO02();

    elapsedTime = get_exec_time_milisseconds() - execTime;
    cout << "controle: " << control[0] << endl;
    cout << "elapsed time: " << elapsedTime << endl;
    //delay( 5 - elapsedTime );
    if( elapsedTime < (int) ( 1000 * Ts ) ) delay( 1000 * Ts - elapsedTime );
    cout << "elapsed time + sleep: " << get_exec_time_milisseconds() - execTime << endl;
    cout << endl;
  }
  
  epos->PDOsetValue( VELOCITY_SETPOINT, 0 );
  epos->WritePDO02();

  exit_thread( 0 );
  return NULL;
}

static void* ankleControl( void* args )
{
  Axis* epos = (Axis*) args;
  
  int execTime, elapsedTime;

  int velocitySetpoint;

  while( epos->active/*epos->PDOgetStatusWord( SWITCHED_ON )*/ )
  {
     execTime = get_exec_time_milisseconds();

	 epos->VCS_SetOperationMode( VELOCITY_MODE );

	 if( epos->PDOgetValue( POSITION_SETPOINT ) > 2000 ) epos->PDOsetValue( POSITION_SETPOINT, 2000 );
	 else if( epos->PDOgetValue( POSITION_SETPOINT ) < -2000 ) epos->PDOsetValue( POSITION_SETPOINT, -2000 );

	 velocitySetpoint = -( epos->PDOgetValue( POSITION ) - epos->PDOgetValue( POSITION_SETPOINT ) ) / 20;

	 epos->PDOsetValue( VELOCITY_SETPOINT, velocitySetpoint );
     epos->WritePDO02();

	 elapsedTime = get_exec_time_milisseconds() - execTime;
     //cout << "ankleControl: elapsed time: " << elapsedTime << endl;
     if( elapsedTime < (int) ( 1000 * Ts ) ) delay( 1000 * Ts - elapsedTime );
     //cout << "ankleControl: elapsed time + sleep: " << get_exec_time_milisseconds() - execTime << endl;
     //cout << endl;
  }

  epos->PDOsetValue( VELOCITY_SETPOINT, 0 );
  epos->WritePDO02();

  exit_thread( 0 );
  return NULL;
}


typedef void* (*async_function)( void* ); // Use "async_function" type as "void (*)( void* )" type

async_function functions[] = { ankleControl, kneeControl, hipsControl };

async_function controlFunction( Joint index )
{
  if( index >= 0 && index < 3 )
    return functions[ index ];
  else
    return NULL;
}

#endif /* CONTROL_H */ 
