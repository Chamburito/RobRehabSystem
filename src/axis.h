/* 
 * File:   AXIS.h
 * Author: GUILHERME FERNANDES
 *
 * ESTA FUNÇÃO REALIZA O CONTROLE DE UM EIXO DA EPOS
 * 
 * Created on 26 de Janeiro de 2012, 19:34
 */

#ifndef AXIS_H
#define	AXIS_H

#include "epos_network.h"

#ifdef WIN32
  #include "timing_windows.h"
#else
  #include "timing_unix.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include <string.h>

enum OperationMode { HOMMING_MODE = 0x06, PROFILE_VELOCITY_MODE = 0x03, PROFILE_POSITION_MODE = 0x01,
	POSITION_MODE = 0xFF, VELOCITY_MODE = 0xFE, CURRENT_MODE = 0xFD, MASTER_ENCODER_MODE = 0xFB, STEP_MODE = 0xFA };

enum Status { READY_2_SWITCH_ON = 1, SWITCHED_ON = 2, OPERATION_ENABLED = 4, FAULT = 8, VOLTAGE_ENABLED = 16,
	QUICK_STOPPED = 32, SWITCH_ON_DISABLE = 64, REMOTE_NMT = 512, TARGET_REACHED = 1024, SETPOINT_ACK = 4096 };

enum Control { SWITCH_ON = 1, ENABLE_VOLTAGE = 2, QUICK_STOP = 4, ENABLE_OPERATION = 8, NEW_SETPOINT = 16,
	CHANGE_IMMEDIATEDLY = 32, ABS_REL = 64, FAULT_RESET = 128, HALT = 256 };

//enum Dimension { POSITION, POSITION_SETPOINT, VELOCITY, VELOCITY_SETPOINT, CURRENT, TENSION, ANGLE, ANGLE_SETPOINT, FORCE, N_DIMS }; 

enum Dimension { POSITION, VELOCITY, CURRENT, TENSION, ANGLE, FORCE, N_DIMS }; 

enum Parameter { POSITION_SETPOINT, VELOCITY_SETPOINT, CURRENT_SETPOINT, PROPORTIONAL_GAIN, DERIVATIVE_GAIN, N_PARAMS }; 

enum FrameType { SDO_TX, SDO_RX, PDO01_RX, PDO01_TX, PDO02_RX, PDO02_TX, N_FRAMES };
static const char* frame_names[ N_FRAMES ] = { "SDO_TX_0", "SDO_RX_0", "PDO01_RX_0", "PDO01_TX_0", "PDO02_RX_0", "PDO02_TX_0" };

typedef struct _Axis_Sensor
{
  CAN_Frame* frame_list[ N_FRAMES ];
  double measures[ N_DIMS ];
  short unsigned int status_word, control_word;
  short int output;
}
Axis_Sensor;

typedef struct _Axis
{
  Axis_Sensor* sensor;
  double parameters[ N_PARAMS ];
  bool active;
}
Axis;

Axis* axis_create( unsigned int );
Axis_Sensor* axis_sensor_create( unsigned int );
void axis_enable( Axis* );
void axis_disable( Axis* );
void axis_destroy( Axis* );
void axis_sensor_destroy( Axis_Sensor* );
double axis_sensor_get_measure( Axis_Sensor*, Dimension );
//void axis_set_reference( Axis*, Dimension, double );   
double axis_get_parameter( Axis*, Parameter );
void axis_set_parameter( Axis*, Parameter, double );
void axis_sensor_set_output( Axis_Sensor*, int );
bool axis_sensor_get_status( Axis_Sensor*, Status );
void axis_sensor_reset( Axis_Sensor* );
static void axis_sensor_set_control( Axis_Sensor*, Control, bool );
void axis_sensor_read( Axis_Sensor* );
static void axis_sensor_write( Axis_Sensor* );
void axis_control_write( Axis* );
static double sensor_read_single_value( Axis_Sensor*, int, u8 );
static void sensor_write_single_value( Axis_Sensor*, int, u8, short int );
void axis_set_operation_mode( Axis*, OperationMode );
void axis_set_digital_output( Axis*, bool );

// Create CAN controlled DC motor handle
Axis* axis_create( unsigned int network_index ) 
{
  Axis* axis = (Axis*) malloc( sizeof(Axis) );

  for( int i = 0; i < N_PARAMS; i++ )
    axis->parameters[ i ] = 0.0;

  axis->sensor = axis_sensor_create( network_index );

  if( axis->sensor != NULL ) axis_disable( axis );
  else axis_destroy( axis );

  return axis;
}

Axis_Sensor* axis_sensor_create( unsigned int network_index )
{
  Axis_Sensor* sensor = (Axis_Sensor*) malloc( sizeof(Axis_Sensor) );

  for( int i = 0; i < N_DIMS; i++ )
    sensor->measures[ i ] = 10.0;

  char network_address[ 20 ];

  for( int frame_id = 0; frame_id < N_FRAMES; frame_id += 2 )
  {
    sprintf( network_address, "%s%u", frame_names[ frame_id ], network_index );
    printf( "axis_init: creating frame %s\n", network_address );
    sensor->frame_list[ frame_id ] = epos_network_init_frame( FRAME_OUT, "CAN2", network_address );

    if( sensor->frame_list[ frame_id ] == NULL ) axis_sensor_destroy( sensor );
    
    sprintf( network_address, "%s%u", frame_names[ frame_id + 1 ], network_index );
    printf( "axis_init: creating frame %s\n", network_address );
    sensor->frame_list[ frame_id + 1 ] = epos_network_init_frame( FRAME_IN, "CAN1", network_address );

    if( sensor->frame_list[ frame_id + 1 ] == NULL ) axis_sensor_destroy( sensor );
  }

  return sensor;
}

void axis_destroy( Axis* axis )
{
  if( axis != NULL )
  {
    if( axis->active) axis_disable( axis );
    
    axis_sensor_destroy( axis->sensor );

    free( axis );
    axis = NULL;
  }
}

void axis_sensor_destroy( Axis_Sensor* sensor )
{
  if( sensor != NULL )
  {
    for( size_t frame_id = 0; frame_id < N_FRAMES; frame_id++ )
      can_frame_end( sensor->frame_list[ frame_id ] );

    free( sensor );
    sensor = NULL;
  }
}

void axis_enable( Axis* axis )
{
  axis_sensor_set_control( axis->sensor, SWITCH_ON, false );
  axis_sensor_set_control( axis->sensor, ENABLE_VOLTAGE, true );
  axis_sensor_set_control( axis->sensor, QUICK_STOP, true );
  axis_sensor_set_control( axis->sensor, ENABLE_OPERATION, false );
  axis_sensor_write( axis->sensor );
        
  delay( 500 );
        
  axis_sensor_set_control( axis->sensor, SWITCH_ON, true );
  axis_sensor_set_control( axis->sensor, ENABLE_VOLTAGE, true );
  axis_sensor_set_control( axis->sensor, QUICK_STOP, true );
  axis_sensor_set_control( axis->sensor, ENABLE_OPERATION, false );
  axis_sensor_write( axis->sensor );

  delay( 500 );
        
  axis_sensor_set_control( axis->sensor, SWITCH_ON, true );
  axis_sensor_set_control( axis->sensor, ENABLE_VOLTAGE, true );
  axis_sensor_set_control( axis->sensor, QUICK_STOP, true );
  axis_sensor_set_control( axis->sensor, ENABLE_OPERATION, true );
  axis_sensor_write( axis->sensor );

  axis->active = true;
}

void axis_disable( Axis* axis )
{
  axis_sensor_set_control( axis->sensor, SWITCH_ON, true );
  axis_sensor_set_control( axis->sensor, ENABLE_VOLTAGE, true );
  axis_sensor_set_control( axis->sensor, QUICK_STOP, true );
  axis_sensor_set_control( axis->sensor, ENABLE_OPERATION, false );
  axis_sensor_write( axis->sensor );
        
  delay( 500 );
        
  axis_sensor_set_control( axis->sensor, SWITCH_ON, false );
  axis_sensor_set_control( axis->sensor, ENABLE_VOLTAGE, true );
  axis_sensor_set_control( axis->sensor, QUICK_STOP, true );
  axis_sensor_set_control( axis->sensor, ENABLE_OPERATION, false );
  axis_sensor_write( axis->sensor );

  for( size_t param_index; param_index < N_PARAMS; param_index++ )
    axis_set_parameter( axis, (Parameter) param_index, 0 );
  axis_control_write( axis );

  axis->active = false;
}

double axis_sensor_get_measure( Axis_Sensor* sensor, Dimension index )
{
  if( index >= N_DIMS )
  {
    fprintf( stderr, "axis_sensor_get_measure: invalid value index: %d\n", index );
    return 0;
  }
      
  return sensor->measures[ index ];
}

/*void axis_set_reference( Axis* axis, Dimension index, double value )
{
  if( index >= N_DIMS )
  {
    fprintf( stderr, "PDOsetValue: invalid value index: %d\n", index );
    return;
  }

  axis->dimension_values[ index ] = value;
}*/
 
double axis_get_parameter( Axis* axis, Parameter index )
{
  if( index >= N_PARAMS )
  {
    fprintf( stderr, "axis_get_parameter: invalid value index: %d\n", index );
    return 0;
  }
      
  return axis->parameters[ index ];
}

void axis_set_parameter( Axis* axis, Parameter index, double value )
{
  if( index >= N_PARAMS )
  {
    fprintf( stderr, "axis_set_parameter: invalid value index: %d\n", index );
    return;
  }
      
  axis->parameters[ index ] = value;
}

void axis_sensor_set_output( Axis_Sensor* sensor, bool enabled )
{
  if( enabled ) sensor->output = 0x2000;
  else sensor->output = 0x0000;
}

bool axis_sensor_get_status( Axis_Sensor* sensor, Status index )
{
  if( (sensor->status_word & index) > 0 ) return true;
  else return false;
}

void axis_sensor_reset( Axis_Sensor* sensor )
{
  axis_sensor_set_control( sensor, FAULT_RESET, true );
  delay( 100 );
  axis_sensor_set_control( sensor, FAULT_RESET, false );
}

static void axis_sensor_set_control( Axis_Sensor* sensor, Control index, bool enabled )
{
  if( enabled ) sensor->control_word |= index;
  else sensor->control_word &= (~index);
}

void axis_sensor_read( Axis_Sensor* sensor )
{
  // Create reading buffer
  static u8 payload[8];

  // Read values from PDO01 (Position, Current and Status Word) to buffer
  can_frame_read( frame_list[ PDO01_RX ], payload );  
  // Update values from PDO01
  sensor->measures[ POSITION ] = payload[3] * 0x1000000 + payload[2] * 0x10000 + payload[1] * 0x100 + payload[0];
  sensor->measures[ CURRENT ] = payload[5] * 0x100 + payload[4];
  sensor->status_word = payload[7] * 0x100 + payload[6];

  // Read values from PDO02 (Position, Current and Status Word) to buffer
  can_frame_read( frame_list[ PDO02_RX ], payload );  
  // Update values from PDO02
	sensor->measures[ VELOCITY ] = payload[3] * 0x1000000 + payload[2] * 0x10000 + payload[1] * 0x100 + payload[0];
	sensor->measures[ TENSION ] = payload[5] * 0x100 + payload[4];
}

static void axis_sensor_write( Axis_Sensor* sensor )
{
  // Build writing buffer
  static u8 payload[8];
  payload[6] = ( sensor->control_word & 0x000000ff );
  payload[7] = ( sensor->control_word & 0x0000ff00 ) / 0x100;

  // Write values from buffer to PDO01 
  can_frame_write( sensor->frame_list[ PDO01_TX ], payload );
}

void axis_control_write( Axis* axis )
{
  // Create writing buffer
  static u8 payload[8];

  // Set values for PDO01 (Position Setpoint and Control Word)
  payload[0] = ( (int) axis->parameters[ POSITION_SETPOINT ] & 0x000000ff );
  payload[1] = ( (int) axis->parameters[ POSITION_SETPOINT ] & 0x0000ff00 ) / 0x100;
  payload[2] = ( (int) axis->parameters[ POSITION_SETPOINT ] & 0x00ff0000 ) / 0x10000;
  payload[3] = ( (int) axis->parameters[ POSITION_SETPOINT ] & 0xff000000 ) / 0x1000000;
  payload[4] = ( 0 & 0x000000ff );
  payload[5] = ( 0 & 0x0000ff00 ) / 0x100; 
  payload[6] = ( axis->sensor->control_word & 0x000000ff );
  payload[7] = ( axis->sensor->control_word & 0x0000ff00 ) / 0x100; 

  // Write values from buffer to PDO01 
  can_frame_write( axis->sensor->frame_list[ PDO01_TX ], payload );

  // Set values for PDO02 (Velocity Setpoint and Output)
  payload[0] = ( (int) axis->parameters[ VELOCITY_SETPOINT ] & 0x000000ff );
  payload[1] = ( (int) axis->parameters[ VELOCITY_SETPOINT ] & 0x0000ff00 ) / 0x100;
  payload[2] = ( (int) axis->parameters[ VELOCITY_SETPOINT ] & 0x00ff0000 ) / 0x10000;
  payload[3] = ( (int) axis->parameters[ VELOCITY_SETPOINT ] & 0xff000000 ) / 0x1000000;
  payload[4] = ( axis->sensor->output & 0x000000ff );
  payload[5] = ( axis->sensor->output & 0x0000ff00 ) / 0x100; 
  payload[6] = ( 0 & 0x000000ff );
  payload[7] = ( 0 & 0x0000ff00 ) / 0x100; 

  // Write values from buffer to PDO01 E
  can_frame_write( axis->sensor->frame_list[ PDO02_TX ], payload );
}

static double sensor_read_single_value( Axis_Sensor* sensor, int index, u8 sub_index )
{
  // Build read requisition buffer for defined value
  static u8 payload[8];
  payload[0] = 0x40; 
  payload[1] = ( index & 0x000000ff );
  payload[2] = ( index & 0x0000ff00 ) / 0xff;
  payload[3] = sub_index;
  payload[4] = 0x0;
  payload[5] = 0x0;
  payload[6] = 0x0;
  payload[7] = 0x0;

  // Write value requisition to SDO frame 
  can_frame_write( sensor->frame_list[ SDO_TX ], payload );

  delay( 100 );

  // Read requested value from SDO frame
  can_frame_read( sensor->frame_list[ SDO_RX ], payload );

  double value = payload[7] * 0x1000000 + payload[6] * 0x10000 + payload[5] * 0x100 + payload[4];

  return value; 
}

static void sensor_write_single_value( Axis_Sensor* sensor, int index, u8 sub_index, short int value )
{
  // Build write buffer
  static u8 payload[8];
  payload[0] = 0x22; 
  payload[1] = ( index & 0x000000ff );
  payload[2] = ( index & 0x0000ff00 ) / 0x100;
  payload[3] = sub_index;
  payload[4] = ( value & 0x000000ff );
  payload[5] = ( value & 0x0000ff00 ) / 0x100;
  payload[6] = ( value & 0x00ff0000 ) / 0x10000;
  payload[7] = ( value & 0xff000000 ) / 0x1000000;

  // Write value to SDO frame 
  can_frame_write( sensor->frame_list[ SDO_TX ], payload );
}

void axis_set_operation_mode( Axis* axis , OperationMode mode )
{
  sensor_write_single_value( axis->sensor, 0x6060, 0x00, mode );
}

void axis_sensor_set_digital_output( Axis_Sensor* sensor, bool enabled )
{
  if( enabled ) sensor_write_single_value( sensor, 0x6060, 0x00, 0xff );
  else sensor_write_single_value( sensor, 0x6060, 0x00, 0x00 );
}


#endif	/* AXIS_H */

