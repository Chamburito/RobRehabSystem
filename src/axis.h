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

enum Dimension { POSITION, POSITION_SETPOINT, VELOCITY, VELOCITY_SETPOINT, CURRENT, TENSION, ANGLE, ANGLE_SETPOINT, FORCE, N_DIMS }; 

enum Parameter { KP, KD, N_PARAMS }; 

enum FrameType { SDO_TX, SDO_RX, PDO01_RX, PDO01_TX, PDO02_RX, PDO02_TX, N_FRAMES };
static const char* frame_names[ N_FRAMES ] = { "SDO_TX_0", "SDO_RX_0", "PDO01_RX_0", "PDO01_TX_0", "PDO02_RX_0", "PDO02_TX_0" };

typedef struct _Axis
{
  CAN_Frame* frame_list[ N_FRAMES ];
  double dimension_values[ N_DIMS ];
  double control_values[ N_PARAMS ];
  short unsigned int status_word, control_word;
  short int output;
  bool active;
}
Axis;

// Created axes count
static size_t n_axes = 0;

Axis* axis_create();
void axis_enable( Axis* );
void axis_disable( Axis* );
void axis_destroy( Axis* );
double axis_get_dimension_value( Dimension );
void axis_set_dimension_value( Dimension, double );   
double axis_get_parameter( Axis*, Parameter );
void axis_set_parameter( Axis*, Parameter, double );
void axis_set_output( Axis*, int );
bool axis_get_status_value( Axis*, Status );
void axis_set_control_value( Axis*, Control, bool );
void axis_read_values( Axis* );
void axis_write_values( Axis* );
double axis_read_single_value( Axis*, int, u8 );
void axis_write_single_value( Axis*, int, u8, short int );
void axis_set_operation_mode( Axis*, OperationMode );
void axis_set_digital_output( Axis*, bool );

// Create CAN controlled DC motor handle
Axis* axis_create() 
{
  Axis* axis = (Axis*) malloc( sizeof(Axis) );

  for( int i = 0; i < N_DIMS; i++ )
    axis->dimension_values[ i ] = 0.0;
  for( int i = 0; i < N_PARAMS; i++ )
    axis->control_values[ i ] = 10.0;

  char network_address[20];
	
  for( int frame_id = 0; frame_id < N_FRAMES; frame_id += 2 )
  {
    sprintf( network_address, "%s%u", frame_names[ frame_id ], n_axes + 1 );
    printf( "axis_init: creating frame %s\n", network_address );
    axis->frame_list[ frame_id ] = epos_network_init_frame( FRAME_OUT, "CAN2", network_address );

    if( axis->frame_list[ frame_id ] == NULL ) axis_destroy( axis );
		    
    sprintf( network_address, "%s%u", frame_names[ frame_id + 1 ], n_axes + 1 );
    printf( "axis_init: creating frame %s\n", network_address );
    axis->frame_list[ frame_id + 1 ] = epos_network_init_frame( FRAME_IN, "CAN1", network_address );

    if( axis->frame_list[ frame_id + 1 ] == NULL ) axis_destroy( axis );
  }

  if( axis != NULL )
  {
    axis_disable( axis );
    n_axes++;
  }

  return axis;
}

void axis_enable( Axis* axis )
{
  axis_set_control_value( axis, SWITCH_ON, false );
  axis_set_control_value( axis, ENABLE_VOLTAGE, true );
  axis_set_control_value( axis, QUICK_STOP, true );
  axis_set_control_value( axis, ENABLE_OPERATION, false );
  axis_write_values( axis );
        
  delay( 500 );
        
  axis_set_control_value( axis, SWITCH_ON, true );
  axis_set_control_value( axis, ENABLE_VOLTAGE, true );
  axis_set_control_value( axis, QUICK_STOP, true );
  axis_set_control_value( axis, ENABLE_OPERATION, false );
  axis_write_values( axis );

  delay( 500 );
        
  axis_set_control_value( axis, SWITCH_ON, true );
  axis_set_control_value( axis, ENABLE_VOLTAGE, true );
  axis_set_control_value( axis, QUICK_STOP, true );
  axis_set_control_value( axis, ENABLE_OPERATION, true );
  axis_write_values( axis );

  axis->active = true;
}

void axis_disable( Axis* axis )
{
  axis_set_control_value( axis, SWITCH_ON, true );
  axis_set_control_value( axis, ENABLE_VOLTAGE, true );
  axis_set_control_value( axis, QUICK_STOP, true );
  axis_set_control_value( axis, ENABLE_OPERATION, false );
  axis_write_values( axis );
        
  delay( 500 );
        
  axis_set_control_value( axis, SWITCH_ON, false );
  axis_set_control_value( axis, ENABLE_VOLTAGE, true );
  axis_set_control_value( axis, QUICK_STOP, true );
  axis_set_control_value( axis, ENABLE_OPERATION, false );
  axis_write_values( axis );

  axis->active = false;
}

void axis_destroy( Axis* axis )
{
  for( size_t frame_id = 0; frame_id < N_FRAMES; frame_id++ )
    can_frame_end( axis->frame_list[ frame_id ] );

  free( axis );
  axis = NULL;
}

double axis_get_dimension_value( Axis* axis, Dimension index )
{
  if( index >= N_DIMS )
  {
    fprintf( stderr, "PDOgetValue: invalid value index: %d\n", index );
    return 0;
  }
      
  return axis->dimension_values[ index ];
}

void axis_set_dimension_value( Axis* axis, Dimension index, double value )
{
  if( index >= N_DIMS )
  {
    fprintf( stderr, "PDOsetValue: invalid value index: %d\n", index );
    return;
  }

  axis->dimension_values[ index ] = value;
}
 
double axis_get_parameter( Axis* axis, Parameter index )
{
  if( index >= N_PARAMS )
  {
    fprintf( stderr, "GetParameterValue: invalid value index: %d\n", index );
    return 0;
  }
      
  return axis->control_values[ index ];
}

void axis_set_parameter( Axis* axis, Parameter index, double value )
{
  if( index >= N_PARAMS )
  {
    fprintf( stderr, "SetParameterValue: invalid value index: %d\n", index );
    return;
  }
      
  axis->control_values[ index ] = value;
}

void axis_set_output( Axis* axis, bool enabled )
{
  if( enabled ) axis->output = 0x2000;
  else axis->output = 0x0000;
}

bool axis_get_status_value( Axis* axis, Status index )
{
  if( (axis->status_word & index) > 0 ) return true;
  else return false;
}

void axis_set_control_value( Axis* axis, Control index, bool enabled )
{
  if( enabled ) axis->control_word |= index;
  else axis->control_word &= (~index);
}

void axis_read_values( Axis* axis )
{
  // Create reading buffer
  static u8 payload[8];

  // Read values from PDO01 (Position, Current and Status Word) to buffer
  can_frame_read( frame_list[ PDO01_RX ], payload );  
  // Update values from PDO01
  axis->dimension_values[ POSITION ] = payload[3] * 0x1000000 + payload[2] * 0x10000 + payload[1] * 0x100 + payload[0];
  axis->dimension_values[ CURRENT ] = payload[5] * 0x100 + payload[4];
  axis->status_word = payload[7] * 0x100 + payload[6];

  // Read values from PDO02 (Position, Current and Status Word) to buffer
  can_frame_read( frame_list[ PDO01_RX ], payload );  
  // Update values from PDO02
	axis->dimension_values[ VELOCITY ] = payload[3] * 0x1000000 + payload[2] * 0x10000 + payload[1] * 0x100 + payload[0];
	axis->dimension_values[ TENSION ] = payload[5] * 0x100 + payload[4];
}

void axis_write_values( Axis* axis )
{
  // Create writing buffer
  static u8 payload[8];

  // Set values for PDO01 (Position Setpoint and Control Word)
  payload[0] = ( (int) axis->dimension_values[ POSITION_SETPOINT ] & 0x000000ff );
  payload[1] = ( (int) axis->dimension_values[ POSITION_SETPOINT ] & 0x0000ff00 ) / 0x100;
  payload[2] = ( (int) axis->dimension_values[ POSITION_SETPOINT ] & 0x00ff0000 ) / 0x10000;
  payload[3] = ( (int) axis->dimension_values[ POSITION_SETPOINT ] & 0xff000000 ) / 0x1000000;
  payload[4] = ( 0 & 0x000000ff );
  payload[5] = ( 0 & 0x0000ff00 ) / 0x100; 
  payload[6] = ( axis->control_word & 0x000000ff );
  payload[7] = ( axis->control_word & 0x0000ff00 ) / 0x100; 

  // Write values from buffer to PDO01 
  can_frame_write( frame_list[ PDO01_TX ], payload );

  // Set values for PDO02 (Velocity Setpoint and Output)
  payload[0] = ( (int) axis->dimension_values[ VELOCITY_SETPOINT ] & 0x000000ff );
  payload[1] = ( (int) axis->dimension_values[ VELOCITY_SETPOINT ] & 0x0000ff00 ) / 0x100;
  payload[2] = ( (int) axis->dimension_values[ VELOCITY_SETPOINT ] & 0x00ff0000 ) / 0x10000;
  payload[3] = ( (int) axis->dimension_values[ VELOCITY_SETPOINT ] & 0xff000000 ) / 0x1000000;
  payload[4] = ( axis->output & 0x000000ff );
  payload[5] = ( axis->output & 0x0000ff00 ) / 0x100; 
  payload[6] = ( 0 & 0x000000ff );
  payload[7] = ( 0 & 0x0000ff00 ) / 0x100; 

  // Write values from buffer to PDO01 E
  can_frame_write( frame_list[ PDO02_TX ], payload );
}

double axis_read_single_value( Axis* axis, int index, u8 sub_index )
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
  can_frame_write( axis->frame_list[ SDO_TX ], payload );

  delay( 100 );

  // Read requested value from SDO frame
  can_frame_read( axis->frame_list[ SDO_RX ], payload );

  double value = payload[7] * 0x1000000 + payload[6] * 0x10000 + payload[5] * 0x100 + payload[4];

  return value; 
}

void axis_write_single_value( Axis* axis, int index, u8 sub_index, short int value )
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
  can_frame_write( axis->frame_list[ SDO_TX ], payload );
}

void axis_set_operation_mode( Axis* axis , OperationMode mode )
{
  axis_write_single_value( axis, 0x6060, 0x00, mode );
}

void axis_set_digital_output( Axis* axis, bool enabled )
{
  if( enabled ) axis_write_single_value( axis, 0x6060, 0x00, 0xff );
  else axis_write_single_value( axis, 0x6060, 0x00, 0x00 );
}


#endif	/* AXIS_H */

