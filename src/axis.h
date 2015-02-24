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
#include <stdbool.h>

enum OperationMode { HOMMING_MODE = 0x06, PROFILE_VELOCITY_MODE = 0x03, PROFILE_POSITION_MODE = 0x01,
	POSITION_MODE = 0xFF, VELOCITY_MODE = 0xFE, CURRENT_MODE = 0xFD, MASTER_ENCODER_MODE = 0xFB, STEP_MODE = 0xFA };

enum Status { READY_2_SWITCH_ON = 1, SWITCHED_ON = 2, OPERATION_ENABLED = 4, FAULT = 8, VOLTAGE_ENABLED = 16,
	QUICK_STOPPED = 32, SWITCH_ON_DISABLE = 64, REMOTE_NMT = 512, TARGET_REACHED = 1024, SETPOINT_ACK = 4096 };

enum Control { SWITCH_ON = 1, ENABLE_VOLTAGE = 2, QUICK_STOP = 4, ENABLE_OPERATION = 8, NEW_SETPOINT = 16,
	CHANGE_IMMEDIATEDLY = 32, ABS_REL = 64, FAULT_RESET = 128, HALT = 256 };

enum Dimension { POSITION, VELOCITY, CURRENT, TENSION, ANGLE, FORCE, N_DIMS }; 

enum Parameter { POSITION_SETPOINT, VELOCITY_SETPOINT, CURRENT_SETPOINT, PROPORTIONAL_GAIN, DERIVATIVE_GAIN, N_PARAMS }; 

enum FrameType { SDO_TX, SDO_RX, PDO01_RX, PDO01_TX, PDO02_RX, PDO02_TX, N_FRAMES };
static const char* frame_names[ N_FRAMES ] = { "SDO_TX_0", "SDO_RX_0", "PDO01_RX_0", "PDO01_TX_0", "PDO02_RX_0", "PDO02_TX_0" };

typedef struct _Encoder
{
  CAN_Frame* frame_list[ N_FRAMES ];
  double measures[ N_DIMS ];
  short unsigned int status_word, control_word;
  unsigned int resolution;
  short int output;
}
Encoder;

typedef struct _Motor
{
  Encoder* encoder;
  double parameters[ N_PARAMS ];
  bool active;
}
Motor;

Motor* motor_connect( unsigned int, unsigned int );
Encoder* encoder_connect( unsigned int, unsigned int );
void motor_enable( Motor* );
void motor_disable( Motor* );
void motor_disconnect( Motor* );
void encoder_disconnect( Encoder* );
double encoder_get_measure( Encoder*, Dimension );
double motor_get_parameter( Motor*, Parameter );
void motor_set_parameter( Motor*, Parameter, double );
void encoder_set_output( Encoder*, int );
bool encoder_get_status( Encoder*, Status );
void encoder_reset( Encoder* );
static void encoder_set_control( Encoder*, Control, bool );
void encoder_read( Encoder* );
static void encoder_config_write( Encoder* );
void motor_control_write( Motor* );
static double encoder_read_single_value( Encoder*, int, u8 );
static void encoder_config_single_value( Encoder*, int, u8, short int );
void motor_set_operation_mode( Motor*, OperationMode );
void motor_set_digital_output( Motor*, bool );

// Create CAN controlled DC motor handle
Motor* motor_connect( unsigned int network_index, unsigned int resolution ) 
{
  Motor* motor = (Motor*) malloc( sizeof(Motor) );

  for( int i = 0; i < N_PARAMS; i++ )
    motor->parameters[ i ] = 0.0;

  motor->encoder = encoder_connect( network_index, resolution );

  if( motor->encoder != NULL ) motor_disable( motor );
  else motor_disconnect( motor );

  return motor;
}

Encoder* encoder_connect( unsigned int network_index, unsigned int resolution )
{
  Encoder* encoder = (Encoder*) malloc( sizeof(Encoder) );

  for( int i = 0; i < N_DIMS; i++ )
    encoder->measures[ i ] = 10.0;

  encoder->control_word = encoder->status_word = 0;
  
  encoder->resolution = resolution;
  
  char network_address[ 20 ];

  for( int frame_id = 0; frame_id < N_FRAMES; frame_id += 2 )
  {
    sprintf( network_address, "%s%u", frame_names[ frame_id ], network_index );
    printf( "motor_init: creating frame %s\n", network_address );
    encoder->frame_list[ frame_id ] = epos_network_init_frame( FRAME_OUT, "CAN2", network_address );

    if( encoder->frame_list[ frame_id ] == NULL ) encoder_disconnect( encoder );
    
    sprintf( network_address, "%s%u", frame_names[ frame_id + 1 ], network_index );
    printf( "motor_init: creating frame %s\n", network_address );
    encoder->frame_list[ frame_id + 1 ] = epos_network_init_frame( FRAME_IN, "CAN1", network_address );

    if( encoder->frame_list[ frame_id + 1 ] == NULL ) encoder_disconnect( encoder );
  }

  return encoder;
}

void motor_disconnect( Motor* motor )
{
  if( motor != NULL )
  {
    if( motor->active) motor_disable( motor );
    
    encoder_disconnect( motor->encoder );

    free( motor );
    motor = NULL;
  }
}

void encoder_disconnect( Encoder* encoder )
{
  if( encoder != NULL )
  {
    for( size_t frame_id = 0; frame_id < N_FRAMES; frame_id++ )
      can_frame_end( encoder->frame_list[ frame_id ] );

    free( encoder );
    encoder = NULL;
  }
}

void motor_enable( Motor* motor )
{
  encoder_set_control( motor->encoder, SWITCH_ON, false );
  encoder_set_control( motor->encoder, ENABLE_VOLTAGE, true );
  encoder_set_control( motor->encoder, QUICK_STOP, true );
  encoder_set_control( motor->encoder, ENABLE_OPERATION, false );
  encoder_config_write( motor->encoder );
        
  delay( 500 );
        
  encoder_set_control( motor->encoder, SWITCH_ON, true );
  encoder_set_control( motor->encoder, ENABLE_VOLTAGE, true );
  encoder_set_control( motor->encoder, QUICK_STOP, true );
  encoder_set_control( motor->encoder, ENABLE_OPERATION, false );
  encoder_config_write( motor->encoder );

  delay( 500 );
        
  encoder_set_control( motor->encoder, SWITCH_ON, true );
  encoder_set_control( motor->encoder, ENABLE_VOLTAGE, true );
  encoder_set_control( motor->encoder, QUICK_STOP, true );
  encoder_set_control( motor->encoder, ENABLE_OPERATION, true );
  encoder_config_write( motor->encoder );

  motor->active = true;
}

void motor_disable( Motor* motor )
{
  encoder_set_control( motor->encoder, SWITCH_ON, true );
  encoder_set_control( motor->encoder, ENABLE_VOLTAGE, true );
  encoder_set_control( motor->encoder, QUICK_STOP, true );
  encoder_set_control( motor->encoder, ENABLE_OPERATION, false );
  encoder_config_write( motor->encoder );
        
  delay( 500 );
        
  encoder_set_control( motor->encoder, SWITCH_ON, false );
  encoder_set_control( motor->encoder, ENABLE_VOLTAGE, true );
  encoder_set_control( motor->encoder, QUICK_STOP, true );
  encoder_set_control( motor->encoder, ENABLE_OPERATION, false );
  encoder_config_write( motor->encoder );

  for( size_t param_index; param_index < N_PARAMS; param_index++ )
    motor_set_parameter( motor, (Parameter) param_index, 0 );
  motor_control_write( motor );

  motor->active = false;
}

double encoder_get_measure( Encoder* encoder, Dimension index )
{
  if( index >= N_DIMS )
  {
    fprintf( stderr, "encoder_get_measure: invalid value index: %d\n", index );
    return 0;
  }
      
  return encoder->measures[ index ];
}
 
double motor_get_parameter( Motor* motor, Parameter index )
{
  if( index >= N_PARAMS )
  {
    fprintf( stderr, "motor_get_parameter: invalid value index: %d\n", index );
    return 0;
  }
      
  return motor->parameters[ index ];
}

void motor_set_parameter( Motor* motor, Parameter index, double value )
{
  if( index >= N_PARAMS )
  {
    fprintf( stderr, "motor_set_parameter: invalid value index: %d\n", index );
    return;
  }
      
  motor->parameters[ index ] = value;
}

void encoder_set_output( Encoder* encoder, bool enabled )
{
  if( enabled ) encoder->output = 0x2000;
  else encoder->output = 0x0000;
}

bool encoder_get_status( Encoder* encoder, Status index )
{
  if( (encoder->status_word & index) > 0 ) return true;
  else return false;
}

void encoder_reset( Encoder* encoder )
{
  encoder_set_control( encoder, FAULT_RESET, true );
  encoder_configure( encoder );
  
  delay( 500 );
  
  encoder_set_control( encoder, FAULT_RESET, false );
  encoder_configure( encoder ); 
}

static inline void encoder_set_control( Encoder* encoder, Control index, bool enabled )
{
  if( enabled ) encoder->control_word |= index;
  else encoder->control_word &= (~index);
}

static void encoder_config_write( Encoder* encoder )
{
  // Build writing buffer
  static u8 payload[8];
  payload[6] = ( encoder->control_word & 0x000000ff );
  payload[7] = ( encoder->control_word & 0x0000ff00 ) / 0x100;

  // Write values from buffer to PDO01 
  can_frame_write( encoder->frame_list[ PDO01_TX ], payload );
}

void encoder_read( Encoder* encoder )
{
  // Create reading buffer
  static u8 payload[8];

  // Read values from PDO01 (Position, Current and Status Word) to buffer
  can_frame_read( frame_list[ PDO01_RX ], payload );  
  // Update values from PDO01
  encoder->measures[ POSITION ] = payload[3] * 0x1000000 + payload[2] * 0x10000 + payload[1] * 0x100 + payload[0];
  encoder->measures[ ANGLE ] = ( encoder->measures[ POSITION ] / encoder->resolution ) * 360.0;
  encoder->measures[ CURRENT ] = payload[5] * 0x100 + payload[4];
  encoder->status_word = payload[7] * 0x100 + payload[6];

  // Read values from PDO02 (Position, Current and Status Word) to buffer
  can_frame_read( frame_list[ PDO02_RX ], payload );  
  // Update values from PDO02
  encoder->measures[ VELOCITY ] = payload[3] * 0x1000000 + payload[2] * 0x10000 + payload[1] * 0x100 + payload[0];
  encoder->measures[ TENSION ] = payload[5] * 0x100 + payload[4];
}

void motor_control_write( Motor* motor )
{
  // Create writing buffer
  static u8 payload[8];

  // Set values for PDO01 (Position Setpoint and Control Word)
  payload[0] = ( (int) motor->parameters[ POSITION_SETPOINT ] & 0x000000ff );
  payload[1] = ( (int) motor->parameters[ POSITION_SETPOINT ] & 0x0000ff00 ) / 0x100;
  payload[2] = ( (int) motor->parameters[ POSITION_SETPOINT ] & 0x00ff0000 ) / 0x10000;
  payload[3] = ( (int) motor->parameters[ POSITION_SETPOINT ] & 0xff000000 ) / 0x1000000;
  payload[4] = ( 0 & 0x000000ff );
  payload[5] = ( 0 & 0x0000ff00 ) / 0x100; 
  payload[6] = ( motor->encoder->control_word & 0x000000ff );
  payload[7] = ( motor->encoder->control_word & 0x0000ff00 ) / 0x100; 

  // Write values from buffer to PDO01 
  can_frame_write( motor->encoder->frame_list[ PDO01_TX ], payload );

  // Set values for PDO02 (Velocity Setpoint and Output)
  payload[0] = ( (int) motor->parameters[ VELOCITY_SETPOINT ] & 0x000000ff );
  payload[1] = ( (int) motor->parameters[ VELOCITY_SETPOINT ] & 0x0000ff00 ) / 0x100;
  payload[2] = ( (int) motor->parameters[ VELOCITY_SETPOINT ] & 0x00ff0000 ) / 0x10000;
  payload[3] = ( (int) motor->parameters[ VELOCITY_SETPOINT ] & 0xff000000 ) / 0x1000000;
  payload[4] = ( motor->encoder->output & 0x000000ff );
  payload[5] = ( motor->encoder->output & 0x0000ff00 ) / 0x100; 
  payload[6] = ( 0 & 0x000000ff );
  payload[7] = ( 0 & 0x0000ff00 ) / 0x100; 

  // Write values from buffer to PDO01 E
  can_frame_write( motor->encoder->frame_list[ PDO02_TX ], payload );
}

static double encoder_read_single_value( Encoder* encoder, int index, u8 sub_index )
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
  can_frame_write( encoder->frame_list[ SDO_TX ], payload );

  delay( 100 );

  // Read requested value from SDO frame
  can_frame_read( encoder->frame_list[ SDO_RX ], payload );

  double value = payload[7] * 0x1000000 + payload[6] * 0x10000 + payload[5] * 0x100 + payload[4];

  return value; 
}

static void encoder_config_single_value( Encoder* encoder, int index, u8 sub_index, short int value )
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
  can_frame_write( encoder->frame_list[ SDO_TX ], payload );
}

extern inline void motor_set_operation_mode( Motor* motor, OperationMode mode )
{
  encoder_config_single_value( motor->encoder, 0x6060, 0x00, mode );
}

extern inline void encoder_set_digital_output( Encoder* encoder, bool enabled )
{
  if( enabled ) encoder_config_single_value( encoder, 0x6060, 0x00, 0xff );
  else encoder_config_single_value( encoder, 0x6060, 0x00, 0x00 );
}


#endif	/* AXIS_H */

