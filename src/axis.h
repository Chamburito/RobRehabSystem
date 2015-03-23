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

#ifdef _CVI_
  #include <ansi_c.h>
#else
  #include <stdio.h>
  #include <string.h>
  #include <stdlib.h>
  #include <malloc.h>
#endif

#include <stdbool.h>

#include "async_debug.h"

enum OperationMode { HOMMING_MODE, PROFILE_VELOCITY_MODE, PROFILE_POSITION_MODE, POSITION_MODE, 
                     VELOCITY_MODE, CURRENT_MODE, MASTER_ENCODER_MODE, STEP_MODE, AXIS_N_MODES };
static const uint8_t operation_modes[ AXIS_N_MODES ] = { 0x06, 0x03, 0x01, 0xFF, 0xFE, 0xFD, 0xFB, 0xFA };

enum State { READY_2_SWITCH_ON, SWITCHED_ON, OPERATION_ENABLED, FAULT, VOLTAGE_ENABLED, 
              QUICK_STOPPED, SWITCH_ON_DISABLE, REMOTE_NMT, TARGET_REACHED, SETPOINT_ACK, AXIS_N_STATES };
static const uint16_t state_values[ AXIS_N_STATES ] = { 1, 2, 4, 8, 16, 32, 64, 512, 1024, 4096 };

enum Control { SWITCH_ON, ENABLE_VOLTAGE, QUICK_STOP, ENABLE_OPERATION, 
               NEW_SETPOINT, CHANGE_IMMEDIATEDLY, ABS_REL, FAULT_RESET, HALT, AXIS_N_CONTROLS };
static const uint16_t control_values[ AXIS_N_CONTROLS ] = { 1, 2, 4, 8, 16, 32, 64, 128, 256 };

enum Dimension { POSITION, VELOCITY, CURRENT, TENSION, AXIS_N_DIMS }; 

enum Parameter { POSITION_SETPOINT, VELOCITY_SETPOINT, PROPORTIONAL_GAIN, DERIVATIVE_GAIN, AXIS_N_PARAMS }; 

enum FrameType { SDO_TX, SDO_RX, PDO01_RX, PDO01_TX, PDO02_RX, PDO02_TX, AXIS_N_FRAMES };
static const char* frame_names[ AXIS_N_FRAMES ] = { "SDO_TX_0", "SDO_RX_0", "PDO01_RX_0", "PDO01_TX_0", "PDO02_RX_0", "PDO02_TX_0" };

const size_t DEVICE_NAME_MAX_LENGTH = 15;

typedef struct _Encoder
{
  CANFrame* frameList[ AXIS_N_FRAMES ];
  double measuresList[ AXIS_N_DIMS ];
  uint16_t statusWord, controlWord;
  unsigned int resolution;
  short int output;
}
Encoder;

typedef struct _Motor
{
  char name[ DEVICE_NAME_MAX_LENGTH ];
  Encoder* encoder;
  double parametersList[ AXIS_N_PARAMS ];
  bool active;
}
Motor;

Motor* Motor_Connect( const char*, unsigned int, unsigned int );
Encoder* Encoder_Connect( unsigned int, unsigned int );
void Motor_Enable( Motor* );
void Motor_Disable( Motor* );
void Motor_Disconnect( Motor* );
void Encoder_Disconnect( Encoder* );
double Encoder_GetMeasure( Encoder*, Dimension );
double Motor_GetParameter( Motor*, Parameter );
void Motor_SetParameter( Motor*, Parameter, double );
void Encoder_SetOutput( Encoder*, bool );
bool Encoder_CheckState( Encoder*, State );
void Encoder_Reset( Encoder* );
static void SetControl( Encoder*, Control, bool );
void Encoder_Read( Encoder* );
static void WriteConfig( Encoder* );
void Motor_WriteConfig( Motor* );
static double ReadSingleValue( Encoder*, u16, u8 );
static void WriteSingleValue( Encoder*, u16, u8, int );
void Motor_SetOperationMode( Motor*, OperationMode );
void Motor_SetDigitalOutput( Motor*, bool );

// Create CAN controlled DC motor handle
Motor* Motor_Connect( const char* device_name, unsigned int networkIndex, unsigned int resolution ) 
{
  DEBUG_EVENT( 0, "created motor %s with network index %u", device_name, networkIndex ); 
  
  Motor* motor = (Motor*) malloc( sizeof(Motor) );

  if( device_name != NULL ) strncpy( motor->name, device_name, DEVICE_NAME_MAX_LENGTH );
  
  for( int i = 0; i < AXIS_N_PARAMS; i++ )
    motor->parametersList[ i ] = 0.0;

  motor->encoder = Encoder_Connect( networkIndex, resolution );

  if( motor->encoder != NULL ) Motor_Disable( motor );
  else Motor_Disconnect( motor );

  DEBUG_EVENT( 0, "created motor %s with network index %u", motor->name, networkIndex );
  
  return motor;
}

Encoder* Encoder_Connect( unsigned int networkIndex, unsigned int resolution )
{
  DEBUG_EVENT( 0, "creating encoder with network index %u", networkIndex );
  
  Encoder* encoder = (Encoder*) malloc( sizeof(Encoder) );
  
  for( int i = 0; i < AXIS_N_DIMS; i++ )
    encoder->measuresList[ i ] = 10.0;

  encoder->controlWord = encoder->statusWord = 0;
  
  encoder->resolution = resolution;
  
  char networkAddress[ 20 ];

  for( int frameID = 0; frameID < AXIS_N_FRAMES; frameID += 2 )
  {
    sprintf( networkAddress, "%s%u", frame_names[ frameID ], networkIndex );
    DEBUG_EVENT( networkIndex + frameID, "creating frame %s", networkAddress );
    encoder->frameList[ frameID ] = EposNetwork_InitFrame( FRAME_OUT, "CAN2", networkAddress );

    if( encoder->frameList[ frameID ] == NULL ) Encoder_Disconnect( encoder );
    
    sprintf( networkAddress, "%s%u", frame_names[ frameID + 1 ], networkIndex );
    DEBUG_EVENT( networkIndex + frameID + 1, "creating frame %s", networkAddress );
    encoder->frameList[ frameID + 1 ] = EposNetwork_InitFrame( FRAME_IN, "CAN1", networkAddress );

    if( encoder->frameList[ frameID + 1 ] == NULL ) Encoder_Disconnect( encoder );
  }

  DEBUG_EVENT( 0, "created encoder with network index %u", networkIndex );
  
  return encoder;
}

void Motor_Disconnect( Motor* motor )
{
  if( motor != NULL )
  {
    if( motor->active) Motor_Disable( motor );
    
    Encoder_Disconnect( motor->encoder );

    free( motor );
    motor = NULL;
  }
}

void Encoder_Disconnect( Encoder* encoder )
{
  if( encoder != NULL )
  {
    for( size_t frameID = 0; frameID < AXIS_N_FRAMES; frameID++ )
      EposNetwork_EndFrame( encoder->frameList[ frameID ] );

    free( encoder );
    encoder = NULL;
  }
}

void Motor_Enable( Motor* motor )
{
  if( motor->active ) return;
  
  SetControl( motor->encoder, SWITCH_ON, false );
  SetControl( motor->encoder, ENABLE_VOLTAGE, true );
  SetControl( motor->encoder, QUICK_STOP, true );
  SetControl( motor->encoder, ENABLE_OPERATION, false );
  WriteConfig( motor->encoder );
        
  Timing_Delay( 500 );
        
  SetControl( motor->encoder, SWITCH_ON, true );
  SetControl( motor->encoder, ENABLE_VOLTAGE, true );
  SetControl( motor->encoder, QUICK_STOP, true );
  SetControl( motor->encoder, ENABLE_OPERATION, false );
  WriteConfig( motor->encoder );

  Timing_Delay( 500 );
        
  SetControl( motor->encoder, SWITCH_ON, true );
  SetControl( motor->encoder, ENABLE_VOLTAGE, true );
  SetControl( motor->encoder, QUICK_STOP, true );
  SetControl( motor->encoder, ENABLE_OPERATION, true );
  WriteConfig( motor->encoder );

  motor->active = true;
}

void Motor_Disable( Motor* motor )
{
  if( !(motor->active) ) return;
  
  SetControl( motor->encoder, SWITCH_ON, true );
  SetControl( motor->encoder, ENABLE_VOLTAGE, true );
  SetControl( motor->encoder, QUICK_STOP, true );
  SetControl( motor->encoder, ENABLE_OPERATION, false );
  WriteConfig( motor->encoder );
        
  Timing_Delay( 500 );
        
  SetControl( motor->encoder, SWITCH_ON, false );
  SetControl( motor->encoder, ENABLE_VOLTAGE, true );
  SetControl( motor->encoder, QUICK_STOP, true );
  SetControl( motor->encoder, ENABLE_OPERATION, false );
  WriteConfig( motor->encoder );

  for( size_t paramIndex = 0; paramIndex < AXIS_N_PARAMS; paramIndex++ )
    Motor_SetParameter( motor, (enum Parameter) paramIndex, 0 );
  Motor_WriteConfig( motor );

  motor->active = false;
}

double Encoder_GetMeasure( Encoder* encoder, enum Dimension index )
{
  if( index >= AXIS_N_DIMS )
  {
    fprintf( stderr, "Encoder_GetMeasure: invalid value index: %d\n", index );
    return 0;
  }
      
  return encoder->measuresList[ index ];
}
 
double Motor_GetParameter( Motor* motor, enum Parameter index )
{
  if( index >= AXIS_N_PARAMS )
  {
    fprintf( stderr, "Motor_GetParameter: invalid value index: %d\n", index );
    return 0;
  }
      
  return motor->parametersList[ index ];
}

void Motor_SetParameter( Motor* motor, enum Parameter index, double value )
{
  if( index >= AXIS_N_PARAMS )
  {
    fprintf( stderr, "Motor_SetParameter: invalid value index: %d\n", index );
    return;
  }
      
  motor->parametersList[ index ] = value;
}

void Encoder_SetOutput( Encoder* encoder, bool enabled )
{
  if( enabled ) encoder->output = 0x2000;
  else encoder->output = 0x0000;
}

bool Encoder_CheckState( Encoder* encoder, enum State index )
{
  if( index < 0 || index >= AXIS_N_STATES ) return false;
  
  uint16_t state_value = state_values[ index ];
  
  if( (encoder->statusWord & state_value) != 0 ) return true;
  else return false;
}

void Encoder_Reset( Encoder* encoder )
{
  SetControl( encoder, FAULT_RESET, true );
  WriteConfig( encoder );
  
  Timing_Delay( 500 );
  
  SetControl( encoder, FAULT_RESET, false );
  WriteConfig( encoder ); 
}

static inline void SetControl( Encoder* encoder, enum Control index, bool enabled )
{
  if( index < 0 || index >= AXIS_N_CONTROLS ) return;
  
  uint16_t control_value = control_values[ index ];
  
  if( enabled ) encoder->controlWord |= control_value;
  else encoder->controlWord &= (~control_value);
}

static void WriteConfig( Encoder* encoder )
{
  // Build writing buffer
  static u8 payload[8];
  payload[6] = (u8) ( encoder->controlWord & 0x000000ff );
  payload[7] = (u8) (( encoder->controlWord & 0x0000ff00 ) / 0x100);

  // Write values from buffer to PDO01 
  CANFrame_Write( encoder->frameList[ PDO01_TX ], payload );
}

void Encoder_Read( Encoder* encoder )
{
  // Create reading buffer
  static u8 payload[8];

  // Read values from PDO01 (Position, Current and Status Word) to buffer
  CANFrame_Read( encoder->frameList[ PDO01_RX ], payload );  
  // Update values from PDO01
  encoder->measuresList[ POSITION ] = payload[3] * 0x1000000 + payload[2] * 0x10000 + payload[1] * 0x100 + payload[0];
  encoder->measuresList[ POSITION ] = encoder->measuresList[ POSITION ] / encoder->resolution;
  encoder->measuresList[ CURRENT ] = payload[5] * 0x100 + payload[4];
  encoder->statusWord = payload[7] * 0x100 + payload[6];

  // Read values from PDO02 (Position, Current and Status Word) to buffer
  CANFrame_Read( encoder->frameList[ PDO02_RX ], payload );  
  // Update values from PDO02
  encoder->measuresList[ VELOCITY ] = payload[3] * 0x1000000 + payload[2] * 0x10000 + payload[1] * 0x100 + payload[0];
  encoder->measuresList[ TENSION ] = payload[5] * 0x100 + payload[4];
}

void Motor_WriteConfig( Motor* motor )
{
  // Create writing buffer
  static u8 payload[8];

  // Set values for PDO01 (Position Setpoint and Control Word)
  payload[0] = (u8) ( (int) motor->parametersList[ POSITION_SETPOINT ] & 0x000000ff );
  payload[1] = (u8) (( (int) motor->parametersList[ POSITION_SETPOINT ] & 0x0000ff00 ) / 0x100);
  payload[2] = (u8) (( (int) motor->parametersList[ POSITION_SETPOINT ] & 0x00ff0000 ) / 0x10000);
  payload[3] = (u8) (( (int) motor->parametersList[ POSITION_SETPOINT ] & 0xff000000 ) / 0x1000000);
  payload[4] = ( 0 & 0x000000ff );
  payload[5] = ( 0 & 0x0000ff00 ) / 0x100; 
  payload[6] = (u8) ( motor->encoder->controlWord & 0x000000ff );
  payload[7] = (u8) (( motor->encoder->controlWord & 0x0000ff00 ) / 0x100); 

  // Write values from buffer to PDO01 
  CANFrame_Write( motor->encoder->frameList[ PDO01_TX ], payload );

  // Set values for PDO02 (Velocity Setpoint and Output)
  payload[0] = (u8) ( (int) motor->parametersList[ VELOCITY_SETPOINT ] & 0x000000ff );
  payload[1] = (u8) (( (int) motor->parametersList[ VELOCITY_SETPOINT ] & 0x0000ff00 ) / 0x100);
  payload[2] = (u8) (( (int) motor->parametersList[ VELOCITY_SETPOINT ] & 0x00ff0000 ) / 0x10000);
  payload[3] = (u8) (( (int) motor->parametersList[ VELOCITY_SETPOINT ] & 0xff000000 ) / 0x1000000);
  payload[4] = (u8) ( motor->encoder->output & 0x000000ff );
  payload[5] = (u8) (( motor->encoder->output & 0x0000ff00 ) / 0x100); 
  payload[6] = ( 0 & 0x000000ff );
  payload[7] = ( 0 & 0x0000ff00 ) / 0x100; 

  // Write values from buffer to PDO01 E
  CANFrame_Write( motor->encoder->frameList[ PDO02_TX ], payload );
}

static double ReadSingleValue( Encoder* encoder, u16 index, u8 subIndex )
{
  // Build read requisition buffer for defined value
  static u8 payload[8];
  payload[0] = 0x40; 
  payload[1] = (u8) ( index & 0x000000ff );
  payload[2] = (u8) (( index & 0x0000ff00 ) / 0x100);//0xff); ?
  payload[3] = subIndex;
  payload[4] = 0x0;
  payload[5] = 0x0;
  payload[6] = 0x0;
  payload[7] = 0x0;

  // Write value requisition to SDO frame 
  CANFrame_Write( encoder->frameList[ SDO_TX ], payload );

  Timing_Delay( 100 );

  // Read requested value from SDO frame
  CANFrame_Read( encoder->frameList[ SDO_RX ], payload );

  double value = payload[7] * 0x1000000 + payload[6] * 0x10000 + payload[5] * 0x100 + payload[4];

  return value; 
}

static void WriteSingleValue( Encoder* encoder, u16 index, u8 subIndex, int value )
{
  // Build write buffer
  static u8 payload[8];
  payload[0] = 0x22; 
  payload[1] = (u8) ( index & 0x000000ff );
  payload[2] = (u8) (( index & 0x0000ff00 ) / 0x100);
  payload[3] = subIndex;
  payload[4] = (u8) ( value & 0x000000ff );
  payload[5] = (u8) (( value & 0x0000ff00 ) / 0x100);
  payload[6] = (u8) (( value & 0x00ff0000 ) / 0x10000);
  payload[7] = (u8) (( value & 0xff000000 ) / 0x1000000);

  // Write value to SDO frame 
  CANFrame_Write( encoder->frameList[ SDO_TX ], payload );
}

extern inline void Motor_SetOperationMode( Motor* motor, enum OperationMode mode )
{
  WriteSingleValue( motor->encoder, 0x6060, 0x00, mode );
}

extern inline void encoder_set_digital_output( Encoder* encoder, bool enabled )
{
  if( enabled ) WriteSingleValue( encoder, 0x6060, 0x00, 0xff );
  else WriteSingleValue( encoder, 0x6060, 0x00, 0x00 );
}


#endif	/* AXIS_H */

