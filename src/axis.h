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
static const uint8_t operationModes[ AXIS_N_MODES ] = { 0x06, 0x03, 0x01, 0xFF, 0xFE, 0xFD, 0xFB, 0xFA };

enum State { READY_2_SWITCH_ON, SWITCHED_ON, OPERATION_ENABLED, FAULT, VOLTAGE_ENABLED, 
              QUICK_STOPPED, SWITCH_ON_DISABLE, REMOTE_NMT, TARGET_REACHED, SETPOINT_ACK, AXIS_N_STATES };
static const uint16_t stateValues[ AXIS_N_STATES ] = { 1, 2, 4, 8, 16, 32, 64, 512, 1024, 4096 };

enum Control { SWITCH_ON, ENABLE_VOLTAGE, QUICK_STOP, ENABLE_OPERATION, 
               NEW_SETPOINT, CHANGE_IMMEDIATEDLY, ABS_REL, FAULT_RESET, HALT, AXIS_N_CONTROLS };
static const uint16_t controlValues[ AXIS_N_CONTROLS ] = { 1, 2, 4, 8, 16, 32, 64, 128, 256 };

enum Dimension { POSITION, VELOCITY, CURRENT, TENSION, AXIS_N_DIMS }; 

enum Parameter { POSITION_SETPOINT, VELOCITY_SETPOINT, PROPORTIONAL_GAIN, DERIVATIVE_GAIN, AXIS_N_PARAMS }; 

enum FrameType { SDO, PDO01, PDO02, AXIS_N_FRAMES };
static const char* CAN_FRAME_NAMES[ AXIS_N_FRAMES ] = { "SDO", "PDO01", "PDO02" };

const size_t DEVICE_NAME_MAX_LENGTH = 15;

typedef struct _Encoder
{
  CANFrame* readFramesList[ AXIS_N_FRAMES ];
  double measuresList[ AXIS_N_DIMS ];
  unsigned int resolution;
  uint16_t statusWord
}
Encoder;

typedef struct _Motor
{
  char name[ DEVICE_NAME_MAX_LENGTH ];
  Encoder* encoder;
  CANFrame* writeFramesList[ AXIS_N_FRAMES ];
  double parametersList[ AXIS_N_PARAMS ];
  uint16_t controlWord;
  uint16_t digitalOutput;
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
bool Encoder_CheckState( Encoder*, State );
void Motor_Reset( Motor* );
static void SetControl( Motor*, Control, bool );
void Encoder_ReadValues( Encoder* );
void Motor_WriteConfig( Motor* );
static double ReadSingleValue( Motor*, uint16_t, uint8_t );
static void WriteSingleValue( Motor*, uint16_t, uint8_t, int );
void Motor_SetOperationMode( Motor*, OperationMode );
static void EnableDigitalOutput( Motor*, bool );
void Motor_SetDigitalOutput( Motor*, uint16_t output );

// Create CAN controlled DC motor handle
Motor* Motor_Connect( const char* device_name, unsigned int networkIndex, unsigned int resolution ) 
{
  DEBUG_EVENT( 0, "created motor %s with network index %u", device_name, networkIndex ); 
  
  Motor* motor = (Motor*) malloc( sizeof(Motor) );

  if( device_name != NULL ) strncpy( motor->name, device_name, DEVICE_NAME_MAX_LENGTH );
  
  for( int i = 0; i < AXIS_N_PARAMS; i++ )
    motor->parametersList[ i ] = 0.0;
  
  motor->controlWord = 0;
  
  static char networkAddress[ DEVICE_NAME_MAX_LENGTH ];

  for( int frameID = 0; frameID < AXIS_N_FRAMES; frameID++ )
  {
    sprintf( networkAddress, "%s_TX_%02u", CAN_FRAME_NAMES[ frameID ], networkIndex );
    motor->writeFramesList[ frameID ] = EposNetwork_InitFrame( FRAME_OUT, "CAN2", networkAddress );

    if( motor->writeFramesList[ frameID ] == NULL ) 
    {
      Motor_Disconnect( motor );
      ERROR_EVENT( "failed creating write frame %s for motor %s with network index %u", networkAddress, motor->name, networkIndex );
      return NULL;
    }
    
    DEBUG_EVENT( 1, "created frame %s", networkAddress );
  }

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
  
  encoder->resolution = resolution;
  
  encoder->statusWord = 0;
  
  static char networkAddress[ DEVICE_NAME_MAX_LENGTH ];

  for( int frameID = 0; frameID < AXIS_N_FRAMES; frameID++ )
  {
    sprintf( networkAddress, "%s_RX_%02u", CAN_FRAME_NAMES[ frameID ], networkIndex );
    encoder->readFramesList[ frameID ] = EposNetwork_InitFrame( FRAME_IN, "CAN1", networkAddress );

    if( encoder->readFramesList[ frameID ] == NULL ) 
    {
      Encoder_Disconnect( encoder );
      ERROR_EVENT( "failed creating read frame %s for encoder with network index %u", networkAddress, networkIndex );
      return NULL;
    }
    
    DEBUG_EVENT( 1, "created frame %s", networkAddress );
  }

  DEBUG_EVENT( 0, "created encoder with network index %u", networkIndex );
  
  return encoder;
}

void Motor_Disconnect( Motor* motor )
{
  if( motor != NULL )
  {
    if( motor->active ) Motor_Disable( motor );
    
    Encoder_Disconnect( motor->encoder );
    motor->encoder = NULL;

    free( motor );
    motor = NULL;
  }
}

void Encoder_Disconnect( Encoder* encoder )
{
  if( encoder != NULL )
  {
    for( size_t frameID = 0; frameID < AXIS_N_FRAMES; frameID++ )
      EposNetwork_EndFrame( encoder->readFramesList[ frameID ] );

    free( encoder );
    encoder = NULL;
  }
}

void Motor_Enable( Motor* motor )
{
  if( motor->active ) return;
  
  SetControl( motor, SWITCH_ON, false );
  SetControl( motor, ENABLE_VOLTAGE, true );
  SetControl( motor, QUICK_STOP, true );
  SetControl( motor, ENABLE_OPERATION, false );
  Motor_WriteConfig( motor );
        
  Timing_Delay( 500 );
        
  SetControl( motor, SWITCH_ON, true );
  SetControl( motor, ENABLE_VOLTAGE, true );
  SetControl( motor, QUICK_STOP, true );
  SetControl( motor, ENABLE_OPERATION, false );
  Motor_WriteConfig( motor );

  Timing_Delay( 500 );
        
  SetControl( motor, SWITCH_ON, true );
  SetControl( motor, ENABLE_VOLTAGE, true );
  SetControl( motor, QUICK_STOP, true );
  SetControl( motor, ENABLE_OPERATION, true );
  Motor_WriteConfig( motor );

  motor->active = true;
}

void Motor_Disable( Motor* motor )
{
  if( !(motor->active) ) return;
  
  SetControl( motor, SWITCH_ON, true );
  SetControl( motor, ENABLE_VOLTAGE, true );
  SetControl( motor, QUICK_STOP, true );
  SetControl( motor, ENABLE_OPERATION, false );
  Motor_WriteConfig( motor );
        
  Timing_Delay( 500 );
        
  SetControl( motor, SWITCH_ON, false );
  SetControl( motor, ENABLE_VOLTAGE, true );
  SetControl( motor, QUICK_STOP, true );
  SetControl( motor, ENABLE_OPERATION, false );
  Motor_WriteConfig( motor );

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

bool Encoder_CheckState( Encoder* encoder, enum State index )
{
  if( index < 0 || index >= AXIS_N_STATES ) return false;
  
  uint16_t stateValue = stateValues[ index ];
  
  if( (encoder->statusWord & stateValue) != 0 ) return true;
  else return false;
}

void Motor_Reset( Motor* motor )
{
  SetControl( motor, FAULT_RESET, true );
  Motor_WriteConfig( motor );
  
  //DEBUG_EVENT( "reseting motor %p: control word: %x", motor, motor->controlWord );
  
  Timing_Delay( 500 );
  
  SetControl( motor, FAULT_RESET, false );
  Motor_WriteConfig( motor );
  
  //DEBUG_EVENT( "reseting motor %p: control word: %x", motor, motor->controlWord );
}

static inline void SetControl( Motor* motor, enum Control index, bool enabled )
{
  if( index < 0 || index >= AXIS_N_CONTROLS ) return;
  
  uint16_t controlValue = controlValues[ index ]; //0x00000001 << index;
  
  if( enabled ) motor->controlWord |= controlValue;
  else motor->controlWord &= (~controlValue);
}

void Encoder_ReadValues( Encoder* encoder )
{
  // Create reading buffer
  static uint8_t payload[8];

  EposNetwork_Sync();
  
  // Read values from PDO01 (Position, Current and Status Word) to buffer
  CANFrame_Read( encoder->readFramesList[ PDO01 ], payload );  
  // Update values from PDO01
  double rawPosition = payload[3] * 0x1000000 + payload[2] * 0x10000 + payload[1] * 0x100 + payload[0];
  encoder->measuresList[ POSITION ] = rawPosition / encoder->resolution;
  encoder->measuresList[ CURRENT ] = payload[5] * 0x100 + payload[4];
  encoder->statusWord = payload[7] * 0x100 + payload[6];

  // Read values from PDO02 (Position, Current and Status Word) to buffer
  CANFrame_Read( encoder->readFramesList[ PDO02 ], payload );  
  // Update values from PDO02
  encoder->measuresList[ VELOCITY ] = payload[3] * 0x1000000 + payload[2] * 0x10000 + payload[1] * 0x100 + payload[0];
  encoder->measuresList[ TENSION ] = payload[5] * 0x100 + payload[4];
}

void Motor_WriteConfig( Motor* motor )
{
  // Create writing buffer
  static uint8_t payload[8];

  int rawPositionSetpoint = (int) ( motor->parametersList[ POSITION_SETPOINT ] * motor->encoder->resolution );
  
  // Set values for PDO01 (Position Setpoint and Control Word)
  payload[0] = (uint8_t) ( rawPositionSetpoint & 0x000000ff );
  payload[1] = (uint8_t) (( rawPositionSetpoint & 0x0000ff00 ) / 0x100);
  payload[2] = (uint8_t) (( rawPositionSetpoint & 0x00ff0000 ) / 0x10000);
  payload[3] = (uint8_t) (( rawPositionSetpoint & 0xff000000 ) / 0x1000000);
  payload[4] = ( 0 & 0x000000ff );
  payload[5] = ( 0 & 0x0000ff00 ) / 0x100; 
  payload[6] = (uint8_t) ( motor->controlWord & 0x000000ff );
  payload[7] = (uint8_t) (( motor->controlWord & 0x0000ff00 ) / 0x100); 

  // Write values from buffer to PDO01 
  CANFrame_Write( motor->writeFramesList[ PDO01 ], payload );

  // Set values for PDO02 (Velocity Setpoint and Output)
  payload[0] = (uint8_t) ( (int) motor->parametersList[ VELOCITY_SETPOINT ] & 0x000000ff );
  payload[1] = (uint8_t) (( (int) motor->parametersList[ VELOCITY_SETPOINT ] & 0x0000ff00 ) / 0x100);
  payload[2] = (uint8_t) (( (int) motor->parametersList[ VELOCITY_SETPOINT ] & 0x00ff0000 ) / 0x10000);
  payload[3] = (uint8_t) (( (int) motor->parametersList[ VELOCITY_SETPOINT ] & 0xff000000 ) / 0x1000000);
  payload[4] = (uint8_t) ( motor->digitalOutput & 0x000000ff );
  payload[5] = (uint8_t) (( motor->digitalOutput & 0x0000ff00 ) / 0x100); 
  payload[6] = ( 0 & 0x000000ff );
  payload[7] = ( 0 & 0x0000ff00 ) / 0x100; 

  // Write values from buffer to PDO01
  CANFrame_Write( motor->writeFramesList[ PDO02 ], payload );
  
  EposNetwork_Sync();
}

static double ReadSingleValue( Motor* motor, uint16_t index, uint8_t subIndex )
{
  // Build read requisition buffer for defined value
  static uint8_t payload[8];
  payload[0] = 0x40; 
  payload[1] = (uint8_t) ( index & 0x000000ff );
  payload[2] = (uint8_t) (( index & 0x0000ff00 ) / 0x100);//0xff); ?
  payload[3] = subIndex;
  payload[4] = 0x0;
  payload[5] = 0x0;
  payload[6] = 0x0;
  payload[7] = 0x0;

  // Write value requisition to SDO frame 
  CANFrame_Write( motor->writeFramesList[ SDO ], payload );

  Timing_Delay( 100 );

  // Read requested value from SDO frame
  CANFrame_Read( motor->encoder->readFramesList[ SDO ], payload );

  double value = payload[7] * 0x1000000 + payload[6] * 0x10000 + payload[5] * 0x100 + payload[4];

  return value; 
}

static void WriteSingleValue( Motor* motor, uint16_t index, uint8_t subIndex, int value )
{
  // Build write buffer
  static uint8_t payload[8];
  payload[0] = 0x22; 
  payload[1] = (uint8_t) ( index & 0x000000ff );
  payload[2] = (uint8_t) (( index & 0x0000ff00 ) / 0x100);
  payload[3] = subIndex;
  payload[4] = (uint8_t) ( value & 0x000000ff );
  payload[5] = (uint8_t) (( value & 0x0000ff00 ) / 0x100);
  payload[6] = (uint8_t) (( value & 0x00ff0000 ) / 0x10000);
  payload[7] = (uint8_t) (( value & 0xff000000 ) / 0x1000000);

  // Write value to SDO frame 
  CANFrame_Write( motor->writeFramesList[ SDO ], payload );
}

extern inline void Motor_SetOperationMode( Motor* motor, enum OperationMode mode )
{
  WriteSingleValue( motor, 0x6060, 0x00, mode );
}

static void EnableDigitalOutput( Motor* motor, bool enabled )
{
  if( enabled ) WriteSingleValue( motor, 0x6060, 0x00, 0xff );
  else WriteSingleValue( motor, 0x6060, 0x00, 0x00 );
}

extern inline void Motor_SetDigitalOutput( Motor* motor, uint16_t output )
{
  motor->digitalOutput = output;
}


#endif	/* AXIS_H */

