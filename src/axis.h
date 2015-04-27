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
                     VELOCITY_MODE, CURRENT_MODE, MASTER_ENCODER_MODE, STEP_MODE, AXIS_MODES_NUMBER };
static const uint8_t operationModes[ AXIS_MODES_NUMBER ] = { 0x06, 0x03, 0x01, 0xFF, 0xFE, 0xFD, 0xFB, 0xFA };

enum State { READY_2_SWITCH_ON, SWITCHED_ON, OPERATION_ENABLED, FAULT, VOLTAGE_ENABLED, 
              QUICK_STOPPED, SWITCH_ON_DISABLE, REMOTE_NMT, TARGET_REACHED, SETPOINT_ACK, AXIS_STATES_NUMBER };
static const uint16_t stateValues[ AXIS_STATES_NUMBER ] = { 1, 2, 4, 8, 16, 32, 64, 512, 1024, 4096 };

enum Control { SWITCH_ON, ENABLE_VOLTAGE, QUICK_STOP, ENABLE_OPERATION, 
               NEW_SETPOINT, CHANGE_IMMEDIATEDLY, ABS_REL, FAULT_RESET, HALT, AXIS_CONTROLS_NUMBER };
static const uint16_t controlValues[ AXIS_CONTROLS_NUMBER ] = { 1, 2, 4, 8, 16, 32, 64, 128, 256 };

enum Dimension { POSITION, VELOCITY, TORQUE, TENSION, AXIS_DIMS_NUMBER }; 

enum Setpoint { POSITION_SETPOINT, VELOCITY_SETPOINT, AXIS_SETPTS_NUMBER }; 

enum FrameType { SDO, PDO01, PDO02, AXIS_FRAME_TYPES_NUMBER };
static const char* CAN_FRAME_NAMES[ AXIS_FRAME_TYPES_NUMBER ] = { "SDO", "PDO01", "PDO02" };

typedef struct _MotorDrive
{
  CANFrame* readFramesList[ AXIS_FRAME_TYPES_NUMBER ];
  CANFrame* writeFramesList[ AXIS_FRAME_TYPES_NUMBER ];
  double measuresList[ AXIS_DIMS_NUMBER ];
  unsigned int encoderResolution;
  double currentToTorqueRatio;
  uint16_t statusWord, controlWord;
  uint16_t digitalOutput;
}
MotorDrive;

typedef struct _Motor
{
  MotorDrive* controller;
  double setpointsList[ AXIS_SETPTS_NUMBER ];
  bool active;
}
Motor;

Motor* Motor_Connect( unsigned int );
MotorDrive* MotorDrive_Connect( unsigned int );
void Motor_Enable( Motor* );
void Motor_Disable( Motor* );
void Motor_Disconnect( Motor* );
void MotorDrive_Disconnect( MotorDrive* );
extern inline void MotorDrive_SetEncoderResolution( MotorDrive*, unsigned int );
extern inline void MotorDrive_SetTorqueConstant( MotorDrive*, double );
double MotorDrive_GetMeasure( MotorDrive*, enum Dimension );
double Motor_GetSetpoint( Motor*, enum Setpoint );
void Motor_SetSetpoint( Motor*, enum Setpoint, double );
bool MotorDrive_CheckState( MotorDrive*, enum State );
void MotorDrive_Reset( MotorDrive* );
static void SetControl( MotorDrive*, enum Control, bool );
void MotorDrive_ReadValues( MotorDrive* );
void Motor_WriteConfig( Motor* );
static void MotorDrive_WriteConfig( MotorDrive* );
static double ReadSingleValue( MotorDrive*, uint16_t, uint8_t );
static void WriteSingleValue( MotorDrive*, uint16_t, uint8_t, int );
void Motor_SetOperationMode( Motor*, enum OperationMode );
static inline void EnableDigitalOutput( MotorDrive*, bool );
extern inline void MotorDrive_SetDigitalOutput( MotorDrive*, uint16_t output );

const size_t DEVICE_NAME_MAX_LENGTH = 16;

// Create CAN controlled DC motor handle
Motor* Motor_Connect( unsigned int networkIndex ) 
{
  DEBUG_EVENT( 0, "created motor with network index %u", networkIndex ); 
  
  Motor* motor = (Motor*) malloc( sizeof(Motor) );
  
  for( int i = 0; i < AXIS_SETPTS_NUMBER; i++ )
    motor->setpointsList[ i ] = 0.0;
  
  if( (motor->controller = MotorDrive_Connect( networkIndex )) == NULL )
  {
    Motor_Disconnect( motor );
    return NULL;
  }

  SetControl( motor->controller, ENABLE_VOLTAGE, true );
  SetControl( motor->controller, QUICK_STOP, true );
  
  Motor_Disable( motor );

  DEBUG_EVENT( 0, "created motor with network index %u", networkIndex );
  
  return motor;
}

MotorDrive* MotorDrive_Connect( unsigned int networkIndex )
{
  DEBUG_EVENT( 0, "creating controller with network index %u", networkIndex );
  
  MotorDrive* controller = (MotorDrive*) malloc( sizeof(MotorDrive) );
  
  for( int i = 0; i < AXIS_DIMS_NUMBER; i++ )
    controller->measuresList[ i ] = 10.0;
  
  controller->encoderResolution = 1;
  controller->currentToTorqueRatio = 0.0;
  
  controller->statusWord = controller->controlWord = 0;
  
  controller->digitalOutput = 0;
  
  static char networkAddress[ DEVICE_NAME_MAX_LENGTH ];

  for( int frameID = 0; frameID < AXIS_FRAME_TYPES_NUMBER; frameID++ )
  {
    sprintf( networkAddress, "%s_RX_%02u", CAN_FRAME_NAMES[ frameID ], networkIndex );
    controller->readFramesList[ frameID ] = EposNetwork_InitFrame( FRAME_IN, "CAN1", networkAddress );

    if( controller->readFramesList[ frameID ] != NULL ) 
    {
      sprintf( networkAddress, "%s_TX_%02u", CAN_FRAME_NAMES[ frameID ], networkIndex );
      controller->writeFramesList[ frameID ] = EposNetwork_InitFrame( FRAME_OUT, "CAN2", networkAddress );;
    }

    if( controller->readFramesList[ frameID ] == NULL || controller->writeFramesList[ frameID ] == NULL ) 
    {
      MotorDrive_Disconnect( controller );
      ERROR_EVENT( "failed creating frame %s for motor with network index %u", networkAddress, networkIndex );
      return NULL;
    }
  }

  EnableDigitalOutput( controller, true );
  
  DEBUG_EVENT( 0, "created controller with network index %u", networkIndex );
  
  return controller;
}

void Motor_Disconnect( Motor* motor )
{
  if( motor != NULL )
  {
    if( motor->active ) Motor_Disable( motor );
    
    MotorDrive_Disconnect( motor->controller );
    motor->controller = NULL;

    free( motor );
    motor = NULL;
  }
}

void MotorDrive_Disconnect( MotorDrive* controller )
{
  if( controller != NULL )
  {
    for( size_t frameID = 0; frameID < AXIS_FRAME_TYPES_NUMBER; frameID++ )
      EposNetwork_EndFrame( controller->readFramesList[ frameID ] );

    free( controller );
    controller = NULL;
  }
}

extern inline void MotorDrive_SetEncoderResolution( MotorDrive* controller, unsigned int resolution )
{
  if( resolution >= 1 )
    controller->encoderResolution = resolution;
}

extern inline void MotorDrive_SetTorqueConstant( MotorDrive* controller, double currentToTorqueRatio )
{
  if( currentToTorqueRatio >= 0.0 )
    controller->currentToTorqueRatio = currentToTorqueRatio;
}

void Motor_Enable( Motor* motor )
{
  if( motor->active ) return;
  
  SetControl( motor->controller, SWITCH_ON, false );
  SetControl( motor->controller, ENABLE_OPERATION, false );
  MotorDrive_WriteConfig( motor->controller );
        
  Timing_Delay( 200 );
        
  SetControl( motor->controller, SWITCH_ON, true );
  MotorDrive_WriteConfig( motor->controller );

  Timing_Delay( 200 );
        
  SetControl( motor->controller, ENABLE_OPERATION, true );
  MotorDrive_WriteConfig( motor->controller );

  motor->active = true;
}

void Motor_Disable( Motor* motor )
{
  if( !(motor->active) ) return;
  
  SetControl( motor->controller, SWITCH_ON, true );
  SetControl( motor->controller, ENABLE_OPERATION, false );
  MotorDrive_WriteConfig( motor->controller );
        
  Timing_Delay( 200 );
        
  SetControl( motor->controller, SWITCH_ON, false );
  MotorDrive_WriteConfig( motor->controller );

  motor->active = false;
}

double MotorDrive_GetMeasure( MotorDrive* controller, enum Dimension index )
{
  if( index >= AXIS_DIMS_NUMBER )
  {
    ERROR_EVENT( "invalid dimension index: %d\n", index );
    return 0;
  }
      
  return controller->measuresList[ index ];
}
 
double Motor_GetSetpoint( Motor* motor, enum Setpoint index )
{
  if( index >= AXIS_SETPTS_NUMBER )
  {
    ERROR_EVENT( "invalid setpoint type index: %d\n", index );
    return 0;
  }
      
  return motor->setpointsList[ index ];
}

void Motor_SetSetpoint( Motor* motor, enum Setpoint index, double value )
{
  if( index >= AXIS_SETPTS_NUMBER )
  {
    ERROR_EVENT( "invalid setpoint type index: %d\n", index );
    return;
  }
      
  motor->setpointsList[ index ] = value;
}

bool MotorDrive_CheckState( MotorDrive* controller, enum State index )
{
  if( index < 0 || index >= AXIS_STATES_NUMBER ) return false;
  
  uint16_t stateValue = stateValues[ index ];
  
  if( (controller->statusWord & stateValue) != 0 ) return true;
  else return false;
}

void MotorDrive_Reset( MotorDrive* controller )
{
  SetControl( controller, FAULT_RESET, true );
  MotorDrive_WriteConfig( controller );
  
  Timing_Delay( 200 );
  
  SetControl( controller, FAULT_RESET, false );
  MotorDrive_WriteConfig( controller );
}

static inline void SetControl( MotorDrive* controller, enum Control index, bool enabled )
{
  if( index < 0 || index >= AXIS_CONTROLS_NUMBER ) return;
  
  uint16_t controlValue = controlValues[ index ]; //0x00000001 << index;
  
  if( enabled ) controller->controlWord |= controlValue;
  else controller->controlWord &= (~controlValue);
}

void MotorDrive_ReadValues( MotorDrive* controller )
{
  // Create reading buffer
  static uint8_t payload[8];

  EposNetwork_Sync();
  
  // Read values from PDO01 (Position, Current and Status Word) to buffer
  CANFrame_Read( controller->readFramesList[ PDO01 ], payload );  
  // Update values from PDO01
  double rawPosition = payload[3] * 0x1000000 + payload[2] * 0x10000 + payload[1] * 0x100 + payload[0];
  controller->measuresList[ POSITION ] = ( rawPosition / controller->encoderResolution ) * ( 2 * PI );
  
  double current = payload[5] * 0x100 + payload[4];
  controller->measuresList[ TORQUE ] = current * controller->currentToTorqueRatio;
  
  controller->statusWord = payload[7] * 0x100 + payload[6];

  // Read values from PDO02 (Velocity and Tension) to buffer
  CANFrame_Read( controller->readFramesList[ PDO02 ], payload );  
  // Update values from PDO02
  controller->measuresList[ VELOCITY ] = payload[3] * 0x1000000 + payload[2] * 0x10000 + payload[1] * 0x100 + payload[0];
  controller->measuresList[ TENSION ] = payload[5] * 0x100 + payload[4];
}

void Motor_WriteConfig( Motor* motor )
{
  // Create writing buffer
  static uint8_t payload[8];

  int rawPositionSetpoint = (int) ( ( motor->setpointsList[ POSITION_SETPOINT ] / ( 2 * PI ) ) * motor->controller->encoderResolution );
  
  // Set values for PDO01 (Position Setpoint and Control Word)
  payload[0] = (uint8_t) ( rawPositionSetpoint & 0x000000ff );
  payload[1] = (uint8_t) ( ( rawPositionSetpoint & 0x0000ff00 ) / 0x100 );
  payload[2] = (uint8_t) ( ( rawPositionSetpoint & 0x00ff0000 ) / 0x10000 );
  payload[3] = (uint8_t) ( ( rawPositionSetpoint & 0xff000000 ) / 0x1000000 );
  payload[4] = ( 0 & 0x000000ff );
  payload[5] = ( 0 & 0x0000ff00 ) / 0x100; 
  payload[6] = (uint8_t) ( motor->controller->controlWord & 0x000000ff );
  payload[7] = (uint8_t) ( ( motor->controller->controlWord & 0x0000ff00 ) / 0x100 ); 

  // Write values from buffer to PDO01 
  CANFrame_Write( motor->controller->writeFramesList[ PDO01 ], payload );

  int velocitySetpoint = (int) motor->setpointsList[ VELOCITY_SETPOINT ];
  
  // Set values for PDO02 (Velocity Setpoint and Digital Output)
  payload[0] = (uint8_t) ( velocitySetpoint & 0x000000ff );
  payload[1] = (uint8_t) ( ( velocitySetpoint & 0x0000ff00 ) / 0x100 );
  payload[2] = (uint8_t) ( ( velocitySetpoint & 0x00ff0000 ) / 0x10000 );
  payload[3] = (uint8_t) ( ( velocitySetpoint & 0xff000000 ) / 0x1000000 );
  payload[4] = (uint8_t) ( motor->controller->digitalOutput & 0x000000ff );
  payload[5] = (uint8_t) ( ( motor->controller->digitalOutput & 0x0000ff00 ) / 0x100 ); 
  payload[6] = ( 0 & 0x000000ff );
  payload[7] = ( 0 & 0x0000ff00 ) / 0x100; 

  // Write values from buffer to PDO01
  CANFrame_Write( motor->controller->writeFramesList[ PDO02 ], payload );
  
  EposNetwork_Sync();
}

static void MotorDrive_WriteConfig( MotorDrive* controller )
{
  // Create writing buffer
  static uint8_t payload[8];
  
  // Set values for PDO01 (Position Setpoint and Control Word)
  payload[6] = (uint8_t) ( controller->controlWord & 0x000000ff );
  payload[7] = (uint8_t) ( ( controller->controlWord & 0x0000ff00 ) / 0x100 ); 

  // Write values from buffer to PDO01 
  CANFrame_Write( controller->writeFramesList[ PDO01 ], payload );
  
  // Set values for PDO02 (Velocity Setpoint and Output)
  payload[4] = (uint8_t) ( controller->digitalOutput & 0x000000ff );
  payload[5] = (uint8_t) ( ( controller->digitalOutput & 0x0000ff00 ) / 0x100 ); 

  // Write values from buffer to PDO01
  CANFrame_Write( controller->writeFramesList[ PDO02 ], payload );
  
  EposNetwork_Sync();
}

static double ReadSingleValue( MotorDrive* controller, uint16_t index, uint8_t subIndex )
{
  // Build read requisition buffer for defined value
  static uint8_t payload[8];
  
  payload[0] = 0x40; 
  payload[1] = (uint8_t) ( index & 0x000000ff );
  payload[2] = (uint8_t) ( ( index & 0x0000ff00 ) / 0x100 );//0xff); ?
  payload[3] = subIndex;
  payload[4] = 0x0;
  payload[5] = 0x0;
  payload[6] = 0x0;
  payload[7] = 0x0;

  // Write value requisition to SDO frame 
  CANFrame_Write( controller->writeFramesList[ SDO ], payload );

  Timing_Delay( 100 );

  // Read requested value from SDO frame
  CANFrame_Read( controller->readFramesList[ SDO ], payload );

  double value = payload[7] * 0x1000000 + payload[6] * 0x10000 + payload[5] * 0x100 + payload[4];

  return value; 
}

static void WriteSingleValue( MotorDrive* controller, uint16_t index, uint8_t subIndex, int value )
{
  // Build write buffer
  static uint8_t payload[8];
  
  payload[0] = 0x22; 
  payload[1] = (uint8_t) ( index & 0x000000ff );
  payload[2] = (uint8_t) ( ( index & 0x0000ff00 ) / 0x100 );
  payload[3] = subIndex;
  payload[4] = (uint8_t) ( value & 0x000000ff );
  payload[5] = (uint8_t) ( ( value & 0x0000ff00 ) / 0x100 );
  payload[6] = (uint8_t) ( ( value & 0x00ff0000 ) / 0x10000 );
  payload[7] = (uint8_t) ( ( value & 0xff000000 ) / 0x1000000 );

  // Write value to SDO frame 
  CANFrame_Write( controller->writeFramesList[ SDO ], payload );
}

extern inline void Motor_SetOperationMode( Motor* motor, enum OperationMode mode )
{
  WriteSingleValue( motor->controller, 0x6060, 0x00, operationModes[ mode ] );
}

static inline void EnableDigitalOutput( MotorDrive* controller, bool enabled )
{
  if( enabled ) WriteSingleValue( controller, 0x6060, 0x00, 0xff );
  else WriteSingleValue( controller, 0x6060, 0x00, 0x00 );
}

extern inline void MotorDrive_SetDigitalOutput( MotorDrive* controller, uint16_t output )
{
  controller->digitalOutput = output;
}


#endif	/* AXIS_H */

