//////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                   AXIS CONTROL THROUGH SHARED MEMORY (Anklebot on Linux RT)                    /////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef AXIS_H
#define	AXIS_H

#ifdef WIN32
  #include "../timing_windows.h"
#else
  #include "../timing_unix.h"
#endif

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <malloc.h>

#include <stdbool.h>

#include "../async_debug.h"

#include "ruser.h"
#include "robdecls.h"
#include "rtl_inc.h"

enum OperationMode { AXIS_OP_MODE_POSITION, AXIS_OP_MODE_VELOCITY, AXIS_OP_MODES_NUMBER };
static const uint8_t operationModes[ AXIS_OP_MODES_NUMBER ] = { 0xFF, 0xFE };

enum State { READY_2_SWITCH_ON, SWITCHED_ON, OPERATION_ENABLED, FAULT, VOLTAGE_ENABLED, 
              QUICK_STOPPED, SWITCH_ON_DISABLE, REMOTE_NMT, TARGET_REACHED, SETPOINT_ACK, AXIS_STATES_NUMBER };
static const uint16_t stateValues[ AXIS_STATES_NUMBER ] = { 1, 2, 4, 8, 16, 32, 64, 512, 1024, 4096 };

enum Control { SWITCH_ON, ENABLE_VOLTAGE, QUICK_STOP, ENABLE_OPERATION, 
               NEW_SETPOINT, CHANGE_IMMEDIATEDLY, ABS_REL, FAULT_RESET, HALT, AXIS_CONTROLS_NUMBER };
static const uint16_t controlValues[ AXIS_CONTROLS_NUMBER ] = { 1, 2, 4, 8, 16, 32, 64, 128, 256 };

enum Dimension { AXIS_POSITION, AXIS_VELOCITY, AXIS_ACCELERATION, AXIS_TORQUE, AXIS_DIMS_NUMBER }; 

typedef struct _MotorDrive
{
  Ob* sharedData;
  double measuresList[ AXIS_DIMS_NUMBER ];
  unsigned int encoderResolution;
  double gearReduction;
  uint16_t statusWord, controlWord;
  uint16_t digitalOutput;
}
MotorDrive;

typedef struct _Motor
{
  MotorDrive* drive;
  double currentToTorqueRatio;
  double setpointsList[ AXIS_OP_MODES_NUMBER ];
  bool active;
}
Motor;

Motor* Motor_Connect( unsigned int );
MotorDrive* MotorDrive_Connect( unsigned int );
void Motor_Enable( Motor* );
void Motor_Disable( Motor* );
void Motor_Disconnect( Motor* );
void MotorDrive_Disconnect( MotorDrive* );
extern inline bool Motor_IsActive( Motor* );
extern inline void MotorDrive_SetEncoderResolution( MotorDrive*, unsigned int );
extern inline void MotorDrive_SetGearReduction( MotorDrive*, double );
double MotorDrive_GetMeasure( MotorDrive*, enum Dimension );
double Motor_GetSetpoint( Motor*, enum OperationMode );
void Motor_SetSetpoint( Motor*, enum OperationMode, double );
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
Motor* Motor_Connect( unsigned int sharedMemoryKey ) 
{
  DEBUG_EVENT( 0, "created motor with network index %u", sharedMemoryKey ); 
  
  Motor* motor = (Motor*) malloc( sizeof(Motor) );
  
  for( int i = 0; i < AXIS_OP_MODES_NUMBER; i++ )
    motor->setpointsList[ i ] = 0.0;
  
  motor->currentToTorqueRatio = 0.0;
  
  if( (motor->drive = MotorDrive_Connect( sharedMemoryKey )) == NULL )
  {
    Motor_Disconnect( motor );
    return NULL;
  }

  SetControl( motor->drive, ENABLE_VOLTAGE, true );
  SetControl( motor->drive, QUICK_STOP, true );
  
  Motor_Disable( motor );

  DEBUG_EVENT( 0, "created motor with network index %u", sharedMemoryKey );
  
  return motor;
}

MotorDrive* MotorDrive_Connect( unsigned int sharedMemoryKey )
{
  DEBUG_EVENT( 0, "creating drive with network index %u", sharedMemoryKey );
  
  MotorDrive* drive = (MotorDrive*) malloc( sizeof(MotorDrive) );
  
  for( int i = 0; i < AXIS_DIMS_NUMBER; i++ )
    drive->measuresList[ i ] = 10.0;
  
  drive->encoderResolution = 1;
  drive->gearReduction = 1.0;
  
  drive->statusWord = drive->controlWord = 0;
  
  drive->digitalOutput = 0;
  
  // allocate shared memory buffers.
  int sharedMemoryFD = shmget( OB_KEY, sizeof(Ob), 0666 );
  if( sharedMemoryFD == -1 ) 
  {
    ERROR_EVENT( "could not shmget() access to shared memory key %x (the robot kernel module is probably not running)", OB_KEY );
    return NULL;
  }
  
  drive->sharedData = (Ob*) shmat( sharedMemoryFD, NULL, 0 );
  if( (s32) drive->sharedData == -1 )
  {
    ERROR_EVENT( "shred memory FD %d shmat() failed", sharedMemoryFD );
    return NULL;
  }

  EnableDigitalOutput( drive, true );
  
  DEBUG_EVENT( 0, "created drive with network index %u", sharedMemoryKey );
  
  return drive;
}

void Motor_Disconnect( Motor* motor )
{
  if( motor != NULL )
  {
    if( motor->active ) Motor_Disable( motor );
    
    MotorDrive_Disconnect( motor->drive );
    motor->drive = NULL;

    free( motor );
    motor = NULL;
  }
}

void MotorDrive_Disconnect( MotorDrive* drive )
{
  if( drive != NULL )
  {
    shmdt( drive->sharedData );

    free( drive );
    drive = NULL;
  }
}

extern inline bool Motor_IsActive( Motor* motor )
{
  return motor->active;
}

extern inline void MotorDrive_SetEncoderResolution( MotorDrive* drive, unsigned int resolution )
{
  if( resolution >= 1 )
    drive->encoderResolution = resolution;
}

extern inline void MotorDrive_SetGearReduction( MotorDrive* drive, double gearReduction )
{
  if( gearReduction != 0.0 )
    drive->gearReduction = gearReduction;
}

void Motor_Enable( Motor* motor )
{
  if( motor->active ) return;
  
  SetControl( motor->drive, SWITCH_ON, false );
  SetControl( motor->drive, ENABLE_OPERATION, false );
  MotorDrive_WriteConfig( motor->drive );
        
  Timing_Delay( 200 );
        
  SetControl( motor->drive, SWITCH_ON, true );
  MotorDrive_WriteConfig( motor->drive );

  Timing_Delay( 200 );
        
  SetControl( motor->drive, ENABLE_OPERATION, true );
  MotorDrive_WriteConfig( motor->drive );

  motor->active = true;
}

void Motor_Disable( Motor* motor )
{
  if( !(motor->active) ) return;
  
  SetControl( motor->drive, SWITCH_ON, true );
  SetControl( motor->drive, ENABLE_OPERATION, false );
  MotorDrive_WriteConfig( motor->drive );
        
  Timing_Delay( 200 );
        
  SetControl( motor->drive, SWITCH_ON, false );
  MotorDrive_WriteConfig( motor->drive );

  motor->active = false;
}

double MotorDrive_GetMeasure( MotorDrive* drive, enum Dimension index )
{
  if( index >= AXIS_DIMS_NUMBER )
  {
    ERROR_EVENT( "invalid dimension index: %d\n", index );
    return 0;
  }
      
  return drive->measuresList[ index ];
}
 
double Motor_GetSetpoint( Motor* motor, enum OperationMode index )
{
  if( index >= AXIS_OP_MODES_NUMBER )
  {
    ERROR_EVENT( "invalid setpoint type index: %d\n", index );
    return 0;
  }
      
  return motor->setpointsList[ index ];
}

void Motor_SetSetpoint( Motor* motor, enum OperationMode index, double value )
{
  if( index >= AXIS_OP_MODES_NUMBER )
  {
    ERROR_EVENT( "invalid setpoint type index: %d\n", index );
    return;
  }
  
  motor->setpointsList[ index ] = value;
}

bool MotorDrive_CheckState( MotorDrive* drive, enum State index )
{
  if( index < 0 || index >= AXIS_STATES_NUMBER ) return false;
  
  uint16_t stateValue = stateValues[ index ];
  
  if( (drive->statusWord & stateValue) != 0 ) return true;
  else return false;
}

void MotorDrive_Reset( MotorDrive* drive )
{
  SetControl( drive, FAULT_RESET, true );
  MotorDrive_WriteConfig( drive );
  
  Timing_Delay( 200 );
  
  SetControl( drive, FAULT_RESET, false );
  MotorDrive_WriteConfig( drive );
}

static inline void SetControl( MotorDrive* drive, enum Control index, bool enabled )
{
  if( index < 0 || index >= AXIS_CONTROLS_NUMBER ) return;
  
  uint16_t controlValue = controlValues[ index ]; //0x00000001 << index;
  
  if( enabled ) drive->controlWord |= controlValue;
  else drive->controlWord &= (~controlValue);
}

void MotorDrive_ReadValues( MotorDrive* drive )
{
  // Create reading buffer
  static uint8_t payload[8];

  EposNetwork_Sync();
  
  // Read values from PDO01 (Position, Current and Status Word) to buffer
  CANFrame_Read( drive->readFramesList[ PDO01 ], payload );  
  // Update values from PDO01
  double rawPosition = payload[3] * 0x1000000 + payload[2] * 0x10000 + payload[1] * 0x100 + payload[0];
  drive->measuresList[ AXIS_POSITION ] = rawPosition / ( drive->encoderResolution * drive->gearReduction );
  
  //int rawCurrent = payload[5] * 0x100 + payload[4];
  drive->measuresList[ AXIS_CURRENT ] = ( (double) payload[4] ) / 1000.0;
  
  drive->statusWord = payload[7] * 0x100 + payload[6];

  // Read values from PDO02 (Velocity and Tension) to buffer
  CANFrame_Read( drive->readFramesList[ PDO02 ], payload );  
  // Update values from PDO02
  double rpmVelocity = payload[3] * 0x1000000 + payload[2] * 0x10000 + payload[1] * 0x100 + payload[0];
  drive->measuresList[ AXIS_VELOCITY ] = rpmVelocity / ( drive->gearReduction * 60.0 );
  drive->measuresList[ AXIS_TENSION ] = payload[5] * 0x100 + payload[4];
}

void Motor_WriteConfig( Motor* motor )
{
  int rawPositionSetpoint = (int) ( motor->setpointsList[ AXIS_OP_MODE_POSITION ] * motor->drive->encoderResolution * motor->drive->gearReduction );

  int velocitySetpoint = (int) motor->setpointsList[ AXIS_OP_MODE_VELOCITY ];
}

static void MotorDrive_WriteConfig( MotorDrive* drive )
{
  /*// Create writing buffer
  static uint8_t payload[8];
  
  // Set values for PDO01 (Position Setpoint and Control Word)
  payload[6] = (uint8_t) ( drive->controlWord & 0x000000ff );
  payload[7] = (uint8_t) ( ( drive->controlWord & 0x0000ff00 ) / 0x100 ); 

  // Write values from buffer to PDO01 
  CANFrame_Write( drive->writeFramesList[ PDO01 ], payload );
  
  // Set values for PDO02 (Velocity Setpoint and Output)
  payload[4] = (uint8_t) ( drive->digitalOutput & 0x000000ff );
  payload[5] = (uint8_t) ( ( drive->digitalOutput & 0x0000ff00 ) / 0x100 ); 

  // Write values from buffer to PDO01
  CANFrame_Write( drive->writeFramesList[ PDO02 ], payload );
  
  EposNetwork_Sync();*/
}

static double ReadSingleValue( MotorDrive* drive, uint16_t index, uint8_t subIndex )
{
  // Build read requisition buffer for defined value
  /*static uint8_t payload[8];
  
  payload[0] = 0x40; 
  payload[1] = (uint8_t) ( index & 0x000000ff );
  payload[2] = (uint8_t) ( ( index & 0x0000ff00 ) / 0x100 );//0xff); ?
  payload[3] = subIndex;
  payload[4] = 0x0;
  payload[5] = 0x0;
  payload[6] = 0x0;
  payload[7] = 0x0;

  // Write value requisition to SDO frame 
  CANFrame_Write( drive->writeFramesList[ SDO ], payload );

  Timing_Delay( 100 );

  // Read requested value from SDO frame
  CANFrame_Read( drive->readFramesList[ SDO ], payload );*/

  double value = 0.0;//payload[7] * 0x1000000 + payload[6] * 0x10000 + payload[5] * 0x100 + payload[4];

  return value; 
}

static void WriteSingleValue( MotorDrive* drive, uint16_t index, uint8_t subIndex, int value )
{
  // Build write buffer
  /*static uint8_t payload[8];
  
  payload[0] = 0x22; 
  payload[1] = (uint8_t) ( index & 0x000000ff );
  payload[2] = (uint8_t) ( ( index & 0x0000ff00 ) / 0x100 );
  payload[3] = subIndex;
  payload[4] = (uint8_t) ( value & 0x000000ff );
  payload[5] = (uint8_t) ( ( value & 0x0000ff00 ) / 0x100 );
  payload[6] = (uint8_t) ( ( value & 0x00ff0000 ) / 0x10000 );
  payload[7] = (uint8_t) ( ( value & 0xff000000 ) / 0x1000000 );

  // Write value to SDO frame 
  CANFrame_Write( drive->writeFramesList[ SDO ], payload );*/
}

extern inline void Motor_SetOperationMode( Motor* motor, enum OperationMode mode )
{
  WriteSingleValue( motor->drive, 0x6060, 0x00, operationModes[ mode ] );
}

static inline void EnableDigitalOutput( MotorDrive* drive, bool enabled )
{
  if( enabled ) WriteSingleValue( drive, 0x6060, 0x00, 0xff );
  else WriteSingleValue( drive, 0x6060, 0x00, 0x00 );
}

extern inline void MotorDrive_SetDigitalOutput( MotorDrive* drive, uint16_t output )
{
  drive->digitalOutput = output;
}

// const struct
// {
//   void (*Reconnect)( int );
//   void (*Disconnect)( int );
//   void (*Enable)( int );
//   void (*Disable)( int );
//   void (*Reset)( int );
//   bool (*IsEnabled)( int );
//   bool (*HasError)( int );
//   void (*SetOption)( int, int );
//   double (*GetMeasure)( int, int );
//   void (*SetParameter)( int, int, double );
//   double (*GetParameter)( int, int );
//   void (*ReadValues)( int );
//   void (*WriteConfig)( int );
// }
// Motor = { Motor_Reconnect, Motor_Disconnect, Motor_Enable, Motor_Disable, Motor_Reset, Motor_IsActive, 
//           Motor_HasError, Motor_SetOperationMode, Motor_GetMeasure, Motor_SetParameter, Motor_GetParameter };


#endif	/* AXIS_H */

