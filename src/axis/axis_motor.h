#ifndef AXIS_MOTOR_H
#define AXIS_MOTOR_H

#ifdef __cplusplus
    extern "C" {
#endif

#include "axis/axis_types.h"

#include "file_parsing/json_parser.h"

#include "debug/async_debug.h"

enum AxisMotorMeasures { MOTOR_POSITION, MOTOR_FORCE, MOTOR_MEASURES_NUMBER };
      
typedef struct _AxisMotorData
{
  int axisID;
  AxisInterface interface;
  double measuresList[ MOTOR_MEASURES_NUMBER ];
  double measureOffsetsList[ MOTOR_MEASURES_NUMBER ];
  size_t operationMode;
  unsigned int encoderResolution;
  double currentToForceRatio;
  double gearReduction;
}
AxisMotorData;

typedef AxisMotorData* AxisMotor;

static AxisMotor AxisMotor_Init( const char* );
static inline void AxisMotor_End( AxisMotor );
static inline void AxisMotor_Enable( AxisMotor );
static inline void AxisMotor_Disable( AxisMotor );
static inline void AxisMotor_Reset( AxisMotor );
static void AxisMotor_SetOffset( AxisMotor );
static inline bool AxisMotor_IsEnabled( AxisMotor );
static inline bool AxisMotor_HasError( AxisMotor );
static double* AxisMotor_ReadMeasures( AxisMotor );
static void AxisMotor_WriteControl( AxisMotor, double );

static inline AxisMotor LoadAxisMotorData( const char* );
static inline void UnloadAxisMotorData( CANInterface* );

static inline void ReadRawMeasures( AxisMotor motor );

static int AxisMotor_Init( const char* configFileName )
{
  AxisMotor newAxisMotor = (AxisMotor) malloc( sizeof(AxisMotorData) );
  
  // File Parsing
  
  
  newAxisMotor->interface = &AxisCANEPOSOperations;
  newAxisMotor->axisID = newAxisMotor->interface->Connect( "CAN Motor Teste" );
  newAxisMotor->operationMode = AXIS_SETPOINT_CURRENT;
  newAxisMotor->encoderResolution = 1;
  newAxisMotor->currentToForceRatio = 151.8; // 0.0302;
  newAxisMotor->gearReduction = 1.0; // 0.0025 / ( 2 * 2 * M_PI );
  
  return newAxisMotor;
}

static inline void AxisMotor_End( AxisMotor motor )
{
  if( motor == NULL ) return;
  
  motor->interface->Disconnect( motor->axisID );
  
  free( motor );
}

static inline void AxisMotor_Enable( AxisMotor motor )
{
  if( motor == NULL ) return;
  
  motor->interface->Enable( motor->axisID );
}

static inline void AxisMotor_Disable( AxisMotor motor )
{
  if( motor == NULL ) return;
  
  motor->interface->Disable( motor->axisID );
}

static inline void AxisMotor_Reset( AxisMotor motor )
{
  if( motor == NULL ) return;
  
  motor->interface->Reset( motor->axisID );
}

static void AxisMotor_SetOffset( AxisMotor motor )
{
  static double rawMeasuresList[ AXIS_MEASURES_NUMBER ];
  
  if( motor == NULL ) return;
  
  motor->interface->ReadMeasures( motor->axisID, rawMeasuresList );
  
  motor->measureOffsetsList[ MOTOR_POSITION ] = rawMeasuresList[ AXIS_MEASURE_ENCODER ] / ( motor->encoderResolution * motor->gearReduction );
  motor->measureOffsetsList[ MOTOR_FORCE ] = rawMeasuresList[ AXIS_MEASURE_CURRENT ] * motor->currentToForceRatio * motor->gearReduction;
}

static inline bool AxisMotor_IsEnabled( AxisMotor motor )
{
  if( motor == NULL ) return false;
  
  return motor->interface->IsEnabled( motor->axisID );
}

static inline bool AxisMotor_HasError( AxisMotor motor )
{
  if( motor == NULL ) return false;
  
  return motor->interface->HasError( motor->axisID );
}

static double* AxisMotor_ReadMeasures( AxisMotor motor )
{
  static double rawMeasuresList[ AXIS_MEASURES_NUMBER ];
  
  if( motor == NULL ) return NULL;
  
  motor->interface->ReadMeasures( motor->axisID, rawMeasuresList );
  
  motor->measuresList[ MOTOR_POSITION ] = rawMeasuresList[ AXIS_MEASURE_ENCODER ] / ( motor->encoderResolution * motor->gearReduction );
  motor->measuresList[ MOTOR_FORCE ] = rawMeasuresList[ AXIS_MEASURE_CURRENT ] * motor->currentToForceRatio * motor->gearReduction;
  
  for( size_t measureIndex = 0; measureIndex < MOTOR_MEASURES_NUMBER; measureIndex++ )
    motor->measuresList[ measureIndex ] -= motor->measureOffsetsList[ measureIndex ];
}

static void AxisMotor_WriteControl( AxisMotor motor, double setpoint )
{
  static double rawSetpointsList[ AXIS_SETPOINTS_NUMBER ];
   
  if( motor == NULL ) return;
  
  rawSetpointsList[ AXIS_SETPOINT_ENCODER ] = setpoint * motor->encoderResolution * motor->gearReduction + motor->measureOffsetsList[ MOTOR_POSITION ];
  rawSetpointsList[ AXIS_SETPOINT_VELOCITY ] = setpoint * motor->gearReduction;
  rawSetpointsList[ AXIS_SETPOINT_CURRENT ] = setpoint / ( motor->currentToForceRatio * motor->gearReduction ) + motor->measureOffsetsList[ MOTOR_FORCE ];
  
  motor->interface->WriteSetpoints( motor->axisID, rawSetpointsList[ motor->operationMode ] );
}

//static inline CANInterface* LoadAxisMotorData( const char* );
//static inline void UnloadAxisMotorData( CANInterface* );

#ifdef __cplusplus
    }
#endif

#endif  // AXIS_MOTOR_H
