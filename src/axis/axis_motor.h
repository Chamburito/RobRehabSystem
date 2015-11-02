#ifndef AXIS_MOTOR_H
#define AXIS_MOTOR_H

#ifdef __cplusplus
    extern "C" {
#endif

#include "axis/axis_interface.h"

#include "config_parser.h"
#include "plugin_loader.h"
      
#include "debug/async_debug.h"
      
enum AxisMotorVariables { AXIS_POSITION = AXIS_ENCODER, AXIS_VELOCITY = AXIS_RPS, AXIS_FORCE = AXIS_CURRENT };
      
typedef struct _AxisMotorData
{
  int axisID;
  AxisInterface interface;
  double measuresList[ AXIS_VARS_NUMBER ];
  double measureGainsList[ AXIS_VARS_NUMBER ];
  double measureOffsetsList[ AXIS_VARS_NUMBER ];
  enum AxisMotorVariables operationMode;
}
AxisMotorData;

typedef AxisMotorData* AxisMotor;


#define AXIS_MOTOR_FUNCTIONS( namespace, function_init ) \
        function_init( AxisMotor, namespace, Init, const char* ) \
        function_init( void, namespace, End, AxisMotor ) \
        function_init( void, namespace, Enable, AxisMotor ) \
        function_init( void, namespace, Disable, AxisMotor ) \
        function_init( void, namespace, Reset, AxisMotor ) \
        function_init( void, namespace, SetOffset, AxisMotor ) \
        function_init( bool, namespace, IsEnabled, AxisMotor ) \
        function_init( bool, namespace, HasError, AxisMotor ) \
        function_init( double*, namespace, ReadMeasures, AxisMotor ) \
        function_init( void, namespace, WriteControl, AxisMotor, double ) \
        function_init( void, namespace, SetOperationMode, AxisMotor, enum AxisMotorVariables )

INIT_NAMESPACE_INTERFACE( AxisMotors, AXIS_MOTOR_FUNCTIONS )


//static inline void ReadRawMeasures( AxisMotor motor );

AxisMotor AxisMotors_Init( const char* configFileName )
{
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "Initializing Axis Motor %s", configFileName );
  
  AxisMotor newMotor = NULL;
  
  if( ConfigParsing.IsAvailable() )
  {
    ParserInterface parser = ConfigParsing.GetParser();
    int configFileID = parser.LoadFileData( configFileName );
    if( configFileID != PARSED_DATA_INVALID_ID )
    {
      bool loadSuccess = true;
  
      newMotor = (AxisMotor) malloc( sizeof(AxisMotorData) );
      memset( newMotor, 0, sizeof(AxisMotorData) );
    
      GET_PLUGIN_INTERFACE( AXIS_INTERFACE_FUNCTIONS, parser.GetStringValue( configFileID, "", "interface.type" ), newMotor->interface, loadSuccess );
      if( loadSuccess )
      {
        newMotor->axisID = newMotor->interface.Connect( (unsigned int) parser.GetIntegerValue( configFileID, AXIS_INVALID_ID, "interface.id" ) );
        if( newMotor->axisID == AXIS_INVALID_ID ) loadSuccess = false;
      }
    
      unsigned int encoderResolution = (unsigned int) parser.GetIntegerValue( configFileID, 1, "encoder_resolution" ); //4096;
      double currentToForceRatio = parser.GetRealValue( configFileID, 1.0, "force_constant" ); //151.8; // 0.0302;
      double gearReduction = parser.GetRealValue( configFileID, 1.0, "gear_reduction" ); //150.0 / ( 2 * M_PI ); // 0.0025 / ( 2 * 2 * M_PI );
  
      newMotor->measureGainsList[ AXIS_POSITION ] = 1.0 / ( encoderResolution * gearReduction );
      newMotor->measureGainsList[ AXIS_VELOCITY ] = 1.0 / gearReduction;
      newMotor->measureGainsList[ AXIS_FORCE ] = currentToForceRatio * gearReduction;
    
      parser.UnloadData( configFileID );
    
      if( !loadSuccess )
      {
        AxisMotors_End( newMotor );
        return NULL;
      }
      
      /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "Axis Motor %s initialized", configFileName );
    }
    else
      DEBUG_PRINT( "configuration file for axis sensor %s not found", configFileName );
  }
  else
    DEBUG_PRINT( "configuration parser for axis sensor %s not available", configFileName );
  
  return newMotor;
}

inline void AxisMotors_End( AxisMotor motor )
{
  if( motor == NULL ) return;
  
  motor->interface.Disconnect( motor->axisID );
  
  free( motor );
}

inline void AxisMotors_Enable( AxisMotor motor )
{
  if( motor == NULL ) return;
  
  motor->interface.Enable( motor->axisID );
}

inline void AxisMotors_Disable( AxisMotor motor )
{
  if( motor == NULL ) return;
  
  motor->interface.Disable( motor->axisID );
}

inline void AxisMotors_Reset( AxisMotor motor )
{
  if( motor == NULL ) return;
  
  motor->interface.Reset( motor->axisID );
}

inline void AxisMotors_SetOffset( AxisMotor motor )
{
  static double rawMeasuresList[ AXIS_VARS_NUMBER ];
  
  if( motor == NULL ) return;
  
  motor->interface.ReadMeasures( motor->axisID, rawMeasuresList );
  
  for( size_t measureIndex = 0; measureIndex < AXIS_VARS_NUMBER; measureIndex++ )
    motor->measureOffsetsList[ measureIndex ] = rawMeasuresList[ measureIndex ] * motor->measureGainsList[ measureIndex ];
}

inline bool AxisMotors_IsEnabled( AxisMotor motor )
{
  if( motor == NULL ) return false;
  
  return motor->interface.IsEnabled( motor->axisID );
}

inline bool AxisMotors_HasError( AxisMotor motor )
{
  if( motor == NULL ) return false;
  
  return motor->interface.HasError( motor->axisID );
}

double* AxisMotors_ReadMeasures( AxisMotor motor )
{
  static double rawMeasuresList[ AXIS_VARS_NUMBER ];
  
  if( motor == NULL ) return NULL;
  
  DEBUG_UPDATE( "motor %p reading from interface %d", motor, motor->axisID );
  motor->interface.ReadMeasures( motor->axisID, rawMeasuresList );
  
  for( size_t measureIndex = 0; measureIndex < AXIS_VARS_NUMBER; measureIndex++ )
  {
    motor->measuresList[ measureIndex ] = rawMeasuresList[ measureIndex ] * motor->measureGainsList[ measureIndex ];
    motor->measuresList[ measureIndex ] -= motor->measureOffsetsList[ measureIndex ];
  }
  
  //DEBUG_PRINT( "motor position: raw: %.3f - gain: %.5f", rawMeasuresList[ AXIS_POSITION ], motor->measuresList[ AXIS_POSITION ] );
  
  return (double*) motor->measuresList;
}

void AxisMotors_WriteControl( AxisMotor motor, double setpoint )
{
  if( motor == NULL ) return;
  
  double axisSetpoint = ( setpoint + motor->measureOffsetsList[ motor->operationMode ] ) / motor->measureGainsList[ motor->operationMode ];
  
  motor->interface.WriteSetpoint( motor->axisID, axisSetpoint );
}

void AxisMotors_SetOperationMode( AxisMotor motor, enum AxisMotorVariables mode )
{
  if( motor == NULL ) return;
  
  if( mode < 0 || mode >= (enum AxisMotorVariables) AXIS_VARS_NUMBER ) return;
  
  motor->interface.SetOperationMode( motor->axisID, (enum AxisVariables) mode );
  
  motor->operationMode = mode;
}

#ifdef __cplusplus
    }
#endif

#endif  // AXIS_MOTOR_H
