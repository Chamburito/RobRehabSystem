#ifndef MOTORS_H
#define MOTORS_H

#ifdef __cplusplus
    extern "C" {
#endif

#include "signal_io/interface.h"

#include "config_parser.h"
#include "plugin_loader.h"
      
#include "debug/async_debug.h"
      
typedef struct _MotorData
{
  DECLARE_MODULE_INTERFACE_REF( SIGNAL_IO_INTERFACE );
  int interfaceID;
  unsigned int outputChannel;
  double outputGain, outputOffset;
  double outputMin, outputMax;
}
MotorData;

typedef MotorData* Motor;


#define MOTORS_FUNCTIONS( namespace, function_init ) \
        function_init( Motor, namespace, Init, const char* ) \
        function_init( void, namespace, End, Motor ) \
        function_init( void, namespace, Enable, Motor ) \
        function_init( void, namespace, Disable, Motor ) \
        function_init( void, namespace, Reset, Motor ) \
        function_init( void, namespace, SetOffset, Motor, double ) \
        function_init( bool, namespace, IsEnabled, Motor ) \
        function_init( bool, namespace, HasError, Motor ) \
        function_init( void, namespace, WriteControl, Motor, double )

INIT_NAMESPACE_INTERFACE( Motors, MOTORS_FUNCTIONS )


char filePath[ DATA_IO_MAX_FILE_PATH_LENGTH ];
Motor Motors_Init( const char* configFileName )
{
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "Initializing Axis Motor %s", configFileName );
  
  Motor newMotor = NULL;
  
  sprintf( filePath, "motors/%s", configFileName );
  int configFileID = ConfigParsing.LoadConfigFile( filePath );
  if( configFileID != DATA_INVALID_ID )
  {
    ConfigParser parser = ConfigParsing.GetParser();
    
    newMotor = (Motor) malloc( sizeof(MotorData) );
    memset( newMotor, 0, sizeof(MotorData) );

    bool loadSuccess = true;
    sprintf( filePath, "signal_io/%s", parser.GetStringValue( configFileID, "", "output_interface.type" ) );
    GET_PLUGIN_IMPLEMENTATION( SIGNAL_IO_INTERFACE, filePath, newMotor, &loadSuccess );
    if( loadSuccess )
    {
      newMotor->interfaceID = newMotor->InitTask( parser.GetStringValue( configFileID, "", "output_interface.id" ) );
      if( newMotor->interfaceID != SIGNAL_IO_TASK_INVALID_ID ) 
      {
        newMotor->outputChannel = (unsigned int) parser.GetIntegerValue( configFileID, -1, "output_interface.channel" );
        DEBUG_PRINT( "trying to aquire channel %u from interface %d", newMotor->outputChannel, newMotor->interfaceID );
        loadSuccess = newMotor->AquireOutputChannel( newMotor->interfaceID, newMotor->outputChannel );
      }
      else loadSuccess = false;
    }

    newMotor->outputGain = parser.GetRealValue( configFileID, 1.0, "output_gain.multiplier" );
    newMotor->outputGain /= parser.GetRealValue( configFileID, 1.0, "output_gain.divisor" );
    
    newMotor->outputMin = parser.GetRealValue( configFileID, 0.0, "output_limits.min" );
    newMotor->outputMax = parser.GetRealValue( configFileID, 0.0, "output_limits.max" );

    parser.UnloadData( configFileID );

    if( !loadSuccess )
    {
      Motors_End( newMotor );
      return NULL;
    }

    /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "Axis Motor %s initialized", configFileName );
  }
  else
    DEBUG_PRINT( "configuration parser for axis sensor %s not available", configFileName );
  
  return newMotor;
}

inline void Motors_End( Motor motor )
{
  if( motor == NULL ) return;
  
  motor->EndTask( motor->interfaceID );
  
  free( motor );
}

inline void Motors_Enable( Motor motor )
{
  if( motor == NULL ) return;
  
  motor->Reset( motor->interfaceID );
  motor->EnableOutput( motor->interfaceID, true );
}

inline void Motors_Disable( Motor motor )
{
  if( motor == NULL ) return;
  
  motor->EnableOutput( motor->interfaceID, false );
}

inline void Motors_Reset( Motor motor )
{
  if( motor == NULL ) return;
  
  motor->Reset( motor->interfaceID );
}

inline void Motors_SetOffset( Motor motor, double offset )
{
  if( motor == NULL ) return;
  
  motor->outputOffset = offset;
}

inline bool Motors_IsEnabled( Motor motor )
{
  if( motor == NULL ) return false;
  
  return motor->IsOutputEnabled( motor->interfaceID );
}

inline bool Motors_HasError( Motor motor )
{
  if( motor == NULL ) return false;
  
  return motor->HasError( motor->interfaceID );
}

void Motors_WriteControl( Motor motor, double setpoint )
{
  if( motor == NULL ) return;
  
  setpoint = ( setpoint + motor->outputOffset ) * motor->outputGain;
  
  if( setpoint > motor->outputMax ) setpoint = motor->outputMax;
  else if( setpoint < motor->outputMin ) setpoint = motor->outputMin;
  
  motor->Write( motor->interfaceID, motor->outputChannel, setpoint );
}

#ifdef __cplusplus
    }
#endif

#endif  // MOTORS_H
