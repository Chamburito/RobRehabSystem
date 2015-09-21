#ifndef SERIES_ELASTIC_ACTUATOR_H
#define SERIES_ELASTIC_ACTUATOR_H 

#include "actuator/actuator_interface.h"
#include "spline3_interpolation.h"
#include "filters.h"

#include "klib/khash.h"
#include "file_parsing/json_parser.h"

#include "debug/async_debug.h"

typedef struct _Actuator
{
  int motorID, sensorID;
  double measuresList[ ACTUATOR_VARS_NUMBER ];
  double gearConversionFactor;
  Splined3Curve* interactionForceCurve;
  enum ActuatorVariables operationMode;
}
Actuator;

KHASH_MAP_INIT_INT( SEA, Actuator* )
static khash_t( SEA )* actuatorsList = NULL;

/*static int SEA_Init( const char* );
static void SEA_End( int );
static void SEA_Enable( int );
static void SEA_Disable( int );
static void SEA_Reset( int );
static void SEA_Calibrate( int );
static bool SEA_IsEnabled( int );
static bool SEA_HasError( int );
static void SEA_ReadAxes( int );
static double SEA_GetMeasure( int, enum AxisDimensions );
static void SEA_SetSetpoint( int, double );*/

#define ACTUATOR_FUNCTION( rvalue, namespace, name, ... ) static rvalue namespace##_##name( __VA_ARGS__ );
ACTUATOR_INTERFACE( SEA )
#undef ACTUATOR_FUNCTION

static inline Actuator* LoadActuatorData( const char* );
static inline void UnloadActuatorData( Actuator* );

#define ACTUATOR_FUNCTION( rvalue, namespace, name, ... ) .name = namespace##_##name,
const ActuatorOperations SEActuatorOperations = { ACTUATOR_INTERFACE( SEA ) };/*{ .Init = SEA_Init, .End = SEA_End, .Enable = SEA_Enable, .Disable = SEA_Disable, .Reset = SEA_Reset, .Calibrate = SEA_Calibrate,
                                                  .IsEnabled = SEA_IsEnabled, .HasError = SEA_HasError, .ReadAxes = SEA_ReadAxes, .GetMeasure = SEA_GetMeasure, .SetSetpoint = SEA_SetSetpoint };*/
#undef ACTUATOR_FUNCTION
                                         
int SEA_Init( const char* configFileName )
{
  DEBUG_PRINT( "trying to create series elastic actuator %s", configFileName );
  
  if( actuatorsList == NULL ) actuatorsList = kh_init( SEA );
  
  int configKey = (int) kh_str_hash_func( configFileName );
  
  int insertionStatus;
  khint_t newActuatorID = kh_put( SEA, actuatorsList, configKey, &insertionStatus );
  if( insertionStatus == -1 ) return -1;
  
  kh_value( actuatorsList, newActuatorID ) = LoadActuatorData( configFileName );
  if( kh_value( actuatorsList, newActuatorID ) == NULL )
  {
    SEA_End( (int) newActuatorID );
    return -1;
  }
  
  DEBUG_PRINT( "created series elastic actuator %s (iterator: %u)", configFileName, newActuatorID );
  
  return (int) newActuatorID;
}

static void SEA_End( int actuatorID )
{
  DEBUG_PRINT( "ending SE actuator %d", actuatorID );
  
  if( kh_exist( actuatorsList, actuatorID ) )
  {
    UnloadActuatorData( kh_value( actuatorsList, actuatorID ) );
    
    kh_del( SEA, actuatorsList, actuatorID );
    
    if( kh_size( actuatorsList ) == 0 )
    {
      kh_destroy( SEA, actuatorsList );
      actuatorsList = NULL;
    }
  }
}

static void SEA_Enable( int actuatorID )
{
  if( kh_exist( actuatorsList, (khint_t) actuatorID ) )
  {
    Actuator* actuator = kh_value( actuatorsList, (khint_t) actuatorID );
    
    DEBUG_PRINT( "enabling actuator %u (key: %d)", (khint_t) actuatorID, kh_key( actuatorsList, (khint_t) actuatorID ) );
    
    AxisMotor_Enable( actuator->motorID );
  }
}

static void SEA_Disable( int actuatorID )
{
  DEBUG_EVENT( 0, "disabling SE actuator %d", actuatorID );
  
  if( kh_exist( actuatorsList, (khint_t) actuatorID ) )
  {
    Actuator* actuator = kh_value( actuatorsList, (khint_t) actuatorID );
    
    AxisMotor_Disable( actuator->motorID );
  }
}

static void SEA_Reset( int actuatorID )
{
  if( kh_exist( actuatorsList, (khint_t) actuatorID ) )
  {
    Actuator* actuator = kh_value( actuatorsList, (khint_t) actuatorID );
    
    AxisMotor_Reset( actuator->motorID );
    AxisSensor_Reset( actuator->sensorID );
  }
}

static void SEA_Calibrate( int actuatorID )
{
  if( kh_exist( actuatorsList, (khint_t) actuatorID ) )
  {
    Actuator* actuator = kh_value( actuatorsList, (khint_t) actuatorID );
    
    AxisMotor_SetOffset( actuator->motorID );
    AxisSensor_SetOffset( actuator->sensorID );
  }
}

static bool SEA_IsEnabled( int actuatorID )
{
  if( kh_exist( actuatorsList, (khint_t) actuatorID ) )
  {
    Actuator* actuator = kh_value( actuatorsList, (khint_t) actuatorID );
    
    return ( AxisMotor_IsEnabled( actuator->motorID ) );
  }
  
  return false;
}

static bool SEA_HasError( int actuatorID )
{
  if( kh_exist( actuatorsList, (khint_t) actuatorID ) )
  {
    Actuator* actuator = kh_value( actuatorsList, (khint_t) actuatorID );
    
    if( AxisMotor_HasError( actuator->motorID ) ) return true;
    else if( AxisSensor_HasError( actuator->sensorID ) ) return true;
  }
  
  return false;
}

static double* SEA_ReadAxes( int actuatorID )
{
  if( !kh_exist( actuatorsList, (khint_t) actuatorID ) ) return NULL; 
    
  Actuator* actuator = kh_value( actuatorsList, (khint_t) actuatorID );
  
  double* motorMeasuresList = AxisMotor_ReadMeasures( actuator->motorID );
  double sensorMeasure = AxisSensor_Read( actuator->sensorID );
  if( motorMeasuresList != NULL && sensorMeasure != 0.0 )
  {
    actuator->measuresList[ ACTUATOR_POSITION ] = sensorMeasure / actuator->gearConversionFactor;
    
    actuator->measuresList[ ACTUATOR_FORCE ] = 320 * ( sensorMeasure - motorMeasuresList[ MOTOR_POSITION ] );     //Newton
    
    //DEBUG_PRINT( "current: %.3f - force: %.3f (initial: %.3f)", sensorMeasuresList[ AXIS_VELOCITY ], actuator->measuresList[ AXIS_FORCE ], actuator->initialForce );
    
    /*DEBUG_PRINT( "p: %.3f - v: %.3f - f: %.3f (%.3f)", actuator->measuresList[ AXIS_POSITION ], actuator->measuresList[ AXIS_VELOCITY ], 
    actuator->measuresLis*t[ AXIS_FORCE ], actuator->initialForce );*/
    
    //actuator->measuresList[ AXIS_FORCE ] = Spline3Interp.GetValue( actuator->interactionForceCurve, deformation );
    actuator->measuresList[ ACTUATOR_FORCE ] *= actuator->gearConversionFactor;
  }
  
  return (double*) actuator->measuresList;
}

static void SEA_SetSetpoint( int actuatorID, double setpoint )
{
  static double setpointsList[ ACTUATOR_VARS_NUMBER ];
  
  if( !kh_exist( actuatorsList, (khint_t) actuatorID ) ) return;

  Actuator* actuator = kh_value( actuatorsList, (khint_t) actuatorID );
  
  setpointsList[ ACTUATOR_POSITION ] = setpoint * actuator->gearConversionFactor;
  setpointsList[ ACTUATOR_VELOCITY ] = setpoint * actuator->gearConversionFactor;
  setpointsList[ ACTUATOR_FORCE ] = setpoint / actuator->gearConversionFactor;
  
  AxisMotor_WriteControl( actuator->motorID, setpointsList[ actuator->operationMode ] );
}

const enum AxisMotorVariables MOTOR_OPERATION_MODES[ ACTUATOR_VARS_NUMBER ] = { MOTOR_POSITION, MOTOR_VELOCITY, MOTOR_FORCE };
static void SEA_SetOperationMode( int actuatorID, enum ActuatorVariables mode )
{
  if( !kh_exist( actuatorsList, (khint_t) actuatorID ) ) return NULL;
  
  if( mode < 0 || mode >= ACTUATOR_VARS_NUMBER ) return;
    
  Actuator* actuator = kh_value( actuatorsList, (khint_t) actuatorID );
  
  AxisMotor_SetOperationMode( actuator->motorID, MOTOR_OPERATION_MODES[ mode ] );
  
  actuator->operationMode = mode;
}


static inline Actuator* LoadActuatorData( const char* configFileName )
{
  //static char searchPath[ FILE_PARSER_MAX_PATH_LENGTH ];
  
  //bool loadError = false;
  
  Actuator* newActuator = (Actuator*) malloc( sizeof(Actuator) );
  memset( newActuator, 0, sizeof(Actuator) );
  
  /*FileParser parser = JSONParser;
  int configFileID = parser.LoadFile( configFileName );
  if( configFileID != -1 )
  {
    for( size_t axisIndex = 0; axisIndex < AXES_NUMBER; axisIndex++ )
    {
      sprintf( searchPath, "axes.%s", AXIS_NAMES[ axisIndex ] );
      parser.SetBaseKey( configFileID, searchPath );
      
      if( (axesList[ axisIndex ]->interface = GetAxisInterface( parser.GetStringValue( configFileID, "interface_type" ) )) != NULL )
      {
        if( (axesList[ axisIndex ]->ID = axesList[ axisIndex ]->interface->Connect( parser.GetStringValue( configFileID, "name" ) )) == -1 ) loadError = true;
      }
      else loadError = true;
      
      axesList[ axisIndex ]->gearConversionFactor = parser.GetRealValue( configFileID, "gear_reduction" );
    }
    
    parser.SetBaseKey( configFileID, "" );
    if( (newActuator->interactionForceCurve = Spline3Interp.LoadCurve( parser.GetStringValue( configFileID, "spring_force_curve" ) )) == NULL ) loadError = true;
    
    parser.UnloadFile( configFileID );
  }
  else
  {
    loadError = true;
    DEBUG_PRINT( "configuration file for series elastic actuator %s not found", configFileName );
  }
  
  if( loadError )
  {
    UnloadActuatorData( newActuator );
    return NULL;
  }*/
  
  newActuator->motorID = AxisMotor_Init( "Motor Teste" );
  newActuator->sensorID = AxisSensor_Init( "Sensor Teste" );
  
  return newActuator;
}

static inline void UnloadActuatorData( Actuator* actuator )
{
  if( actuator != NULL )
  {
    AxisMotor_End( actuator->motorID );
    AxisSensor_End( actuator->sensorID );
    
    Spline3Interp.UnloadCurve( actuator->interactionForceCurve );
  }
}


#endif // SERIES_ELASTIC_ACTUATOR_H
