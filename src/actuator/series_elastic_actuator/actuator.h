#ifndef SERIES_ELASTIC_ACTUATOR_H
#define SERIES_ELASTIC_ACTUATOR_H 

#include "actuator/actuator_interface.h"
#include "spline3_interpolation.h"
#include "filters.h"

#include "klib/khash.h"
#include "file_parsing/json_parser.h"

#include "debug/async_debug.h"

typedef struct _ActuatorData
{
  AxisMotor motor;
  AxisSensor sensor;
  double measuresList[ AXIS_VARS_NUMBER ];
  double gearConversionFactor;
  Splined3Curve interactionForceCurve;
  enum AxisVariables operationMode;
}
ActuatorData;

typedef ActuatorData* Actuator;

KHASH_MAP_INIT_INT( SEA, Actuator )
static khash_t( SEA )* actuatorsList = NULL;

IMPLEMENT_INTERFACE( Actuator, SEActuatorOperations )

//#define NAMESPACE SEA
//ACTUATOR_INTERFACE( INIT_NAMESPACE_FILE, NAMESPACE )
//const ActuatorOperations SEActuatorOperations = { ACTUATOR_INTERFACE( INIT_NAMESPACE_STRUCT, NAMESPACE ) };
//#undef NAMESPACE

static inline Actuator LoadActuatorData( const char* );
static inline void UnloadActuatorData( Actuator );
                                         
int /*SEA_*/Init( const char* configFileName )
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
    /*SEA_*/End( (int) newActuatorID );
    return -1;
  }
  
  DEBUG_PRINT( "created series elastic actuator %s (iterator: %u)", configFileName, newActuatorID );
  
  return (int) newActuatorID;
}

void /*SEA_*/End( int actuatorID )
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

void /*SEA_*/Enable( int actuatorID )
{
  if( kh_exist( actuatorsList, (khint_t) actuatorID ) )
  {
    Actuator actuator = kh_value( actuatorsList, (khint_t) actuatorID );
    
    DEBUG_PRINT( "enabling actuator %u (key: %d)", (khint_t) actuatorID, kh_key( actuatorsList, (khint_t) actuatorID ) );
    
    AxisMotors_Enable( actuator->motor );
  }
}

void /*SEA_*/Disable( int actuatorID )
{
  DEBUG_EVENT( 0, "disabling SE actuator %d", actuatorID );
  
  if( kh_exist( actuatorsList, (khint_t) actuatorID ) )
  {
    Actuator actuator = kh_value( actuatorsList, (khint_t) actuatorID );
    
    AxisMotors_Disable( actuator->motor );
  }
}

void /*SEA_*/Reset( int actuatorID )
{
  if( kh_exist( actuatorsList, (khint_t) actuatorID ) )
  {
    Actuator actuator = kh_value( actuatorsList, (khint_t) actuatorID );
    
    AxisMotors_Reset( actuator->motor );
    AxisSensors_Reset( actuator->sensor );
  }
}

void /*SEA_*/Calibrate( int actuatorID )
{
  if( kh_exist( actuatorsList, (khint_t) actuatorID ) )
  {
    Actuator actuator = kh_value( actuatorsList, (khint_t) actuatorID );
    
    AxisMotors_SetOffset( actuator->motor );
    AxisSensors_SetOffset( actuator->sensor );
  }
}

bool /*SEA_*/IsEnabled( int actuatorID )
{
  if( kh_exist( actuatorsList, (khint_t) actuatorID ) )
  {
    Actuator actuator = kh_value( actuatorsList, (khint_t) actuatorID );
    
    return ( AxisMotors_IsEnabled( actuator->motor ) );
  }
  
  return false;
}

bool /*SEA_*/HasError( int actuatorID )
{
  if( kh_exist( actuatorsList, (khint_t) actuatorID ) )
  {
    Actuator actuator = kh_value( actuatorsList, (khint_t) actuatorID );
    
    if( AxisMotors_HasError( actuator->motor ) ) return true;
    else if( AxisSensors_HasError( actuator->sensor ) ) return true;
  }
  
  return false;
}

double* /*SEA_*/ReadAxes( int actuatorID )
{
  if( !kh_exist( actuatorsList, (khint_t) actuatorID ) ) return NULL; 
    
  Actuator actuator = kh_value( actuatorsList, (khint_t) actuatorID );
  
  double* motorMeasuresList = AxisMotors_ReadMeasures( actuator->motor );
  double sensorMeasure = AxisSensors_Read( actuator->sensor );
  if( motorMeasuresList != NULL && sensorMeasure != 0.0 )
  {
    actuator->measuresList[ AXIS_POSITION ] = sensorMeasure / actuator->gearConversionFactor;
    
    double deformation = sensorMeasure - motorMeasuresList[ AXIS_POSITION ];         // mm

    actuator->measuresList[ AXIS_FORCE ] = Spline3Interp.GetValue( actuator->interactionForceCurve, deformation );
    //actuator->measuresList[ AXIS_FORCE ] *= actuator->gearConversionFactor;
    //actuator->measuresList[ AXIS_FORCE ] = -104.0 * deformation;
    
    DEBUG_PRINT( "motor: %.3f - sensor: %.3f - force: %.3f", motorMeasuresList[ AXIS_POSITION ], sensorMeasure, actuator->measuresList[ AXIS_FORCE ] );
  }
  
  return (double*) actuator->measuresList;
}

void /*SEA_*/SetSetpoint( int actuatorID, double setpoint )
{
  static double setpointsList[ AXIS_VARS_NUMBER ];
  
  if( !kh_exist( actuatorsList, (khint_t) actuatorID ) ) return;

  Actuator actuator = kh_value( actuatorsList, (khint_t) actuatorID );
  
  setpointsList[ AXIS_POSITION ] = setpoint * actuator->gearConversionFactor;
  setpointsList[ AXIS_VELOCITY ] = setpoint * actuator->gearConversionFactor;
  setpointsList[ AXIS_FORCE ] = setpoint / actuator->gearConversionFactor;
  
  AxisMotors_WriteControl( actuator->motor, setpointsList[ actuator->operationMode ] );
}

void /*SEA_*/SetOperationMode( int actuatorID, enum AxisVariables mode )
{
  if( !kh_exist( actuatorsList, (khint_t) actuatorID ) ) return;
  
  if( mode < 0 || mode >= AXIS_VARS_NUMBER ) return;
    
  Actuator actuator = kh_value( actuatorsList, (khint_t) actuatorID );
  
  AxisMotors_SetOperationMode( actuator->motor, mode );
  
  actuator->operationMode = mode;
}


inline Actuator LoadActuatorData( const char* configFileName )
{
  //static char searchPath[ FILE_PARSER_MAX_PATH_LENGTH ];
  
  //bool loadError = false;
  
  Actuator newActuator = (Actuator) malloc( sizeof(ActuatorData) );
  memset( newActuator, 0, sizeof(ActuatorData) );
  
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
  
  newActuator->motor = AxisMotors_Init( "Motor Teste" );
  newActuator->sensor = AxisSensors_Init( "Sensor Teste" );
  newActuator->gearConversionFactor = 1.0;
  newActuator->interactionForceCurve = Spline3Interp.LoadCurve( "spline3_curves/spring104_interaction_torque" );
  
  return newActuator;
}

inline void UnloadActuatorData( Actuator actuator )
{
  if( actuator != NULL )
  {
    AxisMotors_End( actuator->motor );
    AxisSensors_End( actuator->sensor );
    
    Spline3Interp.UnloadCurve( actuator->interactionForceCurve );
  }
}


#endif // SERIES_ELASTIC_ACTUATOR_H
