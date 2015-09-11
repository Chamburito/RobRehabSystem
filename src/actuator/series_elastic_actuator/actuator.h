#ifndef SERIES_ELASTIC_ACTUATOR_H
#define SERIES_ELASTIC_ACTUATOR_H 

#include "actuator_interface.h"
#include "spline3_interpolation.h"
#include "filters.h"

#include "axis/axis_types.h"

#include "klib/khash.h"
#include "file_parsing/json_parser.h"

#include "debug/async_debug.h"

typedef struct _Actuator
{
  Axis motor, sensor;
  double measuresList[ AXIS_DIMENSIONS_NUMBER ];
  Splined3Curve* interactionForceCurve;
  double initialForce;
}
Actuator;

KHASH_MAP_INIT_INT( SEA, Actuator* )
static khash_t( SEA )* actuatorsList = NULL;

static int SEA_Init( const char* );
static void SEA_End( int );
static void SEA_Enable( int );
static void SEA_Disable( int );
static void SEA_Reset( int );
static void SEA_Calibrate( int );
static bool SEA_IsEnabled( int );
static bool SEA_HasError( int );
static void SEA_ReadAxes( int );
static double SEA_GetMeasure( int, enum AxisDimensions );
static void SEA_SetSetpoint( int, double );

static inline Actuator* LoadActuatorData( const char* );
static inline void UnloadActuatorData( Actuator* );

const ActuatorOperations SEActuatorOperations = { .Init = SEA_Init, .End = SEA_End, .Enable = SEA_Enable, .Disable = SEA_Disable, .Reset = SEA_Reset, .Calibrate = SEA_Calibrate,
                                                  .IsEnabled = SEA_IsEnabled, .HasError = SEA_HasError, .ReadAxes = SEA_ReadAxes, .GetMeasure = SEA_GetMeasure, .SetSetpoint = SEA_SetSetpoint };
                                         
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
    
    actuator->motor.interface->Enable( actuator->motor.ID );
    actuator->sensor.interface->Enable( actuator->sensor.ID );
  }
}

static void SEA_Disable( int actuatorID )
{
  if( kh_exist( actuatorsList, (khint_t) actuatorID ) )
  {
    Actuator* actuator = kh_value( actuatorsList, (khint_t) actuatorID );
    
    actuator->motor.interface->Reset( actuator->motor.ID );
    actuator->sensor.interface->Reset( actuator->sensor.ID );
  }
}

static void SEA_Reset( int actuatorID )
{
  if( kh_exist( actuatorsList, (khint_t) actuatorID ) )
  {
    Actuator* actuator = kh_value( actuatorsList, (khint_t) actuatorID );
    
    actuator->motor.interface->Reset( actuator->motor.ID );
    actuator->sensor.interface->Reset( actuator->sensor.ID );
  }
}

static void SEA_Calibrate( int actuatorID )
{
  if( kh_exist( actuatorsList, (khint_t) actuatorID ) )
  {
    Actuator* actuator = kh_value( actuatorsList, (khint_t) actuatorID );
    
    actuator->initialForce = actuator->measuresList[ AXIS_FORCE ];
    
    DEBUG_PRINT( "calibration intial force: %g", actuator->initialForce );
  }
}

static bool SEA_IsEnabled( int actuatorID )
{
  if( kh_exist( actuatorsList, (khint_t) actuatorID ) )
  {
    Actuator* actuator = kh_value( actuatorsList, (khint_t) actuatorID );
    
    return ( actuator->motor.interface->IsEnabled( actuator->motor.ID ) );
  }
  
  return false;
}

static bool SEA_HasError( int actuatorID )
{
  if( kh_exist( actuatorsList, (khint_t) actuatorID ) )
  {
    Actuator* actuator = kh_value( actuatorsList, (khint_t) actuatorID );
    
    if( actuator->motor.interface->HasError( actuator->motor.ID ) ) return true;
    else if( actuator->sensor.interface->IsEnabled( actuator->sensor.ID ) ) return true;
  }
  
  return false;
}

static double SEA_GetMeasure( int actuatorID , enum AxisDimensions dimensionIndex )
{
  if( dimensionIndex < 0 && dimensionIndex > AXIS_DIMENSIONS_NUMBER ) return 0.0;
  
  if( kh_exist( actuatorsList, (khint_t) actuatorID ) )
  {
    Actuator* actuator = kh_value( actuatorsList, (khint_t) actuatorID );
    
    return actuator->measuresList[ dimensionIndex ];
  }
  
  return 0.0;
}

static void SEA_SetSetpoint( int actuatorID, double setpointValue )
{
  if( kh_exist( actuatorsList, (khint_t) actuatorID ) )
  {
    Actuator* actuator = kh_value( actuatorsList, (khint_t) actuatorID );
    
    actuator->motor.interface->WriteControl( actuator->motor.ID, setpointValue / actuator->motor.gearConversionFactor );
  }
}

const double p1 = -5.6853e-024;
const double p2 = 9.5074e-020;
const double p3 = -5.9028e-016;
const double p4 = 1.6529e-012;
const double p5 = -2.0475e-009;
const double p6 = 9.3491e-007;
const double p7 = 0.0021429;
const double p8 = 2.0556;
static void SEA_ReadAxes( int actuatorID )
{
  static double motorMeasuresList[ AXIS_DIMENSIONS_NUMBER ];
  static double sensorMeasuresList[ AXIS_DIMENSIONS_NUMBER ];
  
  static double analogBuffer[ 6 ];
  
  if( kh_exist( actuatorsList, (khint_t) actuatorID ) )
  {
    Actuator* actuator = kh_value( actuatorsList, (khint_t) actuatorID );
    
    actuator->motor.interface->ReadMeasures( actuator->motor.ID, motorMeasuresList );
    actuator->sensor.interface->ReadMeasures( actuator->sensor.ID, sensorMeasuresList );
    
    actuator->measuresList[ AXIS_POSITION ] = sensorMeasuresList[ AXIS_POSITION ];
    actuator->measuresList[ AXIS_VELOCITY ] = sensorMeasuresList[ AXIS_VELOCITY ];
    
    analogBuffer[5] = analogBuffer[4];
    analogBuffer[4] = analogBuffer[3];
    analogBuffer[3] = analogBuffer[2];
    analogBuffer[2] = analogBuffer[1];
    analogBuffer[1] = analogBuffer[0];
    analogBuffer[0] = sensorMeasuresList[ AXIS_FORCE ];
    double analogFiltered = (analogBuffer[0] + analogBuffer[1] + analogBuffer[2] + analogBuffer[3]+ analogBuffer[4] + analogBuffer[5])/6;
    
    double deformation = p1 * pow(analogFiltered,7) + p2 * pow(analogFiltered,6) + p3 * pow(analogFiltered,5) 
                         + p4 * pow(analogFiltered,4) + p5 * pow(analogFiltered,3) + p6 * pow(analogFiltered,2) + p7 * analogFiltered + p8;   //mm

	  actuator->measuresList[ AXIS_FORCE ] = ( 320000 * ( deformation / 1000 ) ) - actuator->initialForce;     //Newton
    
    //DEBUG_PRINT( "spring position: %.3f - force: %.3f (initial: %.3f)", deformation, actuator->measuresList[ AXIS_FORCE ], actuator->initialForce );
    
    /*DEBUG_PRINT( "p: %.3f - v: %.3f - f: %.3f (%.3f)", actuator->measuresList[ AXIS_POSITION ], actuator->measuresList[ AXIS_VELOCITY ], 
                                                       actuator->measuresList[ AXIS_FORCE ], actuator->initialForce );*/
    
    //double deformation = sensorMeasuresList[ AXIS_POSITION ] - motorMeasuresList[ AXIS_POSITION ];
    //actuator->measuresList[ AXIS_FORCE ] = Spline3Interp.GetValue( actuator->interactionForceCurve, deformation );
  }
}

#define AXES_NUMBER 2
//const size_t AXES_NUMBER = sizeof(axesList) / sizeof(Axis*);
const char* AXIS_NAMES[ AXES_NUMBER ] = { "motor", "sensor" };
static inline Actuator* LoadActuatorData( const char* configFileName )
{
  //static char searchPath[ FILE_PARSER_MAX_PATH_LENGTH ];
  
  //bool loadError = false;
  
  Actuator* newActuator = (Actuator*) malloc( sizeof(Actuator) );
  memset( newActuator, 0, sizeof(Actuator) );
  
  Axis* axesList[ AXES_NUMBER ] = { &(newActuator->motor), &(newActuator->sensor) };
  
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
  
  axesList[ 0 ]->interface = &AxisCANEPOSOperations;
  axesList[ 0 ]->ID = axesList[ 0 ]->interface->Connect( "Motor Teste" );
  axesList[ 0 ]->gearConversionFactor = 1.0;
  axesList[ 1 ]->interface = &AxisCANEPOSOperations;
  axesList[ 1 ]->ID = axesList[ 0 ]->ID;//axesList[ 1 ]->interface->Connect( "Sensor Teste" );
  axesList[ 1 ]->gearConversionFactor = 1.0;
  
  return newActuator;
}

static inline void UnloadActuatorData( Actuator* actuator )
{
  if( actuator != NULL )
  {
    if( actuator->motor.interface != NULL ) actuator->motor.interface->Disconnect( actuator->motor.ID );
    if( actuator->sensor.interface != NULL ) actuator->sensor.interface->Disconnect( actuator->sensor.ID );
    
    Spline3Interp.UnloadCurve( actuator->interactionForceCurve );
  }
}


#endif // SERIES_ELASTIC_ACTUATOR_H
