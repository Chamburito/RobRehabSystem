#ifndef SERIES_ELASTIC_ACTUATOR_H
#define SERIES_ELASTIC_ACTUATOR_H 

#include "actuator_interface.h"
#include "spline3_interpolation.h"
#include "filters.h"

#include "axis/axis_types.h"

#include "klib/khash.h"
#include "utils/file_parsing/json_parser.h"

#include "debug/async_debug.h"

typedef struct _Actuator
{
  Axis motor, sensor;
  double measuresList[ AXIS_DIMENSIONS_NUMBER ];
  Splined3Curve* interactionForceCurve;
}
Actuator;

KHASH_MAP_INIT_INT( SEA, Actuator )
static khash_t( SEA )* actuatorsList = NULL;

static int Init( const char* );
static void End( int );
static void Enable( int );
static void Disable( int );
static void Reset( int );
static bool IsEnabled( int );
static bool HasError( int );
static bool SetOption( int, const char*, void* );
static bool GetMeasure( int, enum AxisDimensions );
static bool SetSetpoint( int, double );
static void ReadAxes( int );
static void WriteControl( int );
static void SetOperationMode( int, enum AxisDimensions );

static inline Actuator* LoadActuatorData( const char* );
static inline void UnloadActuatorData( Actuator* );

const ActuatorOperations SEActuatorOperations = { .Init = Init, .End = End, .Enable = Enable, .Disable = Disable, .Reset = Reset,
                                                  .IsEnabled = IsEnabled, .HasError = HasError, .ReadAxes = ReadAxes, .GetMeasure = GetMeasure, .SetSetpoint = SetSetpoint };
                                         
int Init( const char* configKey )
{
  DEBUG_PRINT( "trying to create series elastic actuator %s", configKey );
  
  Actuator* ref_newActuatorData = LoadActuatorData( configKey );
  if( ref_newActuatorData == NULL ) return -1;
  
  if( actuatorsList == NULL ) actuatorsList = kh_init( SEA );
  
  int insertionStatus;
  khint_t newActuatorID = kh_put( SEA, actuatorsList, kh_size( actuatorsList ), &insertionStatus );
  if( insertionStatus == -1 ) return -1;
  
  Actuator* newActuator = &(kh_value( actuatorsList, newActuatorID ));
  
  memcpy( newActuator, ref_newActuatorData, sizeof(Actuator) );
  UnloadActuatorData( ref_newActuatorData );
  
  return (int) newActuatorID;
}

static void End( int actuatorID )
{
  if( kh_exist( actuatorsList, actuatorID ) )
  {
    UnloadActuatorData( &(kh_value( actuatorsList, actuatorID )); );
    
    kh_del( SEA, actuatorsList, actuatorID );
    
    if( kh_size( actuatorsList ) == 0 )
    {
      kh_destroy( SEA, actuatorsList );
      actuatorsList = NULL;
    }
  }
}

static void Enable( int actuatorID )
{
  if( kh_exist( actuatorsList, actuatorID ) )
  {
    Actuator* actuator = &(kh_value( actuatorsList, actuatorID ));
    
    actuator->motor.interface->Enable( actuator->motor.ID );
    actuator->sensor.interface->Enable( actuator->sensor.ID );
  }
}

static void Disable( int actuatorID )
{
  if( kh_exist( actuatorsList, actuatorID ) )
  {
    Actuator* actuator = &(kh_value( actuatorsList, actuatorID ));
    
    actuator->motor.interface->Reset( actuator->motor.ID );
    actuator->sensor.interface->Reset( actuator->sensor.ID );
  }
}

static void Reset( int actuatorID )
{
  if( kh_exist( actuatorsList, actuatorID ) )
  {
    Actuator* actuator = &(kh_value( actuatorsList, actuatorID ));
    
    actuator->motor.interface->Reset( actuator->motor.ID );
    actuator->sensor.interface->Reset( actuator->sensor.ID );
  }
}

static bool IsEnabled( int actuatorID )
{
  if( kh_exist( actuatorsList, actuatorID ) )
  {
    Actuator* actuator = &(kh_value( actuatorsList, actuatorID ));
    
    return ( actuator->motor.interface->IsEnabled( actuator->motor.ID ) );
  }
  
  return false;
}

static bool HasError( int actuatorID )
{
  if( kh_exist( actuatorsList, actuatorID ) )
  {
    Actuator* actuator = &(kh_value( actuatorsList, actuatorID ));
    
    if( actuator->motor.interface->HasError( actuator->motor.ID ) ) return true;
    else if( actuator->sensor.interface->IsEnabled( actuator->sensor.ID ) ) return true;
  }
  
  return false;
}

static double GetMeasure( int actuatorID , enum AxisDimensions dimensionIndex )
{
  if( dimensionIndex < 0 && dimensionIndex > AXIS_DIMENSIONS_NUMBER ) return 0.0;
  
  if( kh_exist( actuatorsList, actuatorID ) )
  {
    Actuator* actuator = &(kh_value( actuatorsList, actuatorID ));
    
    return actuator->measuresList[ dimensionIndex ];
  }
  
  return 0.0;
}

static void SetSetpoint( int actuatorID, double setpointValue )
{
  if( kh_exist( actuatorsList, actuatorID ) )
  {
    Actuator* actuator = &(kh_value( actuatorsList, actuatorID ));
    
    actuator->motor.interface->WriteControl( actuator->motor.ID, setpointValue / actuator->motor.gearConversionFactor );
  }
}

static void ReadAxes( int actuatorID )
{
  static double motorMeasuresList[ AXIS_DIMENSIONS_NUMBER ];
  static double sensorMeasuresList[ AXIS_DIMENSIONS_NUMBER ];
  
  if( kh_exist( actuatorsList, actuatorID ) )
  {
    Actuator* actuator = &(kh_value( actuatorsList, actuatorID ));
    
    actuator->motor.interface->ReadMeasures( actuator->motor.ID, motorMeasuresList );
    actuator->sensor.interface->ReadMeasures( actuator->sensor.ID, sensorMeasuresList );
    
    actuator->measuresList[ AXIS_POSITION ] = sensorMeasuresList[ AXIS_POSITION ];
    actuator->measuresList[ AXIS_VELOCITY ] = sensorMeasuresList[ AXIS_VELOCITY ];
    
    double deformation = sensorMeasuresList[ AXIS_POSITION ] - motorMeasuresList[ AXIS_POSITION ];
    actuator->measuresList[ AXIS_FORCE ] = Spline3Interp_GetValue( actuator->interactionForceCurve, deformation );
  }
}


static inline Actuator* LoadActuatorData( const char* configFileName )
{
  static Actuator actuatorData;
  const Axis* AXES_LIST[] = { &(actuatorData.motor), &(actuatorData.sensor) };
  const size_t AXES_NUMBER = sizeof(AXES_LIST) / sizeof(Axis*);
  const char* AXIS_NAMES[ AXES_NUMBER ] = { "motor", "sensor" };
  
  static char searchPath[ FILE_PARSER_MAX_PATH_LENGTH ];
  
  bool loadError = false;
  
  memset( &actuatorData, 0, sizeof(Actuator) );
  
  FileParser parser = JSONParser;
  int configFileID = parser.OpenFile( configFileName );
  if( configFileID != -1 )
  {
    for( size_t axisIndex = 0; axisIndex < AXES_NUMBER; axisIndex++ )
    {
      sprintf( searchPath, "motor_axes.%s", AXIS_NAMES[ axisIndex ] );
      parser.SetBaseKey( searchPath );
      
      if( (AXES_LIST[ axisIndex ]->interface = AxisTypes.GetInterface( parser.GetStringValue( "interface_type" ) )) != NULL )
      {
        if( (AXES_LIST[ axisIndex ]->ID = AXES_LIST[ axisIndex ]->interface->Connect( parser.GetStringValue( "name" ) )) == -1 ) loadError = true;
      }
      else loadError = true;
      
      AXES_LIST[ axisIndex ]->gearConversionFactor = parser.GetRealValue( "gear_reduction" );
    }
    
    parser.SetBaseKey( "" );
    if( (actuatorData.interactionForceCurve = Spline3Interp_LoadCurve( parser.GetStringValue( "spring_force_curve" ) )) == NULL ) loadError = true;
    
    parser.CloseFile( configFileID );
  }
  else
  {
    loadError = true;
    DEBUG_PRINT( "configuration file for series elastic actuator %s not found", configFileName );
  }
  
  if( loadError )
  {
    UnloadActuatorData( &actuatorData );
    return NULL;
  }
  
  return &actuatorData;
}

static inline void UnloadActuatorData( Actuator* actuator )
{
  if( actuator != NULL )
  {
    if( actuator->motor.interface != NULL ) actuator->motor.interface->Disconnect( actuator->motor.ID );
    if( actuator->sensor.interface != NULL ) actuator->sensor.interface->Disconnect( actuator->sensor.ID );
    
    Spline3Interp_UnloadCurve( actuator->interactionForceCurve );
  }
}


#endif // SERIES_ELASTIC_ACTUATOR_H
