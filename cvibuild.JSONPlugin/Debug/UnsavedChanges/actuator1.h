#ifndef DUAL_LINEAR_ACTUATOR_H
#define DUAL_LINEAR_ACTUATOR_H

#include "actuator_interface.h"
#include "spline3_interpolation.h"
#include "filters.h"

#include "axis/axis_types.h"

#include "klib/khash.h"
#include "file_parsing/json_parser.h"

#include "debug/async_debug.h"

enum { MOTOR_LEFT, MOTOR_RIGHT, MOTORS_NUMBER };

typedef struct _Actuator
{
  Axis motorsList[ MOTORS_NUMBER ];
  double momentArm;
  double measuresList[ AXIS_DIMENSIONS_NUMBER ];
}
Actuator;

KHASH_MAP_INIT_INT( DLA, Actuator )
static khash_t( DLA )* actuatorsList = NULL;

static int Init( const char* );
static void End( int );
static void Enable( int );
static void Disable( int );
static void Reset( int );
static bool IsEnabled( int );
static bool HasError( int );
static void ReadAxes( int );
static double GetMeasure( int, enum AxisDimensions );
static void SetSetpoint( int, double );

static inline Actuator* LoadActuatorData( const char* );
static inline void UnloadActuatorData( Actuator* );

const ActuatorOperations DLActuatorOperations = { .Init = Init, .End = End, .Enable = Enable, .Disable = Disable, .Reset = Reset, 
                                                  .IsEnabled = IsEnabled, .HasError = HasError, .ReadAxes = ReadAxes, .GetMeasure = GetMeasure, .SetSetpoint = SetSetpoint };
                                         
static int Init( const char* configKey )
{
  DEBUG_PRINT( "trying to create dual linear actuator %s", configKey );
  
  Actuator* ref_newActuatorData = LoadActuatorData( configKey ); 
  if( ref_newActuatorData == NULL ) return -1;
  
  if( actuatorsList == NULL ) actuatorsList = kh_init( DLA );
  
  int insertionStatus;
  khint_t newActuatorID = kh_put( DLA, actuatorsList, kh_size( actuatorsList ), &insertionStatus );
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
    
    kh_del( DLA, actuatorsList, actuatorID );
    
    if( kh_size( actuatorsList ) == 0 )
    {
      kh_destroy( DLA, actuatorsList );
      actuatorsList = NULL;
    }
  }
}

static void Enable( int actuatorID )
{
  if( kh_exist( actuatorsList, actuatorID ) )
  {
    Actuator* actuator = &(kh_value( actuatorsList, actuatorID ));
    
    for( size_t motorIndex = 0; motorIndex < MOTORS_NUMBER; motorIndex++ )
      actuator->motorsList[ motorIndex ].interface->Enable( actuator->motorsList[ motorIndex ].ID );
  }
}

static void Disable( int actuatorID )
{
  if( kh_exist( actuatorsList, actuatorID ) )
  {
    Actuator* actuator = &(kh_value( actuatorsList, actuatorID ));
    
    for( size_t motorIndex = 0; motorIndex < MOTORS_NUMBER; motorIndex++ )
      actuator->motorsList[ motorIndex ].interface->Disable( actuator->motorsList[ motorIndex ].ID );
  }
}

static void Reset( int actuatorID )
{
  if( kh_exist( actuatorsList, actuatorID ) )
  {
    Actuator* actuator = &(kh_value( actuatorsList, actuatorID ));
    
    for( size_t motorIndex = 0; motorIndex < MOTORS_NUMBER; motorIndex++ )
      actuator->motorsList[ motorIndex ].interface->Reset( actuator->motorsList[ motorIndex ].ID );
  }
}

static bool IsEnabled( int actuatorID )
{
  if( kh_exist( actuatorsList, actuatorID ) )
  {
    Actuator* actuator = &(kh_value( actuatorsList, actuatorID ));
    
    for( size_t motorIndex = 0; motorIndex < MOTORS_NUMBER; motorIndex++ )
    {
      if( !( actuator->motorsList[ motorIndex ].interface->IsEnabled( actuator->motorsList[ motorIndex ].ID ) ) )
        return false;
    }
    
    return true;
  }
  
  return false;
}

static bool HasError( int actuatorID )
{
  if( kh_exist( actuatorsList, actuatorID ) )
  {
    Actuator* actuator = &(kh_value( actuatorsList, actuatorID ));
    
    for( size_t motorIndex = 0; motorIndex < MOTORS_NUMBER; motorIndex++ )
    {
      if( actuator->motorsList[ motorIndex ].interface->HasError( actuator->motorsList[ motorIndex ].ID ) )
        return true;
    }
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
    
    for( size_t motorIndex = 0; motorIndex < MOTORS_NUMBER; motorIndex++ )
    {
      double motorAxisSetpoint = ( setpointValue / ( 2.0 * actuator->momentArm ) ) / actuator->motorsList[ motorIndex ].gearConversionFactor;
      
      actuator->motorsList[ motorIndex ].interface->WriteControl( actuator->motorsList[ motorIndex ].ID, motorAxisSetpoint );
    }
  }
}

static void ReadAxes( int actuatorID )
{
  static double measuresTable[ MOTORS_NUMBER ][ AXIS_DIMENSIONS_NUMBER ];
  
  if( kh_exist( actuatorsList, actuatorID ) )
  {
    Actuator* actuator = &(kh_value( actuatorsList, actuatorID ));
    
    double resultingDisplacement = 0.0, resultingForce = 0.0;
    
    for( size_t motorIndex = 0; motorIndex < MOTORS_NUMBER; motorIndex++ )
    {
      actuator->motorsList[ motorIndex ].interface->ReadMeasures( actuator->motorsList[ motorIndex ].ID, &(measuresTable[ motorIndex ]) );
      resultingDisplacement+= measuresTable[ motorIndex ][ AXIS_POSITION ] * actuator->motorsList[ motorIndex ].gearConversionFactor;
      resultingForce += measuresTable[ motorIndex ][ AXIS_FORCE ] * actuator->motorsList[ motorIndex ].gearConversionFactor;
    }
    
    actuator->measuresList[ AXIS_POSITION ] = atan2( resultingDisplacement / 2.0, actuator->momentArm );
    actuator->measuresList[ AXIS_FORCE ] = resultingForce * actuator->momentArm;
  }
}


const char* MOTOR_NAMES[ MOTORS_NUMBER ] = { "left", "right" };
static inline Actuator* LoadActuatorData( const char* configFileName )
{
  static Actuator actuatorData;
  static char searchPath[ PARSER_MAX_FILE_PATH_LENGTH ];
  
  bool loadError = false;
  
  memset( &actuatorData, 0, sizeof(Actuator) );
  
  FileParser parser = JSONParser;
  int configFileID = parser.OpenFile( configFileName );
  if( configFileID != -1 )
  {
    for( size_t motorIndex = 0; motorIndex < MOTORS_NUMBER; motorIndex++ )
    {
      sprintf( searchPath, "motor_axes.%s", MOTOR_NAMES[ motorIndex ] );
      parser.SetBaseKey( searchPath );
      
      if( (actuatorData.motorsList[ motorIndex ].interface = AxisTypes.GetInterface( parser.GetStringValue( "interface_type" ) )) != NULL )
      {
        actuatorData.motorsList[ motorIndex ].ID = actuatorData.motorsList[ motorIndex ].interface->Connect( parser.GetStringValue( "name" ) );
        if( actuatorData.motorsList[ motorIndex ].ID == -1 ) loadError = true;
      }
      else loadError = true;
      
      double deviationAngle = parser.GetRealValue( "deviation_angle" );
      double measureWeight = parser.GetRealValue( "measure_weight" );
      if( (actuatorData.motorsList[ motorIndex ].gearConversionFactor = cos( deviationAngle ) * measureWeight) == 0.0 ) loadError = true;
    }
    
    parser.SetBaseKey( "" );
    if( (actuatorData.momentArm = parser.GetRealValue( "moment_arm" )) == 0.0 ) loadError = true;
    
    parser.CloseFile( configFileID );
  }
  
  DEBUG_PRINT( "configuration file for dual linear actuator %s not found", configFileName );
  
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
    for( size_t motorIndex = 0; motorIndex < MOTORS_NUMBER; motorIndex++ )
    {
      if( actuator->motorsList[ motorIndex ].interface != NULL ) 
        actuator->motorsList[ motorIndex ].interface->Disconnect( actuator->motorsList[ motorIndex ].ID );
    }
  }
}

#endif // DUAL_LINEAR_ACTUATOR_H
