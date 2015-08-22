#ifndef DLA_ACTUATOR_H
#define DLA_ACTUATOR_H

#include "actuator_interface.h"
#include "spline3_interpolation.h"
#include "filters.h"

#include "axis/axis_types.h"

#include "klib/khash.h"
#include "file_parsing/json_parser.h"

#include "debug/async_debug.h"

enum { MOTOR_LEFT, MOTOR_RIGHT, MOTORS_NUMBER };

typedef struct _MotorAxis
{
  AxisInterface interface;
  int axisID;
  double measureConversionFactor;
}
MotorAxis;

typedef struct _Actuator
{
  MotorAxis motorsList[ MOTORS_NUMBER ];
  double momentArm;
  double measuresList[ AXIS_DIMENSIONS_NUMBER ];
  //SimpleKalmanFilter* positionFilter;
}
Actuator;

KHASH_MAP_INIT_INT( DLA, Actuator );
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

const ActuatorMethods DLActuatorMethods = { .Init = Init, .End = End, .Enable = Enable, .Disable = Disable, .Reset = Reset, 
                                            .IsEnabled = IsEnabled, .HasError = HasError, .ReadAxes = ReadAxes, .GetMeasure = GetMeasure, .SetSetpoint = SetSetpoint };
                                         
static int Init( const char* configKey )
{
  DEBUG_PRINT( "trying to create dual linear actuator %s", configKey );
  
  Actuator* ref_actuatorData = LoadActuatorData( configKey );
  
  if( ref_actuatorData == NULL ) return -1;
  
  if( ref_actuatorData->motorsList[ MOTOR_LEFT ].axisID == -1 || ref_actuatorData->motorsList[ MOTOR_RIGHT ].axisID == -1 )
    return -1;
  
  if( actuatorsList == NULL ) actuatorsList = kh_init( DLA );
  
  int insertionStatus;
  khint_t actuatorID = kh_put( DLA, actuatorsList, kh_size( actuatorsList ), &insertionStatus );
  
  if( insertionStatus == -1 ) return -1;
  
  Actuator* newActuator = &(kh_value( actuatorsList, actuatorID ));
  
  memcpy( newActuator, ref_actuatorData, sizeof(Actuator) );
  
  return (int) actuatorID;
}

static void End( int actuatorID )
{
  if( kh_exist( actuatorsList, actuatorID ) )
  {
    Actuator* actuator = &(kh_value( actuatorsList, actuatorID ));
    
    for( size_t motorIndex = 0; motorIndex < MOTORS_NUMBER; motorIndex++ )
      actuator->motorsList[ motorIndex ].interface->Disconnect( actuator->motorsList[ motorIndex ].axisID );
    
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
      actuator->motorsList[ motorIndex ].interface->Enable( actuator->motorsList[ motorIndex ].axisID );
  }
}

static void Disable( int actuatorID )
{
  if( kh_exist( actuatorsList, actuatorID ) )
  {
    Actuator* actuator = &(kh_value( actuatorsList, actuatorID ));
    
    for( size_t motorIndex = 0; motorIndex < MOTORS_NUMBER; motorIndex++ )
      actuator->motorsList[ motorIndex ].interface->Disable( actuator->motorsList[ motorIndex ].axisID );
  }
}

static void Reset( int actuatorID )
{
  if( kh_exist( actuatorsList, actuatorID ) )
  {
    Actuator* actuator = &(kh_value( actuatorsList, actuatorID ));
    
    for( size_t motorIndex = 0; motorIndex < MOTORS_NUMBER; motorIndex++ )
      actuator->motorsList[ motorIndex ].interface->Reset( actuator->motorsList[ motorIndex ].axisID );
  }
}

static bool IsEnabled( int actuatorID )
{
  if( kh_exist( actuatorsList, actuatorID ) )
  {
    Actuator* actuator = &(kh_value( actuatorsList, actuatorID ));
    
    for( size_t motorIndex = 0; motorIndex < MOTORS_NUMBER; motorIndex++ )
    {
      if( !( actuator->motorsList[ motorIndex ].interface->IsEnabled( actuator->motorsList[ motorIndex ].axisID ) ) )
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
      if( actuator->motorsList[ motorIndex ].interface->HasError( actuator->motorsList[ motorIndex ].axisID ) )
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
      double motorAxisSetpoint = ( setpointValue / ( 2.0 * actuator->momentArm ) ) / actuator->motorsList[ motorIndex ].measureConversionFactor;
      
      actuator->motorsList[ motorIndex ].interface( actuator->motorsList[ motorIndex ].axisID, motorAxisSetpoint );
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
      actuator->motorsList[ motorIndex ].interface->ReadMeasures( actuator->motorsList[ motorIndex ].axisID, &(measuresTable[ motorIndex ]) );
      resultingDisplacement+= measuresTable[ motorIndex ][ AXIS_POSITION ] * actuator->motorsList[ motorIndex ].measureConversionFactor;
      resultingForce += measuresTable[ motorIndex ][ AXIS_FORCE ] * actuator->motorsList[ motorIndex ].measureConversionFactor;
    }
    
    actuator->measuresList[ AXIS_POSITION ] = atan2( resultingDisplacement / 2.0, actuator->momentArm );
    actuator->measuresList[ AXIS_FORCE ] = resultingForce * actuator->momentArm;
  }
}


const char* motorNames[ MOTORS_NUMBER ] = { "left", "right" };
static inline Actuator* LoadActuatorData( const char* configKey )
{
  static Actuator actuatorData;
  static char searchPath[ FILE_PARSER_MAX_PATH_LENGTH ];
  
  memset( &actuatorData, 0, sizeof(Actuator) );
  
  FileParser parser = JSONParser;
  int configFileID = parser.OpenFile( "dual_linear_actuators" );
  if( configFileID != -1 )
  {
    if( parser.HasKey( configKey ) )
    {
      for( size_t motorIndex = 0; motorIndex < MOTORS_NUMBER; motorIndex++ )
      {
        sprintf( searchPath, "%s.motor_axes.%s", configKey, motorNames[ motorIndex ] );
        parser.SetBaseKey( searchPath );
        
        actuatorData.motorsList[ motorIndex ].interface = AxisTypes.GetInterface( parser.GetStringValue( "interface_type" ) );
        
        actuatorData.motorsList[ motorIndex ].axisID = actuatorData.motorsList[ motorIndex ].interface->Connect( parser.GetStringValue( "name" ) );
        
        double deviationAngle = parser.GetRealValue( "deviation_angle" );
        double measureWeight = parser.GetRealValue( "measure_weight" );
        actuatorData.motorsList[ motorIndex ].measureConversionFactor = cos( deviationAngle ) * measureWeight;
      }
      
      parser.SetBaseKey( configKey );
      actuatorData.momentArm = parser.GetRealValue( "moment_arm" );
      
      return &actuatorData;
    }
    else
    {
      DEBUG_PRINT( "configuration for dual linear actuator %s not found in configuration file", configKey );
      return NULL;
    }
  }
  
  return NULL;
}

#endif // DLA_ACTUATOR_H
