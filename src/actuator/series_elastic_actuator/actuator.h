#ifndef SEA_ACTUATOR_H
#define SEA_ACTUATOR_H 

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
  //SimpleKalmanFilter* positionFilter;
}
Actuator;

KHASH_MAP_INIT_INT( SEA, Actuator );
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

const ActuatorMethods SEActuatorMethods = { .Init = Init, .End = End, .Enable = Enable, .Disable = Disable, .Reset = Reset,
                                            .IsEnabled = IsEnabled, .HasError = HasError, .ReadAxes = ReadAxes, .GetMeasure = GetMeasure, .SetSetpoint = SetSetpoint };
                                         
int Init( const char* options )
{
  kson_node_t* optionNode;
  kson_node_t* subOptionNode;
  
  if( actuatorsList == NULL ) actuatorsList = kh_init( SEA );
  
  int insertionStatus;
  int actuatorID = kh_put( SEA, actuatorsList, kh_size( actuatorsList ), &insertionStatus );
  
  if( insertionStatus == -1 ) return -1;
  //else if( insertionStatus == 0 ) return -1;
  
  Actuator* newActuator = &(kh_value( actuatorsList, actuatorID ));
  
  kson_t* optionsBlock = kson_parse( options );
  
  if( (optionNode = kson_by_key( optionsBlock->root, "motor" )) )
  {   
    if( (subOptionNode = kson_by_key( optionNode, "interface" )) )
    {
      newActuator->motor.interface = &AxisCANEPOSInterface;
      newActuator->motor.interfaceID = newActuator->motor.interface->Connect( kson_by_index( subOptionNode, 1 ) );
    }        
  }
}



static inline Actuator* LoadActuatorData( const char* configKey )
{
  static Actuator actuatorData;
  static char searchPath[ FILE_PARSER_MAX_PATH_LENGTH ];
  
  memset( &actuatorData, 0, sizeof(Actuator) );
  
  FileParser parser = JSONParser;
  int configFileID = parser.OpenFile( "series_elastic_actuators" );
  if( configFileID != -1 )
  {
    if( parser.HasKey( configKey ) )
    {
      parser.SetBaseKey( configKey );
      
      actuatorData.motor.interface = AxisTypes.GetInterface( parser.GetStringValue( "motor.interface_type" ) );  
      actuatorData.motor.interfaceID = actuatorData.motor.interface->Connect( parser.GetStringValue( "motor.name" ) );
        
      actuatorData.sensor.interface = AxisTypes.GetInterface( parser.GetStringValue( "sensor.interface_type" ) );  
      actuatorData.sensor.interfaceID = actuatorData.sensor.interface->Connect( parser.GetStringValue( "sensor.name" ) );
      
      /*double deviationAngle = parser.GetRealValue( "deviation_angle" );
      double measureWeight = parser.GetRealValue( "measure_weight" );
      actuatorData.motorsList[ motorIndex ].measureConversionFactor = cos( deviationAngle ) * measureWeight;
      
      
      actuatorData.momentArm = parser.GetRealValue( "moment_arm" );*/
      
      return &actuatorData;
    }
    else
    {
      DEBUG_PRINT( "configuration for series elastic actuator %s not found in configuration file", configKey );
      return NULL;
    }
  }
  
  return NULL;
}

#endif // SEA_ACTUATOR_H
