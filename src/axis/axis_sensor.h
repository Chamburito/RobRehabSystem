#ifndef AXIS_SENSOR_H
#define AXIS_SENSOR_H

#ifdef __cplusplus
    extern "C" {
#endif

#include "axis/axis_types.h"
#include "spline3_interpolation.h"
      
#include "file_parsing/json_parser.h"

#include "debug/async_debug.h"
      
typedef struct _SensorData
{
  int axisID;
  AxisInterface interface;
  size_t measureIndex;
  Splined3Curve* measureConversionCurve;
  double inputBuffer[ 6 ];
  double inputOffset;
}
SensorData;

typedef SensorData* Sensor;

KHASH_MAP_INIT_INT( SensorInt, Sensor )
static khash_t( SensorInt )* sensorsList = NULL;

/*static AxisSensor AxisSensor_Init( const char* );
static inline void AxisSensor_End( AxisSensor );
static inline void AxisSensor_Reset( AxisSensor );
static void AxisSensor_SetOffset( AxisSensor );
static inline bool AxisSensor_IsEnabled( AxisSensor );
static inline bool AxisSensor_HasError( AxisSensor );
static double AxisSensor_Read( AxisSensor );*/

#define NAMESPACE AxisSensor

#define NAMESPACE_FUNCTIONS( namespace ) \
        NAMESPACE_FUNCTION( int, namespace, Init, const char* ) \
        NAMESPACE_FUNCTION( void, namespace, End, int ) \
        NAMESPACE_FUNCTION( void, namespace, Reset, int ) \
        NAMESPACE_FUNCTION( void, namespace, SetOffset, int ) \
        NAMESPACE_FUNCTION( bool, namespace, HasError, int ) \
        NAMESPACE_FUNCTION( double, namespace, Read, int )

#define NAMESPACE_FUNCTION( rvalue, namespace, name, ... ) static rvalue namespace##_##name( __VA_ARGS__ );
NAMESPACE_FUNCTIONS( NAMESPACE )
#undef NAMESPACE_FUNCTION

#define NAMESPACE_FUNCTION( rvalue, namespace, name, ... ) rvalue (*name)( __VA_ARGS__ );
const struct { NAMESPACE_FUNCTIONS( NAMESPACE ) }
#undef NAMESPACE_FUNCTION
#define NAMESPACE_FUNCTION( rvalue, namespace, name, ... ) .name = namespace##_##name,
NAMESPACE = { NAMESPACE_FUNCTIONS( NAMESPACE ) };
#undef NAMESPACE_FUNCTION

#undef NAMESPACE_FUNCTIONS
#undef NAMESPACE

static inline Sensor LoadSensorData( const char* );
static inline void UnloadSensorData( Sensor );

//static inline void ReadRawMeasures( Sensor sensor );

static int AxisSensor_Init( const char* configFileName )
{
  DEBUG_EVENT( 0, "Initializing Axis Sensor %s", configFileName );
  
  if( sensorsList == NULL ) sensorsList = kh_init( SensorInt );
  
  int configKey = (int) kh_str_hash_func( configFileName );
  
  int insertionStatus;
  khint_t newSensorID = kh_put( SensorInt, sensorsList, configKey, &insertionStatus );
  if( insertionStatus > 0 )
  {
    kh_value( sensorsList, newSensorID ) = LoadSensorData( configFileName );
    if( kh_value( sensorsList, newSensorID ) == NULL )
    {
      AxisSensor_End( (int) newSensorID );
      return -1;
    }
  }
  
  DEBUG_EVENT( 0, "Axis Sensor %s initialized (iterator: %d)", configFileName, (int) newSensorID );
  
  return (int) newSensorID;
}

static void AxisSensor_End( int sensorID )
{
  if( !kh_exist( sensorsList, (khint_t) sensorID ) ) return;
  
  Sensor sensor = kh_value( sensorsList, (khint_t) sensorID );
  
  UnloadSensorData( sensor );
    
  kh_del( SensorInt, sensorsList, (khint_t) sensorID );
    
  if( kh_size( sensorsList ) == 0 )
  {
    kh_destroy( SensorInt, sensorsList );
    sensorsList = NULL;
  }
}

static void AxisSensor_Reset( int sensorID )
{
  if( !kh_exist( sensorsList, (khint_t) sensorID ) ) return;
  
  Sensor sensor = kh_value( sensorsList, (khint_t) sensorID );
  
  sensor->interface->Reset( sensor->axisID );
}

static void AxisSensor_SetOffset( int sensorID )
{
  static double rawMeasuresList[ AXIS_VARS_NUMBER ];
  
  if( !kh_exist( sensorsList, (khint_t) sensorID ) ) return;
  
  Sensor sensor = kh_value( sensorsList, (khint_t) sensorID );
  
  sensor->interface->ReadMeasures( sensor->axisID, rawMeasuresList );
  
  sensor->inputBuffer[5] = sensor->inputBuffer[4];
  sensor->inputBuffer[4] = sensor->inputBuffer[3];
  sensor->inputBuffer[3] = sensor->inputBuffer[2];
  sensor->inputBuffer[2] = sensor->inputBuffer[1];
  sensor->inputBuffer[1] = sensor->inputBuffer[0];
  sensor->inputBuffer[0] = rawMeasuresList[ sensor->measureIndex ];
  
  sensor->inputOffset = (sensor->inputBuffer[0] + sensor->inputBuffer[1] + sensor->inputBuffer[2] + sensor->inputBuffer[3]+ sensor->inputBuffer[4] + sensor->inputBuffer[5])/6;
}

static bool AxisSensor_HasError( int sensorID )
{
  if( !kh_exist( sensorsList, (khint_t) sensorID ) ) return false;
  
  Sensor sensor = kh_value( sensorsList, (khint_t) sensorID );
  
  return sensor->interface->HasError( sensor->axisID );
}

const double p1 = -5.6853e-024;
const double p2 = 9.5074e-020;
const double p3 = -5.9028e-016;
const double p4 = 1.6529e-012;
const double p5 = -2.0475e-009;
const double p6 = 9.3491e-007;
const double p7 = 0.0021429;
const double p8 = 2.0556;
static double AxisSensor_Read( int sensorID )
{
  static double rawMeasuresList[ AXIS_VARS_NUMBER ];
  
  if( !kh_exist( sensorsList, (khint_t) sensorID ) ) return 0.0;
  
  Sensor sensor = kh_value( sensorsList, (khint_t) sensorID );
  
  sensor->interface->ReadMeasures( sensor->axisID, rawMeasuresList );
  
  sensor->inputBuffer[5] = sensor->inputBuffer[4];
  sensor->inputBuffer[4] = sensor->inputBuffer[3];
  sensor->inputBuffer[3] = sensor->inputBuffer[2];
  sensor->inputBuffer[2] = sensor->inputBuffer[1];
  sensor->inputBuffer[1] = sensor->inputBuffer[0];
  sensor->inputBuffer[0] = rawMeasuresList[ sensor->measureIndex ];
  
  double input = (sensor->inputBuffer[0] + sensor->inputBuffer[1] + sensor->inputBuffer[2] + sensor->inputBuffer[3]+ sensor->inputBuffer[4] + sensor->inputBuffer[5])/6 - sensor->inputOffset;
  
  //double input = rawMeasuresList[ sensor->measureIndex ] - sensor->inputOffset;
  
  //DEBUG_PRINT( "sensor input: %.3f (offset: %.3f)", input, sensor->inputOffset );
    
  double measure = p1 * pow(input,7) + p2 * pow(input,6) + p3 * pow(input,5) 
                   + p4 * pow(input,4) + p5 * pow(input,3) + p6 * pow(input,2) + p7 * input;// + p8;   //mm
  
  return measure;
}

static inline Sensor LoadSensorData( const char* configFileName )
{
  Sensor newSensor = (Sensor) malloc( sizeof(SensorData) );
  
  // File Parsing
  
  
  newSensor->interface = &AxisCANEPOSOperations;
  newSensor->axisID = newSensor->interface->Connect( "CAN Sensor Teste" );
  newSensor->measureIndex = AXIS_ANALOG;
  
  return newSensor;
}

static inline void UnloadSensorData( Sensor sensor )
{
  if( sensor == NULL ) return;
  
  sensor->interface->Disconnect( sensor->axisID );
  
  free( sensor );
}

#ifdef __cplusplus
    }
#endif

#endif  // AXIS_SENSOR_H
