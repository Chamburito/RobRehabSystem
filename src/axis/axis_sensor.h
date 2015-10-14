#ifndef AXIS_SENSOR_H
#define AXIS_SENSOR_H

#ifdef __cplusplus
    extern "C" {
#endif

#include "axis/axis_types.h"
#include "spline3_interpolation.h"
      
#include "config_parser.h"
#include "plugin_loader.h"

#include "debug/async_debug.h"
      
typedef struct _AxisSensorData
{
  int axisID;
  AxisInterface interface;
  size_t measureIndex;
  Splined3Curve measureConversionCurve;
  double inputOffset;
  bool absolute;
}
AxisSensorData;

typedef AxisSensorData* AxisSensor;


#define AXIS_SENSOR_FUNCTIONS( namespace, function_init ) \
        function_init( AxisSensor, namespace, Init, const char* ) \
        function_init( void, namespace, End, AxisSensor ) \
        function_init( void, namespace, Reset, AxisSensor ) \
        function_init( void, namespace, SetOffset, AxisSensor ) \
        function_init( bool, namespace, HasError, AxisSensor ) \
        function_init( double, namespace, Read, AxisSensor )

INIT_NAMESPACE_INTERFACE( AxisSensors, AXIS_SENSOR_FUNCTIONS )

static AxisSensor LoadAxisSensorData( const char* );
static void UnloadAxisSensorData( AxisSensor );

//static inline void ReadRawMeasures( AxisSensor sensor );

AxisSensor AxisSensors_Init( const char* configFileName )
{
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "Initializing Axis Sensor %s", configFileName );
  
  AxisSensor newSensor = (AxisSensor) malloc( sizeof(AxisSensorData) );
  
  // File Parsing
  
  
  newSensor->interface = &AxisCANEPOSOperations;
  newSensor->axisID = newSensor->interface->Connect( 2 );
  //newSensor->measureIndex = AXIS_ANALOG;
  newSensor->measureIndex = AXIS_POSITION;
  
  newSensor->measureConversionCurve = Spline3Interp.LoadCurve( NULL );
  Spline3Interp.SetScale( newSensor->measureConversionCurve, -( 1.0/*2 * M_PI*/ ) / 2000.0 );
  
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "Axis Sensor %s initialized", configFileName );
  
  return newSensor;
}

inline void AxisSensors_End( AxisSensor sensor )
{
  if( sensor == NULL ) return;
  
  sensor->interface->Disconnect( sensor->axisID );
  
  free( sensor );
}

inline void AxisSensors_Reset( AxisSensor sensor )
{
  if( sensor == NULL ) return;
  
  sensor->interface->Reset( sensor->axisID );
}

void AxisSensors_SetOffset( AxisSensor sensor )
{
  static double rawMeasuresList[ AXIS_VARS_NUMBER ];
  
  if( sensor == NULL ) return;
  
  if( sensor->absolute )
  {
    sensor->interface->ReadMeasures( sensor->axisID, rawMeasuresList );
    sensor->inputOffset = rawMeasuresList[ sensor->measureIndex ];
  }
}

inline static bool AxisSensors_HasError( AxisSensor sensor )
{
  if( sensor == NULL ) return false;
  
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
double AxisSensors_Read( AxisSensor sensor )
{
  static double rawMeasuresList[ AXIS_VARS_NUMBER ];
  
  if( sensor == NULL ) return 0.0;
  
  sensor->interface->ReadMeasures( sensor->axisID, rawMeasuresList );
  
  double input = rawMeasuresList[ sensor->measureIndex ];
  if( sensor->absolute ) input -= sensor->inputOffset;
  
  double measure = - 2 * M_PI * input / 2000.0;//Spline3Interp.GetValue( sensor->measureConversionCurve, input );
  
  //DEBUG_PRINT( "sensor input: raw: %.5f - gain: %.5f", input, measure );
  
  //double measure = p1 * pow(input,7) + p2 * pow(input,6) + p3 * pow(input,5) 
  //                 + p4 * pow(input,4) + p5 * pow(input,3) + p6 * pow(input,2) + p7 * input;// + p8;   //mm
  
  return measure;
}


static AxisSensor LoadAxisSensorData( const char* configFileName )
{
  static char searchPath[ FILE_PARSER_MAX_PATH_LENGTH ];
  
  bool loadError = false;
  
  AxisSensor newSensor = (AxisSensor) malloc( sizeof(AxisSensorData) );
  memset( newSensor, 0, sizeof(AxisSensorData) );
  
  newSensor->axisID = -1;
  
  int configFileID = ConfigParser.LoadFile( configFileName );
  if( configFileID != INVALID_FILE_ID )
  {
    bool pluginLoaded;
    GET_PLUGIN_INTERFACE( AXIS_INTERFACE_FUNCTIONS, ConfigParser.GetStringValue( configFileID, "interface.type" ), newSensor->interface, pluginLoaded );
    if( pluginLoaded )
    {
      if( ConfigParser.HasKey( configFileID, "interface.id" ) )
        newSensor->axisID = newSensor->interface.Connect( (unsigned int) ConfigParser.GetIntegerValue( configFileID, "interface.id" ) );
      
      if( newSensor->axisID ) loadError = true;
      
      newSensor->measureIndex = (size_t) ConfigParser.GetIntegerValue( configFileID, "interface.channel" );
    }
    else loadError = true;
    
    newSensor->absolute = ConfigParser.GetBooleanValue( configFileID, "is_absolute" );
    
    if( ConfigParser.HasKey( configFileID, "conversion.curve" ) )
      newSensor->measureConversionCurve = Spline3Interp.LoadCurve( ConfigParser.GetStringValue( configFileID, "conversion.curve" ) );
    else
      newSensor->measureConversionCurve = Spline3Interp.LoadCurve( NULL );
    
    if( ConfigParser.HasKey( configFileID, "conversion.factor" ) )
      Spline3Interp.SetScale( newSensor->measureConversionCurve, ConfigParser.GetRealValue( configFileID, "conversion.factor" ) );
    
    ConfigParser.UnloadFile( configFileID );
  }
  else
  {
    loadError = true;
    DEBUG_PRINT( "configuration file for series elastic actuator %s not found", configFileName );
  }
  
  if( loadError )
  {
    UnloadAxisSensorData( newSensor );
    return NULL;
  }
}

static void UnloadAxisSensorData( AxisSensor sensor )
{
  
}

#ifdef __cplusplus
    }
#endif

#endif  // AXIS_SENSOR_H
