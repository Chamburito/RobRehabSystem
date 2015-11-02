#ifndef AXIS_SENSOR_H
#define AXIS_SENSOR_H

#ifdef __cplusplus
    extern "C" {
#endif

#include "axis/axis_interface.h"
#include "curve_interpolation.h"
      
#include "config_parser.h"
#include "plugin_loader.h"

#include "debug/async_debug.h"
      
typedef struct _AxisSensorData
{
  int interfaceID;
  AxisInterface interface;
  double rawMeasuresList[ AXIS_VARS_NUMBER ];
  size_t measureIndex;
  Curve measureConversionCurve;
  double inputGain, inputOffset;
  bool isAbsolute;
}
AxisSensorData;

typedef AxisSensorData* AxisSensor;


#define AXIS_SENSOR_FUNCTIONS( namespace, function_init ) \
        function_init( AxisSensor, namespace, Init, const char* ) \
        function_init( void, namespace, End, AxisSensor ) \
        function_init( void, namespace, Reset, AxisSensor ) \
        function_init( void, namespace, SetOffset, AxisSensor ) \
        function_init( bool, namespace, HasError, AxisSensor ) \
        function_init( double, namespace, Read, AxisSensor, double* )

INIT_NAMESPACE_INTERFACE( AxisSensors, AXIS_SENSOR_FUNCTIONS )

//static inline void ReadRawMeasures( AxisSensor sensor );

AxisSensor AxisSensors_Init( const char* configFileName )
{
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "Initializing Axis Sensor %s", configFileName );
  
  AxisSensor newSensor = NULL;
  
  if( ConfigParsing.IsAvailable() )
  {
    ParserInterface parser = ConfigParsing.GetParser();
    
    bool loadSuccess = true;
  
    newSensor = (AxisSensor) malloc( sizeof(AxisSensorData) );
    memset( newSensor, 0, sizeof(AxisSensorData) );
  
    int configFileID = parser.LoadFileData( configFileName );
    if( configFileID != PARSED_DATA_INVALID_ID )
    {
      GET_PLUGIN_INTERFACE( AXIS_INTERFACE_FUNCTIONS, parser.GetStringValue( configFileID, "", "interface.type" ), newSensor->interface, loadSuccess );
      if( pluginLoaded )
      {
        newSensor->interfaceID = newSensor->interface.Connect( (unsigned int) parser.GetIntegerValue( configFileID, AXIS_INVALID_ID, "interface.id" ) );
        if( newSensor->interfaceID == AXIS_INVALID_ID ) loadSuccess = false;
      }
    
      newSensor->isAbsolute = parser.GetBooleanValue( configFileID, true, "absolute" );
    
      newSensor->inputGain = parser.GetRealValue( configFileID, 1.0, "input_gain.multiplier" );
      newSensor->inputGain /= parser.GetRealValue( configFileID, 1.0, "input_gain.divisor" );
    
      if( parser.HasKey( configFileID, "conversion_curve" ) )
        newSensor->measureConversionCurve = CurveInterpolation.LoadCurveFile( parser.GetStringValue( configFileID, "", "conversion_curve" ) );
    
      parser.UnloadData( configFileID );
      
      if( !loadSuccess )
      {
        AxisSensors_End( newSensor );
        return NULL;
      }
    
      /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "Axis Sensor %s initialized", configFileName );
    }
    else
      DEBUG_PRINT( "configuration file for axis sensor %s not found", configFileName );
  }
  else
    DEBUG_PRINT( "configuration parser for axis sensor %s not available", configFileName );
  
  return newSensor;
}

inline void AxisSensors_End( AxisSensor sensor )
{
  if( sensor == NULL ) return;
  
  sensor->interface.Disconnect( sensor->interfaceID );
  
  CurveInterpolation.UnloadCurve( sensor->measureConversionCurve );
  
  free( sensor );
}

inline void AxisSensors_Reset( AxisSensor sensor )
{
  if( sensor == NULL ) return;
  
  sensor->interface.Reset( sensor->interfaceID );
}

void AxisSensors_SetOffset( AxisSensor sensor )
{
  if( sensor == NULL ) return;
  
  sensor->interface.ReadMeasures( sensor->interfaceID, sensor->rawMeasuresList );
  sensor->inputOffset = sensor->rawMeasuresList[ sensor->measureIndex ];
}

inline static bool AxisSensors_HasError( AxisSensor sensor )
{
  if( sensor == NULL ) return false;
  
  return sensor->interface.HasError( sensor->interfaceID );
}

const double p1 = -5.6853e-024;
const double p2 = 9.5074e-020;
const double p3 = -5.9028e-016;
const double p4 = 1.6529e-012;
const double p5 = -2.0475e-009;
const double p6 = 9.3491e-007;
const double p7 = 0.0021429;
const double p8 = 2.0556;
double AxisSensors_Read( AxisSensor sensor, double* referencesList )
{
  if( sensor == NULL ) return 0.0;
  
  sensor->interface.ReadMeasures( sensor->interfaceID, sensor->rawMeasuresList );
  
  double measure = ( sensor->rawMeasuresList[ sensor->measureIndex ] - sensor->inputOffset ) * sensor->inputGain;
  if( sensor->isAbsolute && referencesList != NULL ) measure -= referencesList[ sensor->measureIndex ];
  
  if( sensor->measureConversionCurve != NULL ) measure = CurveInterpolation.GetValue( sensor->measureConversionCurve, measure, measure );
  
  //DEBUG_PRINT( "sensor input: raw: %.5f - gain: %.5f", input, measure );
  
  //double measure = p1 * pow(input,7) + p2 * pow(input,6) + p3 * pow(input,5) 
  //                 + p4 * pow(input,4) + p5 * pow(input,3) + p6 * pow(input,2) + p7 * input;// + p8;   //mm
  
  return measure;
}

#ifdef __cplusplus
    }
#endif

#endif  // AXIS_SENSOR_H
