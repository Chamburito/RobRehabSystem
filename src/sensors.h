#ifndef SENSORS_H
#define SENSORS_H

#include <math.h>
#include <stdbool.h>

#include "config_parser.h"

#include "signal_io/interface.h"
#include "plugin_loader.h"
#include "signal_processing.h"
#include "curve_interpolation.h"

#include "debug/async_debug.h"
#include "debug/data_logging.h"

typedef struct _SensorData SensorData;    
typedef SensorData* Sensor;

struct _SensorData
{
  Sensor reference;
  SignalIOInterface interface;
  int taskID;
  unsigned int channel;
  SignalFilter filter;
  Curve measurementCurve;
  double gain;
  int logID;
};


#define SENSOR_FUNCTIONS( namespace, function_init ) \
        function_init( Sensor, namespace, Init, const char* ) \
        function_init( void, namespace, End, Sensor ) \
        function_init( double*, namespace, Update, Sensor ) \
        function_init( bool, namespace, HasError, Sensor ) \
        function_init( void, namespace, Reset, Sensor ) \
        function_init( void, namespace, SetState, Sensor, enum SignalProcessingPhase )

INIT_NAMESPACE_INTERFACE( Sensors, SENSOR_FUNCTIONS )


Sensor Sensors_Init( const char* configFileName )
{
  DEBUG_PRINT( "Trying to load sensor %s data", configFileName );
  
  Sensor newSensor = NULL;
  
  int configFileID = ConfigParsing.LoadConfigFile( configFileName );
  if( configFileID != PARSED_DATA_INVALID_ID )
  {
    ParserInterface parser = ConfigParsing.GetParser();
    
    newSensor = (Sensor) malloc( sizeof(SensorData) );
    memset( newSensor, 0, sizeof(SensorData) );
    
    bool loadSuccess;
    GET_PLUGIN_INTERFACE( SIGNAL_IO_FUNCTIONS, parser.GetStringValue( configFileID, "", "interface.type" ), newSensor->interface, loadSuccess );
    if( loadSuccess )
    {
      newSensor->taskID = newSensor->interface.InitTask( parser.GetStringValue( configFileID, "", "interface.name" ) );
      if( newSensor->taskID != SIGNAL_IO_TASK_INVALID_ID )
      {
        newSensor->channel = (unsigned int) parser.GetIntegerValue( configFileID, -1, "interface.input_channel" );
        loadSuccess = newSensor->interface.AquireInputChannel( newSensor->taskID, newSensor->channel );
        
        newSensor->gain = parser.GetRealValue( configFileID, 1.0, "input_gain.multiplier" );
        newSensor->gain /= parser.GetRealValue( configFileID, 1.0, "input_gain.divisor" );
        
        uint8_t filterFlags = 0x00;
        if( parser.GetBooleanValue( configFileID, false, "signal_processing.rectified" ) ) filterFlags |= SIGNAL_PROCESSING_RECTIFY;
        if( parser.GetBooleanValue( configFileID, false, "signal_processing.normalized" ) ) filterFlags |= SIGNAL_PROCESSING_NORMALIZE;
        newSensor->filter = SignalProcessing.CreateFilter( filterFlags );
        
        newSensor->measurementCurve = CurveInterpolation.LoadCurveString( parser.GetStringValue( configFileID, NULL, "conversion_curve" ) );
        
        if( parser.GetBooleanValue( configFileID, false, "log_data" ) )
        {
          newSensor->logID = DataLogging_InitLog( configFileName, 2 * 10 + 3, 1000 ); // ?
          DataLogging_SetDataPrecision( newSensor->logID, 4 );
        }
        
        char* referenceName = parser.GetStringValue( configFileID, "", "relative_to" );
        if( strcmp( referenceName, configFileName ) != 0 && strcmp( referenceName, "" ) != 0 ) newSensor->reference = Sensors_Init( referenceName );
        
        newSensor->interface.Reset( newSensor->taskID );
      }
      else loadSuccess = false;
    }

    parser.UnloadData( configFileID );
    
    if( !loadSuccess )
    {
      Sensors_End( newSensor );
      return NULL;
    }
  }
  else
    DEBUG_PRINT( "configuration for sensor %s not found", configFileName );
  
  return newSensor;
}

void Sensors_End( Sensor sensor )
{
  if( sensor == NULL ) return;
  
  sensor->interface.ReleaseInputChannel( sensor->taskID, sensor->channel );
  sensor->interface.EndTask( sensor->taskID );
  
  SignalProcessing.DiscardFilter( sensor->filter );
  CurveInterpolation.UnloadCurve( sensor->measurementCurve );
  
  if( sensor->logID != 0 ) DataLogging_EndLog( sensor->logID );
  
  Sensors_End( sensor->reference );
  
  free( sensor );
}

double* Sensors_Update( Sensor sensor )
{
  if( sensor == NULL ) return NULL;
  
  double* sensorOutput = NULL;
  
  //DEBUG_PRINT( "updating sensor channel %d-%u", sensor->taskID, sensor->channel );
  
  double signal;
  if( sensor->interface.Read( sensor->taskID, sensor->channel, &signal ) )
  {
    sensorOutput = SignalProcessing.UpdateFilter( sensor->filter, signal * sensor->gain );
    
    double* referenceOutput = Sensors_Update( sensor->reference );
    if( referenceOutput != NULL ) sensorOutput[ 0 ] -= referenceOutput[ 0 ];

    sensorOutput[ 0 ] = CurveInterpolation.GetValue( sensor->measurementCurve, sensorOutput[ 0 ], sensorOutput[ 0 ] );
  }
    
  return sensorOutput;
}

bool Sensors_HasError( Sensor sensor )
{
  if( sensor == NULL ) return false;
  
  return sensor->interface.HasError( sensor->taskID );
}

void Sensors_Reset( Sensor sensor )
{
  if( sensor == NULL ) return;
  
  sensor->interface.Reset( sensor->taskID );
}

void Sensors_SetState( Sensor sensor, enum SignalProcessingPhase newProcessingPhase )
{
  if( sensor == NULL ) return;
  
  SignalProcessing.SetFilterState( sensor->filter, newProcessingPhase );
  Sensors.SetState( sensor->reference, newProcessingPhase );
}


#endif // SENSORS_H
