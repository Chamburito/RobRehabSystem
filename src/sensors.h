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
  double* inputBuffer;
  SignalProcessor processor;
  Curve measurementCurve;
  double gain;
  int logID;
};


#define SENSOR_FUNCTIONS( namespace, function_init ) \
        function_init( Sensor, namespace, Init, const char* ) \
        function_init( void, namespace, End, Sensor ) \
        function_init( double, namespace, Update, Sensor ) \
        function_init( bool, namespace, HasError, Sensor ) \
        function_init( void, namespace, Reset, Sensor ) \
        function_init( void, namespace, SetState, Sensor, enum SignalProcessingPhase ) \
        function_init( SignalProcessor, namespace, GetSignalProcessor, Sensor )

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
        
        size_t maxInputSamplesNumber = newSensor->interface.GetMaxInputSamplesNumber( newSensor->taskID );
        newSensor->inputBuffer = (double*) calloc( maxInputSamplesNumber, sizeof(double) );
        
        newSensor->gain = parser.GetRealValue( configFileID, 1.0, "input_gain.multiplier" );
        newSensor->gain /= parser.GetRealValue( configFileID, 1.0, "input_gain.divisor" );
        
        uint8_t processingFlags = 0x00;
        if( parser.GetBooleanValue( configFileID, false, "signal_processing.rectified" ) ) processingFlags |= SIGNAL_PROCESSING_RECTIFY;
        if( parser.GetBooleanValue( configFileID, false, "signal_processing.normalized" ) ) processingFlags |= SIGNAL_PROCESSING_NORMALIZE;
        if( (newSensor->processor = SignalProcessing.CreateProcessor( processingFlags )) != NULL )
        {
          double relativeCutFrequency = parser.GetRealValue( configFileID, 0.0, "signal_processing.relative_cut_frequency" );
          SignalProcessing.SetMaxFrequency( newSensor->processor, relativeCutFrequency );
        }
        
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
  
  SignalProcessing.DiscardProcessor( sensor->processor );
  CurveInterpolation.UnloadCurve( sensor->measurementCurve );
  
  free( sensor->inputBuffer );
  
  if( sensor->logID != 0 ) DataLogging_EndLog( sensor->logID );
  
  Sensors_End( sensor->reference );
  
  free( sensor );
}

double Sensors_Update( Sensor sensor )
{
  if( sensor == NULL ) return 0.0;
  
  double sensorOutput = 0.0;
  
  size_t aquiredSamplesNumber = sensor->interface.Read( sensor->taskID, sensor->channel, sensor->inputBuffer );
  
  if( aquiredSamplesNumber > 0 )
  {
    sensorOutput = sensor->inputBuffer[ 0 ] * sensor->gain;
    
    //for( size_t sampleIndex = 0; sampleIndex < aquiredSamplesNumber; sampleIndex++ )
    //  sensorOutput = SignalProcessing.UpdateSignal( sensor->processor, sensor->inputBuffer[ sampleIndex ] * sensor->gain );
    
    double referenceOutput = Sensors_Update( sensor->reference );
    sensorOutput -= referenceOutput;

    sensorOutput = CurveInterpolation.GetValue( sensor->measurementCurve, sensorOutput, sensorOutput );
  }
  
  return sensorOutput;
}

inline bool Sensors_HasError( Sensor sensor )
{
  if( sensor == NULL ) return false;
  
  return sensor->interface.HasError( sensor->taskID );
}

inline void Sensors_Reset( Sensor sensor )
{
  if( sensor == NULL ) return;
  
  sensor->interface.Reset( sensor->taskID );
}

inline void Sensors_SetState( Sensor sensor, enum SignalProcessingPhase newProcessingPhase )
{
  if( sensor == NULL ) return;
  
  SignalProcessing.SetProcessorState( sensor->processor, newProcessingPhase );
  Sensors.SetState( sensor->reference, newProcessingPhase );
}

inline SignalProcessor Sensors_GetSignalProcessor( Sensor sensor )
{
  if( sensor == NULL ) return NULL;
  
  return sensor->processor;
}

#endif // SENSORS_H
