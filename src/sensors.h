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
  DEFINE_INTERFACE_REF( SIGNAL_IO_INTERFACE );
  int taskID;
  unsigned int channel;
  double* inputBuffer;
  size_t maxInputSamplesNumber;
  SignalProcessor processor;
  Curve measurementCurve;
  Sensor reference;
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


char filePath[ PARSER_MAX_FILE_PATH_LENGTH ];
Sensor Sensors_Init( const char* configFileName )
{
  DEBUG_PRINT( "Trying to load sensor %s data", configFileName );
  
  Sensor newSensor = NULL;
  
  sprintf( filePath, "sensors/%s", configFileName );
  int configFileID = ConfigParsing.LoadConfigFile( filePath );
  if( configFileID != PARSED_DATA_INVALID_ID )
  {
    ParserInterface parser = ConfigParsing.GetParser();
    
    newSensor = (Sensor) malloc( sizeof(SensorData) );
    memset( newSensor, 0, sizeof(SensorData) );
    
    bool loadSuccess;
    sprintf( filePath, "signal_io/%s", parser.GetStringValue( configFileID, "", "input_interface.type" ) );
    GET_PLUGIN_IMPLEMENTATION( SIGNAL_IO_INTERFACE, filePath, newSensor, &loadSuccess );
    if( loadSuccess )
    {
      newSensor->taskID = newSensor->InitTask( parser.GetStringValue( configFileID, "", "input_interface.id" ) );
      if( newSensor->taskID != SIGNAL_IO_TASK_INVALID_ID )
      {
        newSensor->channel = (unsigned int) parser.GetIntegerValue( configFileID, -1, "input_interface.channel" );
        loadSuccess = newSensor->AquireInputChannel( newSensor->taskID, newSensor->channel );
        
        newSensor->maxInputSamplesNumber = newSensor->GetMaxInputSamplesNumber( newSensor->taskID );
        newSensor->inputBuffer = (double*) calloc( newSensor->maxInputSamplesNumber, sizeof(double) );
        
        uint8_t processingFlags = 0x00;
        if( parser.GetBooleanValue( configFileID, false, "signal_processing.rectified" ) ) processingFlags |= SIGNAL_PROCESSING_RECTIFY;
        if( parser.GetBooleanValue( configFileID, false, "signal_processing.normalized" ) ) processingFlags |= SIGNAL_PROCESSING_NORMALIZE;
        newSensor->processor = SignalProcessing.CreateProcessor( processingFlags );

        double inputGain = parser.GetRealValue( configFileID, 1.0, "input_gain.multiplier" );
        inputGain /= parser.GetRealValue( configFileID, 1.0, "input_gain.divisor" );
        SignalProcessing.SetInputGain( newSensor->processor, inputGain );
          
        double relativeCutFrequency = parser.GetRealValue( configFileID, 0.0, "signal_processing.relative_cut_frequency" );
        SignalProcessing.SetMaxFrequency( newSensor->processor, relativeCutFrequency );
        
        newSensor->measurementCurve = CurveInterpolation.LoadCurveString( parser.GetStringValue( configFileID, NULL, "conversion_curve" ) );
        
        newSensor->logID = DATA_LOG_INVALID_ID;
        if( parser.GetBooleanValue( configFileID, false, "log_data" ) )
        {
          sprintf( filePath, "sensors/%s", configFileName );
          newSensor->logID = DataLogging_InitLog( filePath, newSensor->maxInputSamplesNumber + 3, 1000 );
          DataLogging_SetDataPrecision( newSensor->logID, 4 );
        }
        
        char* referenceName = parser.GetStringValue( configFileID, "", "relative_to" );
        if( strcmp( referenceName, configFileName ) != 0 && strcmp( referenceName, "" ) != 0 ) newSensor->reference = Sensors_Init( referenceName );
        
        newSensor->Reset( newSensor->taskID );
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
  
  sensor->ReleaseInputChannel( sensor->taskID, sensor->channel );
  sensor->EndTask( sensor->taskID );
  
  SignalProcessing.DiscardProcessor( sensor->processor );
  CurveInterpolation.UnloadCurve( sensor->measurementCurve );
  
  free( sensor->inputBuffer );
  
  if( sensor->logID != DATA_LOG_INVALID_ID ) DataLogging_EndLog( sensor->logID );
  
  Sensors_End( sensor->reference );
  
  free( sensor );
}

double Sensors_Update( Sensor sensor )
{
  if( sensor == NULL ) return 0.0;
  
  size_t aquiredSamplesNumber = sensor->Read( sensor->taskID, sensor->channel, sensor->inputBuffer );
  
  double sensorOutput = SignalProcessing.UpdateSignal( sensor->processor, sensor->inputBuffer, aquiredSamplesNumber );
  
  //DEBUG_PRINT( "sample: %g - output: %g", sensor->inputBuffer[ 0 ], sensorOutput );
  
  double referenceOutput = Sensors.Update( sensor->reference );
  sensorOutput -= referenceOutput;

  double sensorMeasure = CurveInterpolation.GetValue( sensor->measurementCurve, sensorOutput, sensorOutput );
  
  if( sensor->logID != DATA_LOG_INVALID_ID )
  {
    DataLogging.RegisterList( sensor->logID, sensor->maxInputSamplesNumber, sensor->inputBuffer );
    DataLogging.RegisterValues( sensor->logID, 3, sensorOutput, referenceOutput, sensorMeasure );
  }
  
  return sensorMeasure;
}

inline bool Sensors_HasError( Sensor sensor )
{
  if( sensor == NULL ) return false;
  
  return sensor->HasError( sensor->taskID );
}

inline void Sensors_Reset( Sensor sensor )
{
  if( sensor == NULL ) return;
  
  sensor->Reset( sensor->taskID );
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
