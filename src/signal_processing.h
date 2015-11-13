#ifndef SIGNAL_PROCESSING_H
#define SIGNAL_PROCESSING_H

#include <math.h>
#include <stdbool.h>

#include "config_parser.h"

#include "signal_io/interface.h"
#include "plugin_loader.h"
#include "filters.h"

#include "klib/khash.h"

#include "debug/async_debug.h"
#include "debug/data_logging.h"

enum SignalProcessingPhase { SIGNAL_PROCESSING_PHASE_MEASUREMENT, SIGNAL_PROCESSING_PHASE_CALIBRATION, SIGNAL_PROCESSING_PHASE_OFFSET, SIGNAL_PROCESSING_PHASES_NUMBER };



typedef struct _SignalData
{
  double samplingTime;
  double calibrationMax, calibrationMin;
  size_t recordedSamplesCount;
  double samplesMean;
  double processingResult;
  enum SignalProcessingPhase processingPhase;
  bool isRectified, isNormalized;
}
SignalData;

typedef struct _SensorData
{
  SignalIOInterface interface;
  int taskID;
  unsigned int channel;
  SignalData signalData;
  KalmanFilter filter;
  double gain, offset;
  int logID;
}
SensorData;

typedef SensorData* Sensor;


#define SIGNAL_PROCESSING_FUNCTIONS( namespace, function_init ) \
        function_init( Sensor, namespace, InitSensor, const char* ) \
        function_init( void, namespace, EndSensor, Sensor ) \
        function_init( double*, namespace, UpdateSensor, Sensor ) \
        function_init( void, namespace, SetSensorState, Sensor, enum SignalProcessingPhase )

INIT_NAMESPACE_INTERFACE( SignalProcessing, SIGNAL_PROCESSING_FUNCTIONS )


Sensor SignalProcessing_InitSensor( const char* configFileName )
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
    GET_PLUGIN_INTERFACE( SIGNAL_IO_FUNCTIONS, parser.GetStringValue( configFileID, "", "aquisition_system.type" ), newSensor->interface, loadSuccess );
    if( loadSuccess )
    {
      newSensor->taskID = newSensor->interface.InitTask( parser.GetStringValue( configFileID, "", "aquisition_system.task" ) );
      if( newSensor->taskID != -1 )
      {
        newSensor->channel = (unsigned int) parser.GetIntegerValue( configFileID, -1, "aquisition_system.channel" );
        loadSuccess = newSensor->interface.AquireInputChannel( newSensor->taskID, newSensor->channel );
        
        newSensor->gain = parser.GetRealValue( configFileID, 1.0, "processing.input_gain.multiplier" );
        newSensor->gain /= parser.GetRealValue( configFileID, 1.0, "processing.input_gain.divisor" );
        newSensor->signalData.isRectified = parser.GetBooleanValue( configFileID, false, "processing.rectified" );
        newSensor->signalData.isNormalized = parser.GetBooleanValue( configFileID, false, "processing.normalized" );
        
        DEBUG_PRINT( "measure properties: rect: %u - norm: %u", newSensor->signalData.isRectified, newSensor->signalData.isNormalized ); 
        
        newSensor->filter = SimpleKalman.CreateFilter( 3, 0.0 );
        
        if( parser.GetBooleanValue( configFileID, false, "log_data" ) )
        {
          newSensor->logID = DataLogging_InitLog( configFileName, 2 * 10 + 3, 1000 ); // ?
          DataLogging_SetDataPrecision( newSensor->logID, 4 );
        }
      }
      else loadSuccess = false;
    }
    else loadSuccess = false;

    parser.UnloadData( configFileID );
    
    if( !loadSuccess )
    {
      SignalProcessing_EndSensor( newSensor );
      return NULL;
    }
  }
  else
    DEBUG_PRINT( "configuration for sensor %s not found", configFileName );
  
  return newSensor;
}

void SignalProcessing_EndSensor( Sensor sensor )
{
  if( sensor == NULL ) return;
  
  sensor->interface.ReleaseInputChannel( sensor->taskID, sensor->channel );
  sensor->interface.EndTask( sensor->taskID );
  
  SimpleKalman.DiscardFilter( sensor->filter );
  
  if( sensor->logID != 0 ) DataLogging_EndLog( sensor->logID );
  
  free( sensor );
}

double* SignalProcessing_UpdateSensor( Sensor sensor )
{
  if( sensor == NULL ) return NULL;
  
  SignalData* data = &(sensor->signalData);
  
  double* sensorOutput = NULL;
  
  DEBUG_PRINT( "updating sensor channel %d-%u", sensor->taskID, sensor->channel );
  
  double signal;
  if( sensor->interface.Read( sensor->taskID, sensor->channel, &signal ) )
  {
    DEBUG_PRINT( "channel %u value: %g", sensor->channel, signal );
    
    signal *= sensor->gain;
    
    if( data->processingPhase == SIGNAL_PROCESSING_PHASE_OFFSET )
    {
      data->samplesMean += signal;
      data->recordedSamplesCount++;
    }
    else
    {
      signal -= sensor->offset;

      if( data->isRectified ) signal = fabs( signal );

      if( data->processingPhase == SIGNAL_PROCESSING_PHASE_CALIBRATION )
      {
        if( signal > data->calibrationMax ) data->calibrationMax = signal;
        else if( signal < data->calibrationMin ) data->calibrationMin = signal;
      }
      else if( data->processingPhase == SIGNAL_PROCESSING_PHASE_MEASUREMENT )
      {
        if( data->isNormalized && ( data->calibrationMin != data->calibrationMax ) )
        {
          if( signal > data->calibrationMax ) signal = data->calibrationMax;
          else if( signal < data->calibrationMin ) signal = data->calibrationMin;

          signal = signal / ( data->calibrationMax - data->calibrationMin );
        }

        double deltaTime = Timing.GetExecTimeSeconds() - data->samplingTime;
        sensorOutput = SimpleKalman.Update( sensor->filter, signal, deltaTime );
      }
    }
  }
  
  data->samplingTime = Timing.GetExecTimeSeconds();
    
  return sensorOutput;
}

void SignalProcessing_SetSensorState( Sensor sensor, enum SignalProcessingPhase newProcessingPhase )
{
  if( sensor == NULL ) return;
  
  if( newProcessingPhase < 0 || newProcessingPhase >= SIGNAL_PROCESSING_PHASES_NUMBER ) return;
  
  SignalData* data = &(sensor->signalData);

  if( data->processingPhase == SIGNAL_PROCESSING_PHASE_OFFSET )
  {
    if( data->recordedSamplesCount > 0 ) 
      sensor->offset = data->samplesMean / data->recordedSamplesCount;
  }
  
  if( newProcessingPhase == SIGNAL_PROCESSING_PHASE_CALIBRATION )
  {
    data->calibrationMax = 0.0;
    data->calibrationMin = 0.0;
  }
  else if( newProcessingPhase == SIGNAL_PROCESSING_PHASE_OFFSET )
  {
    data->samplesMean = 0.0;
    data->recordedSamplesCount = 0;
  }

  SimpleKalman.Reset( sensor->filter, 0.0 );
  data->samplingTime = Timing.GetExecTimeSeconds();
  
  data->processingPhase = newProcessingPhase;
}


#endif // SIGNAL_PROCESSING_H
