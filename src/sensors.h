#ifndef SENSORS_H
#define SENSORS_H

#include <math.h>
#include <stdbool.h>

#include "config_parser.h"

#include "signal_io/interface.h"
#include "plugin_loader.h"
#include "filters.h"

#include "klib/khash.h"

#include "debug/async_debug.h"
#include "debug/data_logging.h"

enum SignalProcessingPhase { SENSORS_PHASE_MEASUREMENT, SENSORS_PHASE_CALIBRATION, SENSORS_PHASE_OFFSET, SENSORS_PHASES_NUMBER };

const size_t FILTER_ORDER = 3;
const double filter_A[ FILTER_ORDER ] = { 1.0, -1.9964, 0.9965 };
const double filter_B[ FILTER_ORDER ] = { 1.576e-6, 3.153e-6, 1.576e-6 };
const int FILTER_EXTRA_SAMPLES_NUMBER = FILTER_ORDER - 1;

typedef struct _SignalData
{
  double* samplesBuffer;
  double* filteredSamplesBuffer;
  size_t samplesBufferLength;
  double calibrationMax, calibrationMin;
  size_t recordedSamplesCount;
  double samplesMean;
  double processingResult;
  enum SignalProcessingPhase processingPhase;
  bool isRectified, isFiltered, isNormalized;
}
SignalData;

typedef struct _SensorData
{
  SignalAquisitionInterface interface;
  int taskID;
  unsigned int channel;
  SignalData signalData;
  double scalingFactor, offset;
  int logID;
}
SensorData;

typedef SensorData* Sensor;

KHASH_MAP_INIT_INT( SensorInt, Sensor )
static khash_t( SensorInt )* sensorsList = NULL;

static bool isProcessing;
Thread processingThreadID;
ThreadLock processingLock;


#define SENSORS_FUNCTIONS( namespace, function_init ) \
        function_init( int, namespace, InitSensor, const char* ) \
        function_init( void, namespace, EndSensor, int ) \
        function_init( double, namespace, GetProcessedSignal, int ) \
        function_init( void, namespace, ChangePhase, int, enum SignalProcessingPhase )

INIT_NAMESPACE_INTERFACE( SignalProcessing, SENSORS_FUNCTIONS )


static void* AsyncUpdate( void* );

static Sensor LoadSensorData( const char* );
static void UnloadSensorData( Sensor );

const int SENSOR_INVALID_ID = 0;

int SignalProcessing_InitSensor( const char* configFileName )
{
  Sensor newSensor = LoadSensorData( configFileName );
  if( newSensor == NULL )
  {
    DEBUG_PRINT( "sensor %s configuration failed", configFileName );
    return SENSOR_INVALID_ID;
  }
  
  if( sensorsList == NULL ) 
  {
    sensorsList = kh_init( SensorInt );
    
    processingLock = ThreadLocks.Create();
    
    isProcessing = true;
    processingThreadID = Threading.StartThread( AsyncUpdate, NULL, THREAD_JOINABLE );
  }
  
  int configKey = (int) kh_str_hash_func( configFileName );
  
  int insertionStatus;
  khint_t newSensorIndex = kh_put( SensorInt, sensorsList, configKey, &insertionStatus );
  if( insertionStatus > 0 )
  {
    kh_value( sensorsList, newSensorIndex ) = newSensor;
    
    DEBUG_PRINT( "new key %d inserted (iterator: %u - total: %u)", kh_key( sensorsList, newSensorIndex ), newSensorIndex, kh_size( sensorsList ) );
  }
  
  return (int) kh_key( sensorsList, newSensorIndex );
}

void SignalProcessing_EndSensor( int sensorID )
{
  khint_t sensorIndex = kh_get( SensorInt, sensorsList, (khint_t) sensorID );
  if( sensorIndex == kh_end( sensorsList ) ) return;
  
  ThreadLocks.Aquire( processingLock );
  
  UnloadSensorData( kh_value( sensorsList, sensorIndex ) );
  kh_del( SensorInt, sensorsList, sensorIndex );
  
  ThreadLocks.Release( processingLock );
  
  if( kh_size( sensorsList ) == 0 )
  {
    isProcessing = false;
    (void) Threading.WaitExit( processingThreadID, 5000 );
    
    ThreadLocks.Discard( processingLock );
    processingLock = NULL;
    
    if( sensorsList != NULL )
    {
      kh_destroy( SensorInt, sensorsList );
      sensorsList = NULL;
    }
  }
}

double SignalProcessing_GetProcessedSignal( int sensorID )
{
  khint_t sensorIndex = kh_get( SensorInt, sensorsList, (khint_t) sensorID );
  if( sensorIndex == kh_end( sensorsList ) ) return 0.0;

  Sensor sensor = kh_value( sensorsList, sensorIndex );
    
  return sensor->signalData.processingResult;
}

void SignalProcessing_ChangePhase( int sensorID, enum SignalProcessingPhase newProcessingPhase )
{
  khint_t sensorIndex = kh_get( SensorInt, sensorsList, (khint_t) sensorID );
  if( sensorIndex == kh_end( sensorsList ) ) return;
  
  if( newProcessingPhase < 0 || newProcessingPhase >= SENSORS_PHASES_NUMBER ) return;
  
  Sensor sensor = kh_value( sensorsList, sensorIndex );
  
  if( processingLock == NULL ) return;
  
  ThreadLocks.Aquire( processingLock );
  
  SignalData* data = &(sensor->signalData);

  if( data->processingPhase == SENSORS_PHASE_OFFSET )
  {
    if( data->recordedSamplesCount > 0 ) 
      sensor->offset = data->samplesMean / data->recordedSamplesCount;
  }
  
  if( newProcessingPhase == SENSORS_PHASE_CALIBRATION )
  {
    data->calibrationMax = 0.0;
    data->calibrationMin = 0.0;
  }
  else if( newProcessingPhase == SENSORS_PHASE_OFFSET )
  {
    data->samplesMean = 0.0;
    data->recordedSamplesCount = 0;
  }
  
  data->processingPhase = newProcessingPhase;
  
  ThreadLocks.Release( processingLock );
}


static double ProcessSignal( Sensor sensor )
{
  SignalData* data = &(sensor->signalData);
  size_t samplesBufferLength = data->samplesBufferLength;
  
  size_t aquiredSamplesCount;
  double* rawSamplesList = sensor->interface.Read( sensor->taskID, sensor->channel, &aquiredSamplesCount );
  if( rawSamplesList == NULL ) return 0.0;

  long processedSamplesStartIndex = samplesBufferLength - aquiredSamplesCount;
  double* processedSamplesList = data->samplesBuffer + processedSamplesStartIndex;
  
  for( int rawSampleIndex = 0; rawSampleIndex < aquiredSamplesCount; rawSampleIndex++ )
    processedSamplesList[ rawSampleIndex ] = rawSamplesList[ rawSampleIndex ] * sensor->scalingFactor - sensor->offset;
  
  if( data->processingPhase != SENSORS_PHASE_OFFSET )
  {
    if( data->isRectified )
    {
      for( int sampleIndex = 0; sampleIndex < aquiredSamplesCount; sampleIndex++ )
        processedSamplesList[ sampleIndex ] = fabs( processedSamplesList[ sampleIndex ] );
    }

    if( data->isFiltered )
    {
      for( int sampleIndex = (int) processedSamplesStartIndex; sampleIndex < samplesBufferLength; sampleIndex++ )
      {
        data->filteredSamplesBuffer[ sampleIndex ] = 0.0;
        for( int factorIndex = 0; factorIndex < FILTER_ORDER; factorIndex++ )
        {
          data->filteredSamplesBuffer[ sampleIndex ] -= filter_A[ factorIndex ] * data->filteredSamplesBuffer[ sampleIndex - factorIndex ];
          data->filteredSamplesBuffer[ sampleIndex ] += filter_B[ factorIndex ] * data->samplesBuffer[ sampleIndex - factorIndex ];
        }
      }
      
      processedSamplesList = data->filteredSamplesBuffer + processedSamplesStartIndex;
    }

    for( int sampleIndex = 0; sampleIndex < processedSamplesStartIndex; sampleIndex++ )
    {
      data->filteredSamplesBuffer[ sampleIndex ] = data->filteredSamplesBuffer[ aquiredSamplesCount + sampleIndex ];
      data->samplesBuffer[ sampleIndex ] = data->samplesBuffer[ aquiredSamplesCount + sampleIndex ];
    }
  }

  double processedSamplesSum = 0.0;
  for( int sampleIndex = 0; sampleIndex < aquiredSamplesCount; sampleIndex++ )
    processedSamplesSum += processedSamplesList[ sampleIndex ];
  
  return processedSamplesSum / aquiredSamplesCount;
}

static void* AsyncUpdate( void* data )
{
  while( isProcessing )
  {
    ThreadLocks.Aquire( processingLock );
    
    for( khint_t sensorID = kh_begin( sensorsList ); sensorID != kh_end( sensorsList ); sensorID++ )
    {
      if( !kh_exist( sensorsList, sensorID ) ) continue;
      
      Sensor sensor = kh_value( sensorsList, sensorID );
      SignalData* data = &(sensor->signalData);
      
      double processedSignal = ProcessSignal( sensor );
      if( processedSignal != 0.0 )
      {
        //DEBUG_PRINT( "sensor %u read", sensorID );
        
        if( data->processingPhase == SENSORS_PHASE_CALIBRATION )
        {
          if( processedSignal > data->calibrationMax ) data->calibrationMax = processedSignal;
          else if( processedSignal < data->calibrationMin ) data->calibrationMin = processedSignal;
        }
        else if( data->processingPhase == SENSORS_PHASE_OFFSET )
        {
          data->samplesMean += processedSignal;
          data->recordedSamplesCount++;
        }
        else if( data->processingPhase == SENSORS_PHASE_MEASUREMENT )
        {
          if( data->isNormalized && ( data->calibrationMin != data->calibrationMax ) )
          {
            processedSignal = ( processedSignal - data->calibrationMin ) / ( data->calibrationMax - data->calibrationMin );
            if( !data->isRectified ) processedSignal = 2.0 * processedSignal - 1.0;
          }

          data->processingResult = processedSignal;
        }
      }
    }
    
    ThreadLocks.Release( processingLock );
  }
  
  DEBUG_PRINT( "ending processing thread %x", THREAD_ID );
  
  return NULL;
}


static Sensor LoadSensorData( const char* configFileName )
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
    GET_PLUGIN_INTERFACE( SIGNAL_AQUISITION_FUNCTIONS, parser.GetStringValue( configFileID, "", "aquisition_system.type" ), newSensor->interface, loadSuccess );
    if( loadSuccess )
    {
      newSensor->taskID = newSensor->interface.InitTask( parser.GetStringValue( configFileID, "", "aquisition_system.task" ) );
      if( newSensor->taskID != -1 )
      {
        size_t newSamplesBufferMaxLength = newSensor->interface.GetMaxSamplesNumber( newSensor->taskID );
        if( newSamplesBufferMaxLength > 0 )
        {
          newSensor->signalData.samplesBufferLength = newSamplesBufferMaxLength + FILTER_EXTRA_SAMPLES_NUMBER;
          newSensor->signalData.samplesBuffer = (double*) calloc( newSensor->signalData.samplesBufferLength, sizeof(double) );
          newSensor->signalData.filteredSamplesBuffer = (double*) calloc( newSensor->signalData.samplesBufferLength, sizeof(double) );
        }
        else loadSuccess = false;

        newSensor->channel = (unsigned int) parser.GetIntegerValue( configFileID, -1, "aquisition_system.channel" );
        newSensor->interface.AquireChannel( newSensor->taskID, newSensor->channel );
        
        newSensor->scalingFactor = parser.GetRealValue( configFileID, 1.0, "processing.input_gain.multiplier" );
        newSensor->scalingFactor /= parser.GetRealValue( configFileID, 1.0, "processing.input_gain.divisor" );
        newSensor->signalData.isRectified = parser.GetBooleanValue( configFileID, false, "processing.rectified" );
        newSensor->signalData.isFiltered = parser.GetBooleanValue( configFileID, false, "processing.filtered" );
        newSensor->signalData.isNormalized = parser.GetBooleanValue( configFileID, false, "processing.normalized" );
        
        DEBUG_PRINT( "measure properties: rect: %u - filter: %u - norm: %u", newSensor->signalData.isRectified, newSensor->signalData.isFiltered, newSensor->signalData.isNormalized ); 
        
        if( parser.GetBooleanValue( configFileID, false, "log_data" ) )
        {
          newSensor->logID = DataLogging_InitLog( configFileName, 2 * newSamplesBufferMaxLength + 3, 1000 );
          DataLogging_SetDataPrecision( newSensor->logID, 4 );
        }
      }
      else loadSuccess = false;
    }
    else loadSuccess = false;

    parser.UnloadData( configFileID );
    
    if( !loadSuccess )
    {
      UnloadSensorData( newSensor );
      return NULL;
    }
  }
  else
    DEBUG_PRINT( "configuration for sensor %s not found", configFileName );
  
  return newSensor;
}

void UnloadSensorData( Sensor sensor )
{
  if( sensor == NULL ) return;
  
  free( sensor->signalData.samplesBuffer );
  free( sensor->signalData.filteredSamplesBuffer );
  
  sensor->interface.ReleaseChannel( sensor->taskID, sensor->channel );
  sensor->interface.EndTask( sensor->taskID );
  
  if( sensor->logID != 0 ) DataLogging_EndLog( sensor->logID );
  
  free( sensor );
}


#endif // SENSORS_H
