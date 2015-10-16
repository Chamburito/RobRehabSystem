#ifndef EMG_PROCESS_H
#define EMG_PROCESS_H

#include <math.h>
#include <stdbool.h>

#include "config_parser.h"

#include "signal_aquisition/signal_aquisition_interface.h"
#include "plugin_loader.h"

#include "klib/khash.h"

#include "debug/async_debug.h"
#include "debug/data_logging.h"

enum EMGProcessPhase { EMG_ACTIVATION_PHASE, EMG_RELAXATION_PHASE, EMG_CONTRACTION_PHASE, EMG_PROCESSING_PHASES_NUMBER };

const size_t OLD_SAMPLES_BUFFER_LEN = 100;

const size_t FILTER_ORDER = 3;
const double filter_A[ FILTER_ORDER ] = { 1.0, -1.9964, 0.9965 };
const double filter_B[ FILTER_ORDER ] = { 1.576e-6, 3.153e-6, 1.576e-6 };
const int FILTER_EXTRA_SAMPLES_NUMBER = FILTER_ORDER - 1;

typedef struct _EMGData
{
  double* samplesBuffer;
  size_t samplesBufferLength, samplesBufferStartIndex;
  double* filteredSamplesBuffer;
  double* rectifiedSamplesBuffer;
  size_t filteredSamplesBufferLength;
  unsigned int processingPhase;
  double processingResultsList[ EMG_PROCESSING_PHASES_NUMBER ];
  unsigned int preparationPassesCount;
}
EMGData;

enum EMGMuscleCurves { MUSCLE_ACTIVE_FORCE, MUSCLE_PASSIVE_FORCE, MUSCLE_MOMENT_ARM, MUSCLE_NORM_LENGTH, MUSCLE_CURVES_NUMBER };
const size_t MUSCLE_CURVE_ORDER = 5;
typedef double MuscleCurve[ MUSCLE_CURVE_ORDER ];

typedef struct _MuscleProperties
{
  MuscleCurve curvesList[ MUSCLE_CURVES_NUMBER ];
  double initialPenationAngle;
  double activationFactor;
  double scaleFactor;
}
MuscleProperties;

typedef struct _EMGSensorData
{
  SignalAquisitionInterface interface;
  int taskID;
  unsigned int channel;
  EMGData emgData;
  MuscleProperties muscleProperties;
  ThreadLock lock;
  bool isReading;
  int logID;
}
EMGSensorData;

typedef EMGSensorData* EMGSensor;

KHASH_MAP_INIT_INT( SensorInt, EMGSensor )
static khash_t( SensorInt )* sensorsList = NULL;

static bool isProcessRunning;
Thread processingThreadID;
ThreadLock phasePassCountLock;

#define EMG_PROCESS_FUNCTIONS( namespace, function_init ) \
        function_init( int, namespace, InitSensor, const char* ) \
        function_init( void, namespace, EndSensor, int ) \
        function_init( double, namespace, GetNormalizedSignal, int ) \
        function_init( double, namespace, GetActivation, int ) \
        function_init( double, namespace, GetMuscleTorque, int, double ) \
        function_init( void, namespace, ChangePhase, int, enum EMGProcessPhase ) \
        function_init( MuscleProperties*, namespace, GetMuscleProperties, int )

INIT_NAMESPACE_INTERFACE( EMGProcessing, EMG_PROCESS_FUNCTIONS )

static void* AsyncUpdate( void* );
static double* GetFilteredSignal( EMGSensor, size_t* );

static EMGSensor LoadEMGSensorData( const char* );
static void UnloadEMGSensorData( EMGSensor );

const int EMG_SENSOR_INVALID_ID = 0;

int EMGProcessing_InitSensor( const char* configFileName )
{
  EMGSensor newSensor = LoadEMGSensorData( configFileName );
  if( newSensor == NULL )
  {
    DEBUG_PRINT( "EMG sensor %s configuration failed", configFileName );
    return EMG_SENSOR_INVALID_ID;
  }
  
  if( sensorsList == NULL ) 
  {
    sensorsList = kh_init( SensorInt );
    
    phasePassCountLock = ThreadLocks.Create();
    
    isProcessRunning = true;
    processingThreadID = Threading.StartThread( AsyncUpdate, NULL, THREAD_JOINABLE );
  }
  
  int configKey = (int) kh_str_hash_func( configFileName );
  
  int insertionStatus;
  khint_t newSensorIndex = kh_put( SensorInt, sensorsList, configKey, &insertionStatus );
  if( insertionStatus > 0 )
  {
    kh_value( sensorsList, newSensorIndex ) = newSensor;
    newSensor->isReading = true;
    
    DEBUG_PRINT( "new key %d inserted (iterator: %u - total: %u)", kh_key( sensorsList, newSensorIndex ), newSensorIndex, kh_size( sensorsList ) );
  }
  else if( insertionStatus == 0 ) DEBUG_PRINT( "EMG sensor key %d already exists (iterator %u)", configKey, newSensorIndex );
  
  return (int) kh_key( sensorsList, newSensorIndex );
}

void EMGProcessing_EndSensor( int sensorID )
{
  khint_t sensorIndex = kh_get( SensorInt, sensorsList, (khint_t) sensorID );
  if( sensorIndex == kh_end( sensorsList ) ) return;
  
  UnloadEMGSensorData( kh_value( sensorsList, sensorIndex ) );
  
  kh_del( SensorInt, sensorsList, sensorIndex );
  
  if( kh_size( sensorsList ) == 0 )
  {
    isProcessRunning = false;
    (void) Threading.WaitExit( processingThreadID, 5000 );
    
    ThreadLocks.Discard( phasePassCountLock );
    
    if( sensorsList != NULL )
    {
      kh_destroy( SensorInt, sensorsList );
      sensorsList = NULL;
    }
  }
}

double EMGProcessing_GetNormalizedSignal( int sensorID )
{
  khint_t sensorIndex = kh_get( SensorInt, sensorsList, (khint_t) sensorID );
  if( sensorIndex == kh_end( sensorsList ) ) return 0.0;

  EMGSensor sensor = kh_value( sensorsList, sensorIndex );
    
  return sensor->emgData.processingResultsList[ EMG_ACTIVATION_PHASE ];
}

double EMGProcessing_GetActivation( int sensorID )
{
  khint_t sensorIndex = kh_get( SensorInt, sensorsList, (khint_t) sensorID );
  if( sensorIndex == kh_end( sensorsList ) ) return 0.0;

  MuscleProperties* muscleProperties = &(kh_value( sensorsList, sensorIndex )->muscleProperties);
  
  double normalizedSignal = EMGProcessing_GetNormalizedSignal( sensorID );
  
  return ( exp( muscleProperties->activationFactor * normalizedSignal ) - 1 ) / ( exp( muscleProperties->activationFactor ) - 1 );
}

double EMGProcessing_GetMuscleTorque( int sensorID, double jointAngle )
{
  static double measuresList[ MUSCLE_CURVES_NUMBER ];
  static double penationAngle;
  
  khint_t sensorIndex = kh_get( SensorInt, sensorsList, (khint_t) sensorID );
  if( sensorIndex == kh_end( sensorsList ) ) return 0.0;

  EMGSensor sensor = kh_value( sensorsList, sensorIndex );
  
  //DEBUG_PRINT( "logging data for sensor %d", sensorID );
  
  double activation = EMGProcessing_GetActivation( sensorID );
  
  if( sensor->logID != 0 )
  {
    size_t newSamplesBufferStart = sensor->emgData.samplesBufferStartIndex % sensor->emgData.samplesBufferLength;
    size_t newSamplesBufferMaxLength = sensor->emgData.filteredSamplesBufferLength - FILTER_EXTRA_SAMPLES_NUMBER;
    DataLogging_RegisterList( sensor->logID, newSamplesBufferMaxLength, sensor->emgData.samplesBuffer + newSamplesBufferStart );
    DataLogging_RegisterList( sensor->logID, newSamplesBufferMaxLength, sensor->emgData.filteredSamplesBuffer + FILTER_EXTRA_SAMPLES_NUMBER );
    DataLogging_RegisterValues( sensor->logID, 3, sensor->emgData.processingResultsList[ EMG_ACTIVATION_PHASE ], jointAngle, Timing_GetExecTimeMilliseconds() );
  }
  
  for( size_t curveIndex = 0; curveIndex < MUSCLE_CURVES_NUMBER; curveIndex++ )
  {
    measuresList[ curveIndex ] = 0.0;
    for( size_t factorIndex = 0; factorIndex < MUSCLE_CURVE_ORDER; factorIndex++ )
      measuresList[ curveIndex ] += sensor->muscleProperties.curvesList[ curveIndex ][ factorIndex ] * pow( jointAngle, factorIndex );
  }
  
  /*double normalizedSin = sin( sensor->muscleProperties.initialPenationAngle ) / measuresList[ MUSCLE_NORM_LENGTH ];
  if( normalizedSin > 1.0 ) normalizedSin == 1.0;
  else if( normalizedSin < -1.0 ) normalizedSin == -1.0;
  
  penationAngle = asin( normalizedSin );*/
  
  double activationForce = measuresList[ MUSCLE_ACTIVE_FORCE ] * activation + measuresList[ MUSCLE_PASSIVE_FORCE ];
  double resultingForce = sensor->muscleProperties.scaleFactor * cos( penationAngle ) * activationForce;
  
  return resultingForce * measuresList[ MUSCLE_MOMENT_ARM ];
}

void EMGProcessing_ChangePhase( int sensorID, int newProcessingPhase )
{
  khint_t sensorIndex = kh_get( SensorInt, sensorsList, (khint_t) sensorID );
  if( sensorIndex == kh_end( sensorsList ) ) return;
  
  if( newProcessingPhase < 0 || newProcessingPhase >= EMG_PROCESSING_PHASES_NUMBER ) return;
  
  EMGSensor sensor = kh_value( sensorsList, sensorIndex );
  
  ThreadLocks.Aquire( sensor->lock/*phasePassCountLock*/ );
  
  EMGData* data = &(sensor->emgData);

  if( data->processingPhase == EMG_RELAXATION_PHASE || data->processingPhase == EMG_CONTRACTION_PHASE )
  {
    if( data->preparationPassesCount > 0 )
    {
      DEBUG_EVENT( 0, "new %s value: %g / %u = %g", ( data->processingPhase == EMG_RELAXATION_PHASE ) ? "min" : "max", 
                                                      data->processingResultsList[ data->processingPhase ],
                                                      data->preparationPassesCount, 
                                                      data->processingResultsList[ data->processingPhase ] / data->preparationPassesCount );

      data->processingResultsList[ data->processingPhase ] /= data->preparationPassesCount;
    }

    data->preparationPassesCount = 0;
  }

  data->processingResultsList[ newProcessingPhase ] = 0.0;
  
  data->processingPhase = newProcessingPhase;
  
  ThreadLocks.Release( sensor->lock/*phasePassCountLock*/ );
}

MuscleProperties* EMGProcessing_GetMuscleProperties( int sensorID )
{
  khint_t sensorIndex = kh_get( SensorInt, sensorsList, (khint_t) sensorID );
  if( sensorIndex == kh_end( sensorsList ) ) return NULL;

  return  &(kh_value( sensorsList, sensorIndex )->muscleProperties);
}



static double* GetFilteredSignal( EMGSensor sensor, size_t* ref_aquiredSamplesCount )
{
  const double DATA_AQUISITION_SCALE_FACTOR = 909.0;
  
  size_t aquiredSamplesCount;
  
  double* newSamplesList = sensor->interface.Read( sensor->taskID, sensor->channel, &aquiredSamplesCount );
  
  EMGData* data = &(sensor->emgData);
  size_t samplesBufferLength = data->samplesBufferLength;
  size_t filteredSamplesBufferLength = data->filteredSamplesBufferLength;
  
  if( newSamplesList != NULL ) 
  {
    size_t oldSamplesBufferStart = data->samplesBufferStartIndex;
    size_t newSamplesBufferStart = oldSamplesBufferStart + OLD_SAMPLES_BUFFER_LEN;
    for( int newSampleIndex = 0; newSampleIndex < aquiredSamplesCount; newSampleIndex++ )
      data->samplesBuffer[ ( newSamplesBufferStart + newSampleIndex ) % samplesBufferLength ] = newSamplesList[ newSampleIndex ] / DATA_AQUISITION_SCALE_FACTOR;
    
    static double previousSamplesMean;
    for( int sampleIndex = 0; sampleIndex < aquiredSamplesCount; sampleIndex++ )
    {
      previousSamplesMean = 0.0;
      for( int previousSampleIndex = oldSamplesBufferStart + sampleIndex; previousSampleIndex < newSamplesBufferStart + sampleIndex; previousSampleIndex++ )
        previousSamplesMean += data->samplesBuffer[ previousSampleIndex % samplesBufferLength ] / OLD_SAMPLES_BUFFER_LEN;

      data->rectifiedSamplesBuffer[ FILTER_EXTRA_SAMPLES_NUMBER + sampleIndex ] = fabs( data->samplesBuffer[ ( newSamplesBufferStart + sampleIndex ) % samplesBufferLength ] - previousSamplesMean );
    }
    
    for( int sampleIndex = FILTER_EXTRA_SAMPLES_NUMBER; sampleIndex < filteredSamplesBufferLength; sampleIndex++ )
    {
      data->filteredSamplesBuffer[ sampleIndex ] = 0.0;
      for( int factorIndex = 0; factorIndex < FILTER_ORDER; factorIndex++ )
      {
        data->filteredSamplesBuffer[ sampleIndex ] -= filter_A[ factorIndex ] * data->filteredSamplesBuffer[ sampleIndex - factorIndex ];
        data->filteredSamplesBuffer[ sampleIndex ] += filter_B[ factorIndex ] * data->rectifiedSamplesBuffer[ sampleIndex - factorIndex ];
      }
      if( data->filteredSamplesBuffer[ sampleIndex ] < 0.0 ) data->filteredSamplesBuffer[ sampleIndex ] = 0.0;
    }
    
    for( int sampleIndex = 0; sampleIndex < FILTER_EXTRA_SAMPLES_NUMBER; sampleIndex++ )
    {
      data->filteredSamplesBuffer[ sampleIndex ] = data->filteredSamplesBuffer[ aquiredSamplesCount + sampleIndex ];
      data->rectifiedSamplesBuffer[ sampleIndex ] = data->rectifiedSamplesBuffer[ aquiredSamplesCount + sampleIndex ];
    }

    data->samplesBufferStartIndex += aquiredSamplesCount;
    
    *ref_aquiredSamplesCount = aquiredSamplesCount;

    return ( data->filteredSamplesBuffer + FILTER_EXTRA_SAMPLES_NUMBER );
  }
  
  return NULL;
}

static void* AsyncUpdate( void* data )
{
  while( isProcessRunning )
  {
    for( khint_t sensorID = kh_begin( sensorsList ); sensorID != kh_end( sensorsList ); sensorID++ )
    {
      //DEBUG_PRINT( "looking for sensor %u", sensorID );
      
      if( !kh_exist( sensorsList, sensorID ) ) continue;
      
      EMGSensor sensor = kh_value( sensorsList, sensorID );
      
      if( !sensor->isReading ) continue;

      size_t aquiredSamplesCount;
      double* filteredSamplesList = GetFilteredSignal( sensor, &aquiredSamplesCount );
      
      EMGData* data = &(sensor->emgData);
      
      if( filteredSamplesList != NULL )
      {
        //DEBUG_PRINT( "sensor %u read", sensorID );
        
        if( data->processingPhase == EMG_RELAXATION_PHASE || data->processingPhase == EMG_CONTRACTION_PHASE )
        {
          //DEBUG_PRINT( "adding emg filtered: %g", filteredSamplesList[ 0 ] / aquiredSamplesCount );

          for( size_t sampleIndex = 0; sampleIndex < aquiredSamplesCount; sampleIndex++ )
            data->processingResultsList[ data->processingPhase ] += ( filteredSamplesList[ sampleIndex ] / aquiredSamplesCount );
          
          //DEBUG_PRINT( "emg sum: %g", emgData->processingResultsList[ emgData->processingPhase ] );

          ThreadLocks.Aquire( sensor->lock/*phasePassCountLock*/ );
          data->preparationPassesCount++;
          ThreadLocks.Release( sensor->lock/*phasePassCountLock*/ );
        }
        else if( data->processingPhase == EMG_ACTIVATION_PHASE )
        {
          for( size_t sampleIndex = 0; sampleIndex < aquiredSamplesCount; sampleIndex++ )
          {
            double normalizedSample = 0.0;

            if( data->processingResultsList[ EMG_CONTRACTION_PHASE ] > 0.0 )
              normalizedSample = ( filteredSamplesList[ sampleIndex ] - data->processingResultsList[ EMG_RELAXATION_PHASE ] )
                                 / ( data->processingResultsList[ EMG_CONTRACTION_PHASE ] - data->processingResultsList[ EMG_RELAXATION_PHASE ] );

            if( normalizedSample > 1.0 ) normalizedSample = 1.0;
            else if( normalizedSample < 0.0 ) normalizedSample = 0.0;

            //data->processingResultsList[ EMG_ACTIVATION_PHASE ] = ( exp( -2 * normalizedSample ) - 1 ) / ( exp( -2 ) - 1 );
            data->processingResultsList[ EMG_ACTIVATION_PHASE ] = normalizedSample;
          }
        }
      }
    }
  }
  
  DEBUG_PRINT( "ending processing thread %x", THREAD_ID );
  
  return NULL;
}

const char* CURVE_NAMES[ MUSCLE_CURVES_NUMBER ] = { "active_force", "passive_force", "moment_arm", "normalized_length" };
static EMGSensor LoadEMGSensorData( const char* configFileName )
{
  DEBUG_PRINT( "Trying to load muscle %s EMG sensor data", configFileName );
  
  char searchPath[ FILE_PARSER_MAX_PATH_LENGTH ];
  
  bool loadError = false;
  
  EMGSensor newSensor = (EMGSensor) malloc( sizeof(EMGSensorData) );
  memset( newSensor, 0, sizeof(EMGSensorData) );
  
  int configFileID = ConfigParser.LoadFile( configFileName );
  if( configFileID != -1 )
  {
    bool pluginLoaded;
    GET_PLUGIN_INTERFACE( SIGNAL_AQUISITION_FUNCTIONS, ConfigParser.GetStringValue( configFileID, "aquisition_system.type" ), newSensor->interface, pluginLoaded );
    if( pluginLoaded )
    {
      newSensor->taskID = newSensor->interface.InitTask( ConfigParser.GetStringValue( configFileID, "aquisition_system.task" ) );
      if( newSensor->taskID != -1 )
      {
        size_t newSamplesBufferMaxLength = newSensor->interface.GetMaxSamplesNumber( newSensor->taskID );
        if( newSamplesBufferMaxLength > 0 )
        {
          newSensor->emgData.samplesBufferLength = newSamplesBufferMaxLength + OLD_SAMPLES_BUFFER_LEN;
          newSensor->emgData.samplesBuffer = (double*) calloc( newSensor->emgData.samplesBufferLength, sizeof(double) );
          newSensor->emgData.filteredSamplesBufferLength = newSamplesBufferMaxLength + FILTER_EXTRA_SAMPLES_NUMBER;
          newSensor->emgData.filteredSamplesBuffer = (double*) calloc( newSensor->emgData.filteredSamplesBufferLength, sizeof(double) );
          newSensor->emgData.rectifiedSamplesBuffer = (double*) calloc( newSensor->emgData.filteredSamplesBufferLength, sizeof(double) );

          if( ConfigParser.GetBooleanValue( configFileID, "log_data" ) )
          {
            newSensor->logID = DataLogging_InitLog( configFileName, 2 * newSamplesBufferMaxLength + 3, 1000 );
            DataLogging_SetDataPrecision( newSensor->logID, 4 );
          }
        }
        else loadError = true;

        newSensor->channel = ConfigParser.GetIntegerValue( configFileID, "aquisition_system.channel" );
        if( !(newSensor->interface.AquireChannel( newSensor->taskID, newSensor->channel )) ) loadError = true;
      }
      else loadError = true;
    }
    else loadError = true;

    for( size_t curveIndex = 0; curveIndex < MUSCLE_CURVES_NUMBER; curveIndex++ )
    {
      sprintf( searchPath, "muscle_properties.curves.%s", CURVE_NAMES[ curveIndex ] );
      size_t coeffsNumber = ConfigParser.GetListSize( configFileID, searchPath );
      for( size_t coeffIndex = 0; coeffIndex < coeffsNumber; coeffIndex++ )
      {
        sprintf( searchPath, "muscle_properties.curves.%s.%u", CURVE_NAMES[ curveIndex ], coeffIndex );
        DEBUG_PRINT( "searching for value %s", searchPath );
        newSensor->muscleProperties.curvesList[ curveIndex ][ coeffIndex ] = ConfigParser.GetRealValue( configFileID, searchPath );
      }
    }

    newSensor->muscleProperties.initialPenationAngle = ConfigParser.GetRealValue( configFileID, "muscle_properties.penation_angle" );
    newSensor->muscleProperties.scaleFactor = ConfigParser.GetRealValue( configFileID, "muscle_properties.scale_factor" );

    newSensor->lock = ThreadLocks.Create();

    ConfigParser.UnloadFile( configFileID );
  }
  else
  {
    DEBUG_PRINT( "configuration for muscle %s EMG sensor not found", configFileName );
    loadError = true;
  }
    
  if( loadError )
  {
    UnloadEMGSensorData( newSensor );
    return NULL;
  }
  
  return newSensor;
}

void UnloadEMGSensorData( EMGSensor sensor )
{
  if( sensor == NULL ) return;
  
  sensor->isReading = false;
  
  ThreadLocks.Aquire( sensor->lock );
  ThreadLocks.Release( sensor->lock );
    
  ThreadLocks.Discard( sensor->lock );
  
  free( sensor->emgData.samplesBuffer );
  free( sensor->emgData.filteredSamplesBuffer );
  free( sensor->emgData.rectifiedSamplesBuffer );
  
  sensor->interface.ReleaseChannel( sensor->taskID, sensor->channel );
  sensor->interface.EndTask( sensor->taskID );
  
  if( sensor->logID != 0 ) DataLogging_EndLog( sensor->logID );
  
  free( sensor );
}


#endif /* EMG_PROCESS_H */
