#ifndef EMG_PROCESS_H
#define EMG_PROCESS_H

#include <math.h>
#include <stdbool.h>

#include "signal_aquisition/signal_aquisition_types.h"

#include "klib/khash.h"
#include "file_parsing/json_parser.h"

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

enum { ACTIVE_FORCE, PASSIVE_FORCE, MOMENT_ARM, NORM_LENGTH, MUSCLE_CURVES_NUMBER };
const size_t MUSCLE_CURVE_ORDER = 5;
typedef double MuscleCurve[ MUSCLE_CURVE_ORDER ];

typedef struct _MuscleProperties
{
  MuscleCurve curvesList[ MUSCLE_CURVES_NUMBER ];
  double initialPenationAngle;
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

/*static int InitSensor( const char* );
static void EndSensor( int );
static double GetActivation( int );
static double GetMuscleTorque( int, double );
static void ChangePhase( int, enum EMGProcessPhase );

const struct
{
  int (*InitSensor)( const char* );
  void (*EndSensor)( int );
  double (*GetActivation)( int );
  double (*GetMuscleTorque)( int, double );
  void (*ChangePhase)( int, enum EMGProcessPhase );
}
EMGProcessing = { .InitSensor = InitSensor, .EndSensor = EndSensor, .GetActivation = GetActivation, .GetMuscleTorque = GetMuscleTorque, .ChangePhase = ChangePhase };*/

#define NAMESPACE EMGProcessing

#define NAMESPACE_FUNCTIONS( namespace ) \
        NAMESPACE_FUNCTION( int, namespace, InitSensor, const char* ) \
        NAMESPACE_FUNCTION( void, namespace, EndSensor, int ) \
        NAMESPACE_FUNCTION( double, namespace, GetActivation, int ) \
        NAMESPACE_FUNCTION( double, namespace, GetMuscleTorque, int, double ) \
        NAMESPACE_FUNCTION( void, namespace, ChangePhase, int, enum EMGProcessPhase )

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

static void* AsyncUpdate( void* );
static double* GetFilteredSignal( EMGSensor, size_t* );

static EMGSensor LoadEMGSensorData( const char* );
static void UnloadEMGSensorData( EMGSensor );

int EMGProcessing_InitSensor( const char* configFileName )
{
  EMGSensor newSensor = LoadEMGSensorData( configFileName );
  if( newSensor == NULL )
  {
    DEBUG_PRINT( "EMG sensor %s configuration failed", configFileName );
    return -1;
  }
  
  if( sensorsList == NULL ) 
  {
    sensorsList = kh_init( SensorInt );
    
    phasePassCountLock = ThreadLocks_Create();
    
    isProcessRunning = true;
    processingThreadID = Threading_StartThread( AsyncUpdate, NULL, THREAD_JOINABLE );
  }
  
  int configKey = (int) kh_str_hash_func( configFileName );
  
  int insertionStatus;
  khint_t newSensorID = kh_put( SensorInt, sensorsList, configKey, &insertionStatus );
  if( insertionStatus > 0 )
  {
    kh_value( sensorsList, newSensorID ) = newSensor;
    newSensor->isReading = true;
    
    DEBUG_PRINT( "new key %d inserted (iterator: %u - total: %u)", kh_key( sensorsList, newSensorID ), newSensorID, kh_size( sensorsList ) );
  }
  else if( insertionStatus == 0 ) { DEBUG_PRINT( "EMG sensor key %d already exists (iterator %u)", configKey, newSensorID ); }
  
  return (int) newSensorID;
}

void EMGProcessing_EndSensor( int sensorID )
{
  if( !kh_exist( sensorsList, (khint_t) sensorID ) ) return;
  
  UnloadEMGSensorData( kh_value( sensorsList, (khint_t) sensorID ) );
  
  kh_del( SensorInt, sensorsList, (khint_t) sensorID );
  
  if( kh_size( sensorsList ) == 0 )
  {
    isProcessRunning = false;
    (void) Threading_WaitExit( processingThreadID, 5000 );
    
    ThreadLocks_Discard( phasePassCountLock );
    
    if( sensorsList != NULL )
    {
      kh_destroy( SensorInt, sensorsList );
      sensorsList = NULL;
    }
  }
}

static double EMGProcessing_GetActivation( int sensorID )
{
  if( !kh_exist( sensorsList, (khint_t) sensorID ) ) return 0.0;

  EMGSensor sensor = kh_value( sensorsList, (khint_t) sensorID );
    
  return sensor->emgData.processingResultsList[ EMG_ACTIVATION_PHASE ];
}

double EMGProcessing_GetMuscleTorque( int sensorID, double jointAngle )
{
  static double measuresList[ MUSCLE_CURVES_NUMBER ];
  static double penationAngle;
  
  if( !kh_exist( sensorsList, (khint_t) sensorID ) ) return 0.0;

  EMGSensor sensor = kh_value( sensorsList, (khint_t) sensorID );
  
  //DEBUG_PRINT( "logging data for sensor %d", sensorID );
  
  size_t newSamplesBufferStart = sensor->emgData.samplesBufferStartIndex % sensor->emgData.samplesBufferLength;
  size_t newSamplesBufferMaxLength = sensor->emgData.filteredSamplesBufferLength - FILTER_EXTRA_SAMPLES_NUMBER;
  DataLogging_RegisterList( sensor->logID, newSamplesBufferMaxLength, sensor->emgData.samplesBuffer + newSamplesBufferStart );
  DataLogging_RegisterList( sensor->logID, newSamplesBufferMaxLength, sensor->emgData.filteredSamplesBuffer + FILTER_EXTRA_SAMPLES_NUMBER );
  DataLogging_RegisterValues( sensor->logID, 3, sensor->emgData.processingResultsList[ EMG_ACTIVATION_PHASE ], jointAngle, Timing_GetExecTimeMilliseconds() );
  
  for( size_t curveIndex = 0; curveIndex < MUSCLE_CURVES_NUMBER; curveIndex++ )
  {
    measuresList[ curveIndex ] = 0.0;
    for( size_t factorIndex = 0; factorIndex < MUSCLE_CURVE_ORDER; factorIndex++ )
      measuresList[ curveIndex ] += sensor->muscleProperties.curvesList[ curveIndex ][ factorIndex ] * pow( jointAngle, factorIndex );
  }
  
  /*double normalizedSin = sin( sensor->muscleProperties.initialPenationAngle ) / measuresList[ NORM_LENGTH ];
  if( normalizedSin > 1.0 ) normalizedSin == 1.0;
  else if( normalizedSin < -1.0 ) normalizedSin == -1.0;
  
  penationAngle = asin( normalizedSin );*/
  
  double resultingForce = sensor->muscleProperties.scaleFactor * cos( penationAngle )
                          * ( measuresList[ ACTIVE_FORCE ] * sensor->emgData.processingResultsList[ EMG_ACTIVATION_PHASE ] + measuresList[ PASSIVE_FORCE ] );
  
  return resultingForce * measuresList[ MOMENT_ARM ];
}

void EMGProcessing_ChangePhase( int sensorID, int newProcessingPhase )
{
  if( !kh_exist( sensorsList, (khint_t) sensorID ) ) return;
  
  if( newProcessingPhase < 0 || newProcessingPhase >= EMG_PROCESSING_PHASES_NUMBER ) return;
  
  EMGSensor sensor = kh_value( sensorsList, sensorID );
  
  ThreadLocks_Aquire( sensor->lock/*phasePassCountLock*/ );
  
  EMGData* data = &(sensor->emgData);

  if( data->processingPhase == EMG_RELAXATION_PHASE || data->processingPhase == EMG_CONTRACTION_PHASE )
  {
    if( data->preparationPassesCount > 0 )
    {
      //DEBUG_PRINT( "new %s value: %g / %u = %g", ( data->processingPhase == EMG_RELAXATION_PHASE ) ? "min" : "max", 
      //                                             data->processingResultsList[ data->processingPhase ],
      //                                             data->preparationPassesCount, 
      //                                             data->processingResultsList[ data->processingPhase ] / data->preparationPassesCount );

      data->processingResultsList[ data->processingPhase ] /= data->preparationPassesCount;
    }

    data->preparationPassesCount = 0;
  }

  data->processingResultsList[ newProcessingPhase ] = 0.0;
  
  data->processingPhase = newProcessingPhase;
  
  ThreadLocks_Release( sensor->lock/*phasePassCountLock*/ );
}



static double* GetFilteredSignal( EMGSensor sensor, size_t* ref_aquiredSamplesCount )
{
  const double DATA_AQUISITION_SCALE_FACTOR = 909.0;
  
  size_t aquiredSamplesCount;
  
  double* newSamplesList = sensor->interface->Read( sensor->taskID, sensor->channel, &aquiredSamplesCount );
  
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
      DEBUG_PRINT( "looking for sensor %u", sensorID );
      
      if( !kh_exist( sensorsList, sensorID ) ) continue;
      
      EMGSensor sensor = kh_value( sensorsList, sensorID );
      
      if( !sensor->isReading ) continue;

      size_t aquiredSamplesCount;
      double* filteredSamplesList = GetFilteredSignal( sensor, &aquiredSamplesCount );
      
      EMGData* data = &(sensor->emgData);
      
      if( filteredSamplesList != NULL )
      {
        DEBUG_PRINT( "sensor %u read", sensorID );
        
        if( data->processingPhase == EMG_RELAXATION_PHASE || data->processingPhase == EMG_CONTRACTION_PHASE )
        {
          //DEBUG_PRINT( "adding emg filtered: %g", filteredSamplesList[ 0 ] / aquiredSamplesCount );

          for( size_t sampleIndex = 0; sampleIndex < aquiredSamplesCount; sampleIndex++ )
            data->processingResultsList[ data->processingPhase ] += ( filteredSamplesList[ sampleIndex ] / aquiredSamplesCount );
          
          //DEBUG_PRINT( "emg sum: %g", emgData->processingResultsList[ emgData->processingPhase ] );

          ThreadLocks_Aquire( sensor->lock/*phasePassCountLock*/ );
          data->preparationPassesCount++;
          ThreadLocks_Release( sensor->lock/*phasePassCountLock*/ );
        }
        else if( data->processingPhase == EMG_ACTIVATION_PHASE )
        {
          static double normalizedSample;
          
          for( size_t sampleIndex = 0; sampleIndex < aquiredSamplesCount; sampleIndex++ )
          {
            normalizedSample = 0.0;

            if( data->processingResultsList[ EMG_CONTRACTION_PHASE ] > 0.0 )
              normalizedSample = ( filteredSamplesList[ sampleIndex ] - data->processingResultsList[ EMG_RELAXATION_PHASE ] )
                                 / ( data->processingResultsList[ EMG_CONTRACTION_PHASE ] - data->processingResultsList[ EMG_RELAXATION_PHASE ] );

            if( normalizedSample > 1.0 ) normalizedSample = 1.0;
            else if( normalizedSample < 0.0 ) normalizedSample = 0.0;

            data->processingResultsList[ EMG_ACTIVATION_PHASE ] = ( exp( -2 * normalizedSample ) - 1 ) / ( exp( -2 ) - 1 );
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
  
  static char searchPath[ FILE_PARSER_MAX_PATH_LENGTH ];
  
  bool loadError = false;
  
  EMGSensor newSensor = (EMGSensor) malloc( sizeof(EMGSensorData) );
  memset( newSensor, 0, sizeof(EMGSensorData) );
  
  FileParser parser = JSONParser;
  int configFileID = parser.LoadFile( configFileName );
  if( configFileID != -1 )
  {
    if( (newSensor->interface = GetSignalAquisitionInterface( parser.GetStringValue( configFileID, "aquisition_system.type" ) )) != NULL )
    {
      newSensor->taskID = newSensor->interface->InitTask( parser.GetStringValue( configFileID, "aquisition_system.task" ) );
      if( newSensor->taskID != -1 ) 
      {
        size_t newSamplesBufferMaxLength = newSensor->interface->GetMaxSamplesNumber( newSensor->taskID );
        if( newSamplesBufferMaxLength > 0 )
        {
          newSensor->emgData.samplesBufferLength = newSamplesBufferMaxLength + OLD_SAMPLES_BUFFER_LEN;
          newSensor->emgData.samplesBuffer = (double*) calloc( newSensor->emgData.samplesBufferLength, sizeof(double) );
          newSensor->emgData.filteredSamplesBufferLength = newSamplesBufferMaxLength + FILTER_EXTRA_SAMPLES_NUMBER;
          newSensor->emgData.filteredSamplesBuffer = (double*) calloc( newSensor->emgData.filteredSamplesBufferLength, sizeof(double) );
          newSensor->emgData.rectifiedSamplesBuffer = (double*) calloc( newSensor->emgData.filteredSamplesBufferLength, sizeof(double) );
          
          newSensor->logID = DataLogging_InitLog( configFileName, 2 * newSamplesBufferMaxLength + 3, 1000 );
          DataLogging_SetDataPrecision( newSensor->logID, 4 );
        }
        else loadError = true;
        
        newSensor->channel = parser.GetIntegerValue( configFileID, "aquisition_system.channel" );
        if( !(newSensor->interface->AquireChannel( newSensor->taskID, newSensor->channel )) ) loadError = true;
      }
      else loadError = true;
    }
    else loadError = true;
    
    for( size_t curveIndex = 0; curveIndex < MUSCLE_CURVES_NUMBER; curveIndex++ )
    {
      sprintf( searchPath, "muscle_properties.curves.%s", CURVE_NAMES[ curveIndex ] );
      size_t coeffsNumber = parser.GetListSize( configFileID, searchPath ); 
      for( size_t coeffIndex = 0; coeffIndex < coeffsNumber; coeffIndex++ )
      {
        sprintf( searchPath, "muscle_properties.curves.%s.%u", CURVE_NAMES[ curveIndex ], coeffIndex );
        DEBUG_PRINT( "searching for value %s", searchPath );
        newSensor->muscleProperties.curvesList[ curveIndex ][ coeffIndex ] = parser.GetRealValue( configFileID, searchPath );
      }
    }
    
    newSensor->muscleProperties.initialPenationAngle = parser.GetRealValue( configFileID, "muscle_properties.penation_angle" );
    newSensor->muscleProperties.scaleFactor = parser.GetRealValue( configFileID, "muscle_properties.scale_factor" );
    
    newSensor->lock = ThreadLocks_Create();
    
    parser.UnloadFile( configFileID );
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
  
  ThreadLocks_Aquire( sensor->lock );
  ThreadLocks_Release( sensor->lock );
    
  ThreadLocks_Discard( sensor->lock );
  
  free( sensor->emgData.samplesBuffer );
  free( sensor->emgData.filteredSamplesBuffer );
  free( sensor->emgData.rectifiedSamplesBuffer );
  
  sensor->interface->ReleaseChannel( sensor->taskID, sensor->channel );
  sensor->interface->EndTask( sensor->taskID );
  
  DataLogging_EndLog( sensor->logID );
  
  free( sensor );
}


#endif /* EMG_PROCESS_H */
