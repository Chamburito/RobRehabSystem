#ifndef EMG_PROCESS_H
#define EMG_PROCESS_H

#include <math.h>
#include <stdbool.h>

#include "signal_aquisition/signal_aquisition_types.h"

#include "klib/khash.h"
#include "file_parsing/json_parser.h"

#include "debug/async_debug.h"

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

typedef struct _EMGSensor
{
  SignalAquisitionInterface interface;
  int taskID;
  unsigned int channel;
  EMGData emgData;
  MuscleProperties muscleProperties;
  ThreadLock lock;
}
EMGSensor;

KHASH_MAP_INIT_INT( SensorInt, EMGSensor* )
static khash_t( SensorInt )* sensorsList = NULL;

Thread_Handle emgThreadID;
ThreadLock phasePassCountLock;
static bool isProcessRunning;

static int InitSensor( const char* );
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
EMGProcessing = { .InitSensor = InitSensor, .EndSensor = EndSensor, .GetActivation = GetActivation, .GetMuscleTorque = GetMuscleTorque, .ChangePhase = ChangePhase };

static void* AsyncUpdate( void* );
static double* GetFilteredSignal( EMGSensor*, size_t* );

static EMGSensor* LoadEMGSensorData( const char* );
static void UnloadEMGSensorData( EMGSensor* );

int InitSensor( const char* configFileName )
{
  EMGSensor* newSensor = LoadEMGSensorData( configFileName );
  if( newSensor == NULL )
  {
    DEBUG_PRINT( "EMG sensor %s configuration failed", configFileName );
    return -1;
  }
  
  if( sensorsList == NULL ) 
  {
    sensorsList = kh_init( SensorInt );
    
    phasePassCountLock = ThreadLock_Create();
    
    isProcessRunning = true;
    emgThreadID = Thread_Start( AsyncUpdate, NULL, THREAD_JOINABLE );
  }
  
  int configKey = (int) kh_str_hash_func( configFileName );
  
  //ThreadLock_Aquire( phasePassCountLock );
  int insertionStatus;
  khint_t newSensorID = kh_put( SensorInt, sensorsList, configKey, &insertionStatus );
  if( insertionStatus > 0 )
  {
    kh_value( sensorsList, newSensorID ) = newSensor;
    
    DEBUG_PRINT( "new key %d inserted (iterator: %u - total: %u)", kh_key( sensorsList, newSensorID ), newSensorID, kh_size( sensorsList ) );
  }
  else if( insertionStatus == 0 ) { DEBUG_PRINT( "EMG sensor key %d already exists (iterator %u)", configKey, newSensorID ); }
  
  //ThreadLock_Release( phasePassCountLock );
  
  return (int) newSensorID;
}

void EndSensor( int sensorID )
{
  if( !kh_exist( sensorsList, (khint_t) sensorID ) ) return;
  
  UnloadEMGSensorData( kh_value( sensorsList, (khint_t) sensorID ) );
  
  kh_del( SensorInt, sensorsList, (khint_t) sensorID );
  
  if( kh_size( sensorsList ) == 0 )
  {
    isProcessRunning = false;
    (void) Thread_WaitExit( emgThreadID, 5000 );
    
    ThreadLock_Discard( phasePassCountLock );
    
    if( sensorsList != NULL )
    {
      kh_destroy( SensorInt, sensorsList );
      sensorsList = NULL;
    }
  }
}

static double GetActivation( int sensorID )
{
  if( !kh_exist( sensorsList, (khint_t) sensorID ) ) return 0.0;

  EMGSensor* sensor = kh_value( sensorsList, (khint_t) sensorID );
    
  return sensor->emgData.processingResultsList[ EMG_ACTIVATION_PHASE ];
}

double GetMuscleTorque( int sensorID, double jointAngle )
{
  static double measuresList[ MUSCLE_CURVES_NUMBER ];
  static double penationAngle;
  
  if( !kh_exist( sensorsList, (khint_t) sensorID ) ) return 0.0;

  EMGSensor* sensor = kh_value( sensorsList, (khint_t) sensorID );
  
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

void ChangePhase( int sensorID, int newProcessingPhase )
{
  if( !kh_exist( sensorsList, (khint_t) sensorID ) ) return;
  
  if( newProcessingPhase < 0 || newProcessingPhase >= EMG_PROCESSING_PHASES_NUMBER ) return;
  
  ThreadLock_Aquire( phasePassCountLock );
  
  EMGData* emgData = &(kh_value( sensorsList, sensorID )->emgData);

  if( emgData->processingPhase == EMG_RELAXATION_PHASE || emgData->processingPhase == EMG_CONTRACTION_PHASE )
  {
    if( emgData->preparationPassesCount > 0 )
    {
      //DEBUG_PRINT( "new %s value: %g / %u = %g", ( emgData->processingPhase == EMG_RELAXATION_PHASE ) ? "min" : "max", 
      //                                              emgData->processingResultsList[ emgData->processingPhase ],
      //                                              emgData->preparationPassesCount, 
      //                                              emgData->processingResultsList[ emgData->processingPhase ] / emgData->preparationPassesCount );

      emgData->processingResultsList[ emgData->processingPhase ] /= emgData->preparationPassesCount;
    }

    emgData->preparationPassesCount = 0;
  }

  emgData->processingResultsList[ newProcessingPhase ] = 0.0;
  
  emgData->processingPhase = newProcessingPhase;
  
  ThreadLock_Release( phasePassCountLock );
}



static double* GetFilteredSignal( EMGSensor* sensor, size_t* ref_aquiredSamplesCount )
{
  const double DATA_AQUISITION_SCALE_FACTOR = 909.0;
  
  size_t aquiredSamplesCount;
  //ThreadLock_Aquire( phasePassCountLock );
  double* newSamplesList = sensor->interface->Read( sensor->taskID, sensor->channel, &aquiredSamplesCount );
  
  EMGData* data = &(sensor->emgData);
  size_t samplesBufferLength = sensor->emgData.samplesBufferLength;
  size_t filteredSamplesBufferLength = sensor->emgData.filteredSamplesBufferLength;
  //ThreadLock_Release( phasePassCountLock );
  
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
      if( !kh_exist( sensorsList, sensorID ) ) continue;
      
      EMGSensor* sensor = kh_value( sensorsList, sensorID );

      size_t aquiredSamplesCount;
      double* filteredSamplesList = GetFilteredSignal( sensor, &aquiredSamplesCount );
      
      EMGData* data = &(sensor->emgData);
      
      if( filteredSamplesList != NULL )
      {
        if( data->processingPhase == EMG_RELAXATION_PHASE || data->processingPhase == EMG_CONTRACTION_PHASE )
        {
          //DEBUG_PRINT( "adding emg filtered: %g", filteredSamplesList[ 0 ] / aquiredSamplesCount );

          for( size_t sampleIndex = 0; sampleIndex < aquiredSamplesCount; sampleIndex++ )
            data->processingResultsList[ data->processingPhase ] += ( filteredSamplesList[ sampleIndex ] / aquiredSamplesCount );
          
          //DEBUG_PRINT( "emg sum: %g", emgData->processingResultsList[ emgData->processingPhase ] );

          ThreadLock_Aquire( phasePassCountLock );
          data->preparationPassesCount++;
          ThreadLock_Release( phasePassCountLock );
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
  
  Thread_Exit( 0 );
  return NULL;
}

const char* CURVE_NAMES[ MUSCLE_CURVES_NUMBER ] = { "active_force", "passive_force", "moment_arm", "normalized_length" };
static EMGSensor* LoadEMGSensorData( const char* configFileName )
{
  DEBUG_PRINT( "Trying to load muscle %s EMG sensor data", configFileName );
  
  static char searchPath[ FILE_PARSER_MAX_PATH_LENGTH ];
  
  bool loadError = false;
  
  EMGSensor* newSensor = (EMGSensor*) malloc( sizeof(EMGSensor) );
  memset( newSensor, 0, sizeof(EMGSensor) );
  
  FileParser parser = JSONParser;
  sprintf( searchPath, "emg_sensors/%s", configFileName );
  int configFileID = parser.LoadFile( searchPath );
  if( configFileID != -1 )
  {
    if( (newSensor->interface = SignalAquisitionTypes.GetInterface( parser.GetStringValue( configFileID, "aquisition_system.type" ) )) != NULL )
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
    
    newSensor->lock = ThreadLock_Create();
    
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

void UnloadEMGSensorData( EMGSensor* sensor )
{
  if( sensor == NULL ) return;
  
  ThreadLock_Aquire( sensor->lock );
  ThreadLock_Release( sensor->lock );
    
  ThreadLock_Discard( sensor->lock );
  
  free( sensor->emgData.samplesBuffer );
  free( sensor->emgData.filteredSamplesBuffer );
  free( sensor->emgData.rectifiedSamplesBuffer );
  
  sensor->interface->ReleaseChannel( sensor->taskID, sensor->channel );
  sensor->interface->EndTask( sensor->taskID );
  
  free( sensor );
}


#endif /* EMG_PROCESS_H */
