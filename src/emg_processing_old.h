#ifndef EMG_PROCESS_H
#define EMG_PROCESS_H

#include <math.h>
#include <stdbool.h>

#include "signal_aquisition.h"

#include "klib/khash.h"
#include "file_parsing/json_parser.h"

#include "debug/async_debug.h"

enum EMGProcessPhase { EMG_ACTIVATION_PHASE, EMG_RELAXATION_PHASE, EMG_CONTRACTION_PHASE, EMG_PROCESSING_PHASES_NUMBER };

const size_t AQUISITION_BUFFER_LEN = 10;
const size_t OLD_SAMPLES_BUFFER_LEN = 100;
const size_t HISTORY_BUFFER_LEN = OLD_SAMPLES_BUFFER_LEN + AQUISITION_BUFFER_LEN;

const size_t FILTER_ORDER = 3;
const double filter_A[ FILTER_ORDER ] = { 1.0, -1.9964, 0.9965 };
const double filter_B[ FILTER_ORDER ] = { 1.576e-6, 3.153e-6, 1.576e-6 };

const int FILTER_EXTRA_SAMPLES_NUMBER = FILTER_ORDER - 1;
const int FILTER_BUFFER_LEN = FILTER_EXTRA_SAMPLES_NUMBER + AQUISITION_BUFFER_LEN;

typedef struct _EMGData
{
  double samplesBuffer[ HISTORY_BUFFER_LEN ];
  size_t samplesBufferStartIndex;
  double rectifiedBuffer[ FILTER_BUFFER_LEN ];
  double filteredBuffer[ FILTER_BUFFER_LEN ];
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
  int taskID;
  unsigned int channel;
  EMGData emgData;
  MuscleProperties muscleProperties;
}
EMGSensor;

KHASH_MAP_INIT_INT( EMG, EMGSensor )
static khash_t( EMG )* emgSensorsList = NULL;

Thread_Handle emgThreadID;
ThreadLock phasePassCountLock;
static bool isProcessRunning;

static int InitSensor( const char* );
static void EndSensor( int );
static double GetActivation( int );
static double GetMuscleTorque( int );
static void ChangePhase( int, enum EMGProcessPhase );

const struct
{
  int (*InitSensor)( const char* );
  void (*EndSensor)( int );
  double (*GetActivation)( int );
  double (*GetMuscleTorque)( int );
  void (*ChangePhase)( int, enum EMGProcessPhase );
}
EMGProcessing = { .InitSensor = InitSensor, .EndSensor = EndSensor, .GetActivation = GetActivation, .GetMuscleTorque = GetMuscleTorque, .ChangePhase = ChangePhase };

static void* AsyncUpdate( void* );
static double* GetFilteredSignal( EMGSensor* );

static EMGSensor* LoadEMGSensorData( const char* );
static void UnloadEMGSensorData( EMGSensor* );

int InitSensor( const char* configFileName )
{
  EMGSensor* ref_newSensorData = LoadEMGSensorData( configFileName );
  if( ref_newSensorData == NULL ) return -1;
  
  if( emgSensorsList == NULL ) 
  {
    phasePassCountLock = ThreadLock_Create();
    
    isProcessRunning = true;
    emgThreadID = Thread_Start( AsyncUpdate, NULL, THREAD_JOINABLE );
    
    emgSensorsList = kh_init( EMG );
  }
  
  int insertionStatus;
  khint_t newSensorID = kh_put( EMG, emgSensorsList, (int) configFileName, &insertionStatus );
  if( insertionStatus == -1 ) return -1;
  
  EMGSensor* newSensor = &(kh_value( emgSensorsList, newSensorID ));
  
  memcpy( newSensor, ref_newSensorData, sizeof(EMGSensor) );
  
  return (int) newSensorID;
}

void EndSensor( int sensorID )
{
  if( !kh_exist( emgSensorsList, (khint_t) sensorID ) ) return;
  
  kh_del( EMG, emgSensorsList, (khint_t) sensorID );
  
  if( kh_size( emgSensorsList ) == 0 )
  {
    isProcessRunning = false;
    (void) Thread_WaitExit( emgThreadID, 5000 );
    
    ThreadLock_Discard( phasePassCountLock );
    
    kh_destroy( EMG, emgSensorsList );
    emgSensorsList = NULL;
  }
}

static double GetActivation( int sensorID )
{
  if( !kh_exist( emgSensorsList, (khint_t) sensorID ) ) return 0.0;

  EMGSensor* sensor = &(kh_value( emgSensorsList, (khint_t) sensorID ));
    
  return sensor->emgData.processingResultsList[ EMG_ACTIVATION_PHASE ];
}

double GetMuscleTorque( int sensorID, double jointAngle )
{
  static double measuresList[ MUSCLE_CURVES_NUMBER ];
  static double penationAngle;
  
  if( !kh_exist( emgSensorsList, (khint_t) sensorID ) ) return 0.0;

  EMGSensor* sensor = &(kh_value( emgSensorsList, (khint_t) sensorID ));
  
  for( size_t curveIndex = 0; curveIndex < MUSCLE_CURVES_NUMBER; curveIndex++ )
  {
    measuresList[ curveIndex ] = 0.0;
    for( size_t factorIndex = 0; factorIndex < MUSCLE_CURVE_ORDER; factorIndex++ )
      measuresList[ curveIndex ] += sensor->muscleProperties.curvesList[ curveIndex ][ factorIndex ] * pow( jointAngle, factorIndex );
  }
  
  penationAngle = asin( sin( sensor->muscleProperties.initialPenationAngle ) / measuresList[ NORM_LENGTH ] );
  
  //if( muscleProperties->processingPhase == EMG_ACTIVATION_PHASE )
  //  DEBUG_PRINT( "theta: %lf - active force: %lf - passive force: %lf - moment arm: %lf - normalized length: %lf - penation angle: %lf",
  //               jointAngle, measuresList[ ACTIVE_FORCE ], measuresList[ PASSIVE_FORCE ], measuresList[ MOMENT_ARM ], measuresList[ NORM_LENGTH ], penationAngle );
  
  double resultingForce = sensor->muscleProperties.scaleFactor * cos( penationAngle )
                          * ( measuresList[ ACTIVE_FORCE ] * sensor->emgData.processingResultsList[ EMG_ACTIVATION_PHASE ] + measuresList[ PASSIVE_FORCE ] );
  
  return resultingForce * measuresList[ MOMENT_ARM ];
}

void ChangePhase( int sensorID, int newProcessingPhase )
{
  if( !kh_exist( emgSensorsList, (khint_t) sensorID ) ) return;
  
  if( newProcessingPhase < 0 || newProcessingPhase >= EMG_PROCESSING_PHASES_NUMBER ) return;
  
  ThreadLock_Aquire( phasePassCountLock );
  
  EMGData* emgData = &(kh_value( emgSensorsList, sensorID ).emgData);

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



static double* GetFilteredSignal( EMGSensor* sensor )
{
  const double DATA_AQUISITION_SCALE_FACTOR = 909.0;
  
  static double newFilteredSamplesList[ AQUISITION_BUFFER_LEN ];
  
  double* newSamplesList = SignalAquisition.Read( sensor->taskID, sensor->channel );
  
  EMGData* data = &(sensor->emgData);
  
  if( newSamplesList != NULL ) 
  {
    size_t oldSamplesBufferStart = data->samplesBufferStartIndex;
    size_t newSamplesBufferStart = oldSamplesBufferStart + OLD_SAMPLES_BUFFER_LEN;
    for( int newSampleIndex = 0; newSampleIndex < AQUISITION_BUFFER_LEN; newSampleIndex++ )
      data->samplesBuffer[ ( newSamplesBufferStart + newSampleIndex ) % HISTORY_BUFFER_LEN ] = newSamplesList[ newSampleIndex ] / DATA_AQUISITION_SCALE_FACTOR;
    
    static double previousSamplesMean;
    for( int sampleIndex = 0; sampleIndex < AQUISITION_BUFFER_LEN; sampleIndex++ )
    {
      previousSamplesMean = 0.0;
      for( int previousSampleIndex = oldSamplesBufferStart + sampleIndex; previousSampleIndex < newSamplesBufferStart + sampleIndex; previousSampleIndex++ )
        previousSamplesMean += data->samplesBuffer[ previousSampleIndex % HISTORY_BUFFER_LEN ] / OLD_SAMPLES_BUFFER_LEN;

      data->rectifiedBuffer[ FILTER_EXTRA_SAMPLES_NUMBER + sampleIndex ] = fabs( data->samplesBuffer[ ( newSamplesBufferStart + sampleIndex ) % HISTORY_BUFFER_LEN ] - previousSamplesMean );
    }
    
    for( int sampleIndex = FILTER_EXTRA_SAMPLES_NUMBER; sampleIndex < FILTER_BUFFER_LEN; sampleIndex++ )
    {
      data->filteredBuffer[ sampleIndex ] = 0.0;
      for( int factorIndex = 0; factorIndex < FILTER_ORDER; factorIndex++ )
      {
        data->filteredBuffer[ sampleIndex ] -= filter_A[ factorIndex ] * data->filteredBuffer[ sampleIndex - factorIndex ];
        data->filteredBuffer[ sampleIndex ] += filter_B[ factorIndex ] * data->rectifiedBuffer[ sampleIndex - factorIndex ];
      }
      if( data->filteredBuffer[ sampleIndex ] < 0.0 ) data->filteredBuffer[ sampleIndex ] = 0.0;
    }
    
    for( int sampleIndex = 0; sampleIndex < FILTER_EXTRA_SAMPLES_NUMBER; sampleIndex++ )
    {
      data->filteredBuffer[ sampleIndex ] = data->filteredBuffer[ AQUISITION_BUFFER_LEN + sampleIndex ];
      data->rectifiedBuffer[ sampleIndex ] = data->rectifiedBuffer[ AQUISITION_BUFFER_LEN + sampleIndex ];
    }

    for( int sampleIndex = 0; sampleIndex < AQUISITION_BUFFER_LEN; sampleIndex++ )
      newFilteredSamplesList[ sampleIndex ] = data->filteredBuffer[ FILTER_EXTRA_SAMPLES_NUMBER + sampleIndex ];

    data->samplesBufferStartIndex += AQUISITION_BUFFER_LEN;

    return (double*) newFilteredSamplesList;
  }
  
  return NULL;
}

static void* AsyncUpdate( void* data )
{
  while( isProcessRunning )
  {
    for( khint_t sensorID = kh_begin( emgSensorsList ); sensorID != kh_end( emgSensorsList ); sensorID++ )
    {
      if( !kh_exist( emgSensorsList, sensorID ) ) continue;
      
      EMGSensor* sensor = &(kh_value( emgSensorsList, sensorID ));

      double* filteredSamplesList = GetFilteredSignal( sensor );
      
      EMGData* data = &(sensor->emgData);
      
      if( filteredSamplesList != NULL )
      {
        if( data->processingPhase == EMG_RELAXATION_PHASE || data->processingPhase == EMG_CONTRACTION_PHASE )
        {
          //DEBUG_PRINT( "adding emg filtered: %g", filteredSamplesList[ 0 ] / AQUISITION_BUFFER_LEN );
;
          for( size_t sampleIndex = 0; sampleIndex < AQUISITION_BUFFER_LEN; sampleIndex++ )
            data->processingResultsList[ data->processingPhase ] += ( filteredSamplesList[ sampleIndex ] / AQUISITION_BUFFER_LEN );
          
          //DEBUG_PRINT( "emg sum: %g", emgData->processingResultsList[ emgData->processingPhase ] );

          ThreadLock_Aquire( phasePassCountLock );
          data->preparationPassesCount++;
          ThreadLock_Release( phasePassCountLock );
        }
        else if( data->processingPhase == EMG_ACTIVATION_PHASE )
        {
          static double normalizedSample;
          
          for( size_t sampleIndex = 0; sampleIndex < AQUISITION_BUFFER_LEN; sampleIndex++ )
          {
            normalizedSample = 0.0;

            if( data->processingResultsList[ EMG_CONTRACTION_PHASE ] > 0.0 )
              normalizedSample = ( filteredSamplesList[ sampleIndex ] - data->processingResultsList[ EMG_RELAXATION_PHASE ] )
                                 / ( data->processingResultsList[ EMG_CONTRACTION_PHASE ] - data->processingResultsList[ EMG_RELAXATION_PHASE ] );

            if( normalizedSample > 1.0 ) normalizedSample = 1.0;
            else if( normalizedSample < 0.0 ) normalizedSample = 0.0;

            data->processingResultsList[ EMG_ACTIVATION_PHASE ] = ( exp( -2 * normalizedSample ) - 1 ) / ( exp( -2 ) - 1 );

            //if( channel == 0 && sampleIndex == 0 && channelNormalizedSamplesList[ sampleIndex ] > 0.0 )
            //  DEBUG_PRINT( "emg norm: %g - activ: %g", channelNormalizedSamplesList[ sampleIndex ], channelActivationsList[ sampleIndex ] );
          }
          
          //DEBUG_PRINT( "emg: (%g - %g)/(%g - %g) = %g", filteredSamplesList[ 0 ], emgData->processingResultsList[ EMG_RELAXATION_PHASE ],
          //                                              emgData->processingResultsList[ EMG_CONTRACTION_PHASE ], emgData->processingResultsList[ EMG_RELAXATION_PHASE ], normalizedSample );
        }
      }
    }
    
  }
  
  Thread_Exit( 0 );
  return NULL;
}

const char* CURVE_NAMES[ MUSCLE_CURVES_NUMBER ] = { "active_force", "passive_force", "moment_arm", "normalized_length" };
static EMGSensor* LoadEMGSensorData( const char* configFileName )
{
  DEBUG_PRINT( "Trying to load muscle %s EMG sensor data", configFileName );
  
  static EMGSensor emgSensorData;
  static char searchPath[ FILE_PARSER_MAX_PATH_LENGTH ];
  
  bool loadError = false;
  
  memset( &emgSensorData, 0, sizeof(EMGSensor) );
  
  FileParser parser = JSONParser;
  int configFileID = parser.OpenFile( configFileName );
  if( configFileID != -1 )
  {
    emgSensorData.taskID = SignalAquisition.InitTask( parser.GetStringValue( "interface" ), AQUISITION_BUFFER_LEN );
    if( emgSensorData.taskID == -1 ) loadError = true;
    emgSensorData.channel = parser.GetIntegerValue( "channel" );
    
    for( size_t curveIndex = 0; curveIndex < MUSCLE_CURVES_NUMBER; curveIndex++ )
    {
      for( size_t coeffIndex = 0; coeffIndex < MUSCLE_CURVE_ORDER; coeffIndex++ )
      {
        sprintf( searchPath, "muscle_properties.curves.%s.%u", CURVE_NAMES[ curveIndex ], coeffIndex );
        emgSensorData.muscleProperties.curvesList[ curveIndex ][ coeffIndex ] = parser.GetRealValue( searchPath );
      }
    }
    
    emgSensorData.muscleProperties.initialPenationAngle = parser.GetRealValue( "muscle_properties.penation_angle" );
    emgSensorData.muscleProperties.scaleFactor = parser.GetRealValue( "muscle_properties.scale_factor" );
    
    parser.CloseFile( configFileID );
  }
  else
  {
    DEBUG_PRINT( "configuration for muscle %s EMG sensor not found", configFileName );
    loadError = true;
  }
  
  if( loadError )
  {
    UnloadEMGSensorData( &emgSensorData );
    return NULL;
  }
    
  return &emgSensorData;
}

void UnloadEMGSensorData( EMGSensor* sensor )
{
  if( sensor == NULL ) return;
  
  SignalAquisition.EndTask( sensor->taskID );
}


#endif /* EMG_PROCESS_H */