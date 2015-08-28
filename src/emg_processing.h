#ifndef EMG_PROCESS_H
#define EMG_PROCESS_H

#include <math.h>
#include <stdbool.h>

#include "signal_aquisition.h"

#include "klib/khash.h"
#include "file_parsing/json_parser.h"

#include "debug/async_debug.h"

Thread_Handle emgThreadID;

enum EMGProccessPhase { EMG_ACTIVATION_PHASE, EMG_RELAXATION_PHASE, EMG_CONTRACTION_PHASE, EMG_PROCESSING_PHASES_NUMBER };

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
  unsigned int channel;
  double samplesBuffer[ HISTORY_BUFFER_LEN ];
  size_t samplesBufferStartIndex;
  double rectifiedBuffer[ FILTER_BUFFER_LEN ];
  double filteredBuffer[ FILTER_BUFFER_LEN ];
  unsigned int processingPhase;
  double processingResultsList[ EMG_PROCESSING_PHASES_NUMBER ];
  unsigned int preparationPassesCount;
}
EMGData;

ThreadLock phasePassCountLock;

static bool isRunning;

enum { ACTIVE_FORCE, PASSIVE_FORCE, MOMENT_ARM, NORM_LENGTH, MUSCLE_CURVES_NUMBER };
const size_t MUSCLE_CURVE_ORDER = 5;
typedef double MuscleCurve[ MUSCLE_CURVE_ORDER ];

const size_t MUSCLE_NAME_MAX_LEN = 20;
typedef struct _MuscleProperties
{
  char name[ MUSCLE_NAME_MAX_LEN ];
  EMGData emgData;
  MuscleCurve curvesFactorsList[ MUSCLE_CURVES_NUMBER ];
  double initialPenationAngle;
  double scaleFactor;
}
MuscleProperties;

KHASH_MAP_INIT_INT( Muscle, MuscleProperties )
static khash_t(MuscleProperties)* musclePropertiesList = NULL;

static int Init( const char* );
static void End( int );
static double GetActivation( int );
static double GetMuscleForce( int );
static void ChangePhase( int, enum EMGProccessPhase );

const struct
{
  int (*Init)( const char* );
  int (*End)( const char* );
  double (*GetActivation)( int );
  double (*GetMuscleForce)( int );
  void (*ChangePhase)( int, enum EMGProccessPhase );
}
EMGSensor = { .Init = Init, .End = End, .GetActivation = GetActivation, .GetMuscleForce = GetMuscleForce, .ChangePhase = ChangePhase };

static MuscleProperties* LoadMuscleData( const char* );
static void UnloadMuscleData( MuscleProperties* );

static void* AsyncUpdate( void* );
static double* GetFilteredSignal( EMGData* );

static MuscleProperties* LoadMuscleData( const char* configFileName )
{
  DEBUG_PRINT( "Trying to load muscle %s EMG sensor data", configFileName );
  
  static MuscleProperties musclePropertiesData;
  
  bool loadError = false;
  
  memset( &musclePropertiesData, 0, sizeof(musclePropertiesData) );
  
  FileParser parser = JSONParser;
  int configFileID = parser.OpenFile( configFileName );
  if( configFileID != -1 )
  {
    
    parser.CloseFile( configFileID );
  }
  {
    DEBUG_PRINT( "configuration for muscle %s EMG sensor not found", configFileName );
    loadError = true;
  }
  
  struct FILE* configFile = fopen( "../config/muscles_properties.txt", "r" );
  if( configFile != NULL ) 
  {
    while( fscanf( configFile, "%s", readBuffer ) != EOF )
    {
      if( strcmp( readBuffer, "BEGIN_MUSCLE" ) == 0 )
      {
        musclePropertiesList = (MuscleProperties*) realloc( musclePropertiesList, sizeof(MuscleProperties) * ( musclesNumber + 1 ) );
  
        newMuscleProperties = &(musclePropertiesList[ musclesNumber ]);
        
        fscanf( configFile, "%s", newMuscleProperties->name );
  
        DEBUG_EVENT( 0, "found muscle %s", newMuscleProperties->name );
        
        newMuscleProperties->initialPenationAngle = 0.0;
        newMuscleProperties->scaleFactor = 1.0;
        
        newMuscleProperties->emgData.channel = 0;
        newMuscleProperties->emgData.processingPhase = EMG_ACTIVATION_PHASE;
        for( size_t processingPhase = 0; processingPhase < EMG_PROCESSING_PHASES_NUMBER; processingPhase++ )
          newMuscleProperties->emgData.processingResultsList[ processingPhase ] = 0.0;
      }
      
      if( newMuscleProperties == NULL ) continue;
      
      if( strcmp( readBuffer, "emg_channel:" ) == 0 )
      {
        fscanf( configFile, "%u", &(newMuscleProperties->emgData.channel) );
        
        DEBUG_EVENT( 8, "found muscle %s EMG channel", newMuscleProperties->name );
      }
      else if( strcmp( readBuffer, "active_force_curve:" ) == 0 )
      {
        for( size_t factorIndex = 0; factorIndex < MUSCLE_CURVE_ORDER; factorIndex++ )
          fscanf( configFile, "%lf", &(newMuscleProperties->curvesFactorsList[ ACTIVE_FORCE ][ factorIndex ]) );
        
        DEBUG_EVENT( 1, "found muscle %s active force curve", newMuscleProperties->name );
      }
      else if( strcmp( readBuffer, "passive_force_curve:" ) == 0 )
      {
        for( size_t factorIndex = 0; factorIndex < MUSCLE_CURVE_ORDER; factorIndex++ )
          fscanf( configFile, "%lf", &(newMuscleProperties->curvesFactorsList[ PASSIVE_FORCE ][ factorIndex ]) );
        
        DEBUG_EVENT( 2, "found muscle %s passive force curve", newMuscleProperties->name );
      }
      else if( strcmp( readBuffer, "moment_arm_curve:" ) == 0 )
      {
        for( size_t factorIndex = 0; factorIndex < MUSCLE_CURVE_ORDER; factorIndex++ )
          fscanf( configFile, "%lf", &(newMuscleProperties->curvesFactorsList[ MOMENT_ARM ][ factorIndex ]) );
        
        DEBUG_EVENT( 3, "found muscle %s moment arm curve", newMuscleProperties->name );
      }
      else if( strcmp( readBuffer, "norm_length_curve:" ) == 0 )
      {
        for( size_t factorIndex = 0; factorIndex < MUSCLE_CURVE_ORDER; factorIndex++ )
          fscanf( configFile, "%lf", &(newMuscleProperties->curvesFactorsList[ NORM_LENGTH ][ factorIndex ]) );
        
        DEBUG_EVENT( 4, "found muscle %s normalized length curve", newMuscleProperties->name );
      }
      else if( strcmp( readBuffer, "penation_angle:" ) == 0 )
      {
        fscanf( configFile, "%lf", &(newMuscleProperties->initialPenationAngle) );
        
        DEBUG_EVENT( 5, "found muscle %s penation angle", newMuscleProperties->name );
      }
      else if( strcmp( readBuffer, "scale_factor:" ) == 0 )
      {
        fscanf( configFile, "%lf", &(newMuscleProperties->scaleFactor) );
        
        DEBUG_EVENT( 6, "found muscle %s scale factor", newMuscleProperties->name );
      }
      else if( strcmp( readBuffer, "END_MUSCLE" ) == 0 )
      {
        if( newMuscleProperties->emgData.channel < SignalAquisition_GetChannelsNumber() )
        {
          musclesNumber++;
        
          DEBUG_EVENT( 7, "muscle %s properties added", newMuscleProperties->name );
        }
      }
      else if( strcmp( readBuffer, "#" ) == 0 )
      {
        char dummyChar;
        
        do { 
          if( fscanf( configFile, "%c", &dummyChar ) == EOF ) 
            break; 
        } while( dummyChar != '\n' ); 
      }
    }
    
    fclose( configFile );
  }
}

static double* EMGProcessing_GetFilteredSignal( EMGData* );
static void* EMGProcessing_AsyncUpdate( void* );

void EMGProcessing_Init()
{
  if( SignalAquisition_Init( "EMGReadTask", AQUISITION_BUFFER_LEN ) < 0 ) return;
  
  LoadMusclesProperties();
  
  phasePassCountLock = ThreadLock_Create();
  
  isRunning = true;
  emgThreadID = Thread_Start( EMGProcessing_AsyncUpdate, NULL, THREAD_JOINABLE );
}

void EMGProcessing_End()
{
  isRunning = false;
  
  (void) Thread_WaitExit( emgThreadID, 5000 );
  
  if( musclePropertiesList != NULL ) free( musclePropertiesList );
  
  ThreadLock_Discard( phasePassCountLock );
  
  SignalAquisition_End();
}

int EMGProcessing_GetMuscleID( const char* muscleName )
{
  for( size_t muscleID = 0; muscleID < musclesNumber; muscleID++ )
  {
    if( strcmp( musclePropertiesList[ muscleID ].name, muscleName ) == 0 )
      return (int) muscleID;
  }
  
  return -1;
}

static double* EMGProcessing_GetFilteredSignal( EMGData* emgData )
{
  const double DATA_AQUISITION_SCALE_FACTOR = 909.0;
  
  static double newFilteredSamplesList[ AQUISITION_BUFFER_LEN ];
  
  double* newSamplesList = SignalAquisition_Read( emgData->channel );
  
  if( newSamplesList != NULL ) 
  {
    size_t oldSamplesBufferStart = emgData->samplesBufferStartIndex;
    size_t newSamplesBufferStart = oldSamplesBufferStart + OLD_SAMPLES_BUFFER_LEN;
    for( int newSampleIndex = 0; newSampleIndex < AQUISITION_BUFFER_LEN; newSampleIndex++ )
      emgData->samplesBuffer[ ( newSamplesBufferStart + newSampleIndex ) % HISTORY_BUFFER_LEN ] = newSamplesList[ newSampleIndex ] / DATA_AQUISITION_SCALE_FACTOR;
    
    static double previousSamplesMean;
    for( int sampleIndex = 0; sampleIndex < AQUISITION_BUFFER_LEN; sampleIndex++ )
    {
      previousSamplesMean = 0.0;
      for( int previousSampleIndex = oldSamplesBufferStart + sampleIndex; previousSampleIndex < newSamplesBufferStart + sampleIndex; previousSampleIndex++ )
        previousSamplesMean += emgData->samplesBuffer[ previousSampleIndex % HISTORY_BUFFER_LEN ] / OLD_SAMPLES_BUFFER_LEN;

      emgData->rectifiedBuffer[ FILTER_EXTRA_SAMPLES_NUMBER + sampleIndex ] = fabs( emgData->samplesBuffer[ ( newSamplesBufferStart + sampleIndex ) % HISTORY_BUFFER_LEN ] - previousSamplesMean );
    }
    
    for( int sampleIndex = FILTER_EXTRA_SAMPLES_NUMBER; sampleIndex < FILTER_BUFFER_LEN; sampleIndex++ )
    {
      emgData->filteredBuffer[ sampleIndex ] = 0.0;
      for( int factorIndex = 0; factorIndex < FILTER_ORDER; factorIndex++ )
      {
        emgData->filteredBuffer[ sampleIndex ] -= filter_A[ factorIndex ] * emgData->filteredBuffer[ sampleIndex - factorIndex ];
        emgData->filteredBuffer[ sampleIndex ] += filter_B[ factorIndex ] * emgData->rectifiedBuffer[ sampleIndex - factorIndex ];
      }
      if( emgData->filteredBuffer[ sampleIndex ] < 0.0 ) emgData->filteredBuffer[ sampleIndex ] = 0.0;
    }
    
    for( int sampleIndex = 0; sampleIndex < FILTER_EXTRA_SAMPLES_NUMBER; sampleIndex++ )
    {
      emgData->filteredBuffer[ sampleIndex ] = emgData->filteredBuffer[ AQUISITION_BUFFER_LEN + sampleIndex ];
      emgData->rectifiedBuffer[ sampleIndex ] = emgData->rectifiedBuffer[ AQUISITION_BUFFER_LEN + sampleIndex ];
    }

    for( int sampleIndex = 0; sampleIndex < AQUISITION_BUFFER_LEN; sampleIndex++ )
      newFilteredSamplesList[ sampleIndex ] = emgData->filteredBuffer[ FILTER_EXTRA_SAMPLES_NUMBER + sampleIndex ];

    emgData->samplesBufferStartIndex += AQUISITION_BUFFER_LEN;

    return (double*) newFilteredSamplesList;
  }
  
  return NULL;
}

static void* EMGProcessing_AsyncUpdate( void* data )
{
  while( isRunning )
  {
    for( size_t muscleID = 0; muscleID < musclesNumber; muscleID++ )
    {
      EMGData* emgData = &(musclePropertiesList[ muscleID ].emgData);

      double* filteredSamplesList = EMGProcessing_GetFilteredSignal( emgData );
      
      if( filteredSamplesList != NULL )
      {
        if( emgData->processingPhase == EMG_RELAXATION_PHASE || emgData->processingPhase == EMG_CONTRACTION_PHASE )
        {
          //DEBUG_PRINT( "adding emg filtered: %g", filteredSamplesList[ 0 ] / AQUISITION_BUFFER_LEN );
;
          for( size_t sampleIndex = 0; sampleIndex < AQUISITION_BUFFER_LEN; sampleIndex++ )
            emgData->processingResultsList[ emgData->processingPhase ] += ( filteredSamplesList[ sampleIndex ] / AQUISITION_BUFFER_LEN );
          
          //DEBUG_PRINT( "emg sum: %g", emgData->processingResultsList[ emgData->processingPhase ] );

          ThreadLock_Aquire( phasePassCountLock );
          emgData->preparationPassesCount++;
          ThreadLock_Release( phasePassCountLock );
        }
        else if( emgData->processingPhase == EMG_ACTIVATION_PHASE )
        {
          static double normalizedSample;
          
          for( size_t sampleIndex = 0; sampleIndex < AQUISITION_BUFFER_LEN; sampleIndex++ )
          {
            normalizedSample = 0.0;

            if( emgData->processingResultsList[ EMG_CONTRACTION_PHASE ] > 0.0 )
              normalizedSample = ( filteredSamplesList[ sampleIndex ] - emgData->processingResultsList[ EMG_RELAXATION_PHASE ] )
                                 / ( emgData->processingResultsList[ EMG_CONTRACTION_PHASE ] - emgData->processingResultsList[ EMG_RELAXATION_PHASE ] );

            if( normalizedSample > 1.0 ) normalizedSample = 1.0;
            else if( normalizedSample < 0.0 ) normalizedSample = 0.0;

            emgData->processingResultsList[ EMG_ACTIVATION_PHASE ] = ( exp( -2 * normalizedSample ) - 1 ) / ( exp( -2 ) - 1 );

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

void EMGProcessing_ChangePhase( int muscleID, int newProcessingPhase )
{
  if( muscleID < 0 || muscleID >= (int) musclesNumber ) return;
  
  if( newProcessingPhase < 0 || newProcessingPhase >= EMG_PROCESSING_PHASES_NUMBER ) return;
  
  ThreadLock_Aquire( phasePassCountLock );
  
  EMGData* emgData = &(musclePropertiesList[ muscleID ].emgData);

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

extern inline double EMGProcessing_GetMuscleActivation( int muscleID )
{
  if( muscleID < 0 || muscleID >= (int) musclesNumber ) return 0.0;
  
  return musclePropertiesList[ muscleID ].emgData.processingResultsList[ EMG_ACTIVATION_PHASE ];
}

double EMGProcessing_GetMuscleTorque( int muscleID, double jointAngle )
{
  static double measuresList[ MUSCLE_CURVES_NUMBER ];
  static double penationAngle;
  
  if( muscleID < 0 || muscleID >= (int) musclesNumber ) return 0.0;
  
  MuscleProperties* muscleProperties = &(musclePropertiesList[ muscleID ]);
  
  for( size_t curveIndex = 0; curveIndex < MUSCLE_CURVES_NUMBER; curveIndex++ )
  {
    measuresList[ curveIndex ] = 0.0;
    for( size_t factorIndex = 0; factorIndex < MUSCLE_CURVE_ORDER; factorIndex++ )
      measuresList[ curveIndex ] += muscleProperties->curvesFactorsList[ curveIndex ][ factorIndex ] * pow( jointAngle, factorIndex );
  }

  penationAngle = asin( sin( muscleProperties->initialPenationAngle ) / measuresList[ NORM_LENGTH ] );
  
  //if( muscleProperties->processingPhase == EMG_ACTIVATION_PHASE )
  //  DEBUG_PRINT( "theta: %lf - active force: %lf - passive force: %lf - moment arm: %lf - normalized length: %lf - penation angle: %lf",
  //               jointAngle, measuresList[ ACTIVE_FORCE ], measuresList[ PASSIVE_FORCE ], measuresList[ MOMENT_ARM ], measuresList[ NORM_LENGTH ], penationAngle );
  
  double resultingForce = muscleProperties->scaleFactor 
                          * ( measuresList[ ACTIVE_FORCE ] * muscleProperties->emgData.processingResultsList[ EMG_ACTIVATION_PHASE ] + measuresList[ PASSIVE_FORCE ] ) 
                          * cos( penationAngle );
  
  return resultingForce * measuresList[ MOMENT_ARM ];
}


#endif /* EMG_PROCESS_H */
