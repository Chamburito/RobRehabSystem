#ifndef EMG_PROCESS_H
#define EMG_PROCESS_H

#include <math.h>
#include <stdbool.h>

#ifdef _CVI_
  #include "signal_aquisition_cvi.h"
#elif
  #include "signal_aquisition_ttl.h"
#endif

#include "async_debug.h"

Thread_Handle emgThreadID;

enum { EMG_RELAXATION_PHASE, EMG_CONTRACTION_PHASE, EMG_ACTIVATION_PHASE, EMG_PROCESSING_PHASES_NUMBER };

ThreadLock phasePassCountLock;

static bool isRunning;

enum { ACTIVE_FORCE, PASSIVE_FORCE, MOMENT_ARM, NORM_LENGTH, MUSCLE_CURVES_NUMBER };
const size_t MUSCLE_CURVE_ORDER = 5;
typedef double MuscleCurve[ MUSCLE_CURVE_ORDER ];

const size_t MUSCLE_NAME_MAX_LEN = 20;
typedef struct _MuscleProperties
{
  char name[ MUSCLE_NAME_MAX_LEN ];
  unsigned int emgChannel;
  MuscleCurve curvesFactorsList[ MUSCLE_CURVES_NUMBER ];
  double emgValuesList[ EMG_PROCESSING_PHASES_NUMBER ];
  unsigned int processingPhase, preparationPassesCount;
  double initialPenationAngle;
  double scaleFactor;
}
MuscleProperties;

static MuscleProperties* musclePropertiesList = NULL;
static size_t musclesNumber = 0;

static void LoadMusclesProperties()
{
  char readBuffer[ 64 ];
  
  MuscleProperties* newMuscleProperties = NULL;
  
  FILE* configFile = fopen( "../config/muscles_properties.txt", "r" );
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
        
        newMuscleProperties->emgChannel = 0;
        newMuscleProperties->initialPenationAngle = 0.0;
        newMuscleProperties->scaleFactor = 1.0;
        
        for( size_t emgValueIndex = 0; emgValueIndex < EMG_PROCESSING_PHASES_NUMBER; emgValueIndex++ )
          newMuscleProperties->emgValuesList[ emgValueIndex ] = 0.0;
        
        newMuscleProperties->processingPhase = EMG_ACTIVATION_PHASE;
        newMuscleProperties->preparationPassesCount = 0;
      }
      
      if( newMuscleProperties == NULL ) continue;
      
      if( strcmp( readBuffer, "emg_channel:" ) == 0 )
      {
        fscanf( configFile, "%u", &(newMuscleProperties->emgChannel) );
        
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
        musclesNumber++;
        
        DEBUG_EVENT( 7, "muscle %s properties added", newMuscleProperties->name );
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

const size_t AQUISITION_BUFFER_LEN = 10;
typedef double AquisitionBuffer[ AQUISITION_BUFFER_LEN ];
static AquisitionBuffer* newFilteredSamplesList = NULL;

const size_t OLD_SAMPLES_BUFFER_LEN = 100;
const size_t HISTORY_BUFFER_LEN = OLD_SAMPLES_BUFFER_LEN + AQUISITION_BUFFER_LEN;
typedef double HistoryBuffer[ HISTORY_BUFFER_LEN ];
static HistoryBuffer* samplesBuffer = NULL;

const size_t FILTER_ORDER = 3;
const double filter_A[ FILTER_ORDER ] = { 1.0, -1.9964, 0.9965 };
const double filter_B[ FILTER_ORDER ] = { 1.576e-6, 3.153e-6, 1.576e-6 };

const int FILTER_EXTRA_SAMPLES_NUMBER = FILTER_ORDER - 1;
const int FILTER_BUFFER_LEN = FILTER_EXTRA_SAMPLES_NUMBER + AQUISITION_BUFFER_LEN;

typedef double FilterBuffer[ FILTER_BUFFER_LEN ];
static FilterBuffer* rectifiedBuffer = NULL;
static FilterBuffer* filteredBuffer = NULL;

static AquisitionBuffer* EMGProcessing_GetFilteredSignal();
static void* EMGProcessing_AsyncUpdate( void* );

void EMGProcessing_Init()
{
  if( SignalAquisition_Init( AQUISITION_BUFFER_LEN ) < 0 ) return;
  
  LoadMusclesProperties();
  
  phasePassCountLock = ThreadLock_Create();
  
  size_t emgChannelsNumber = SignalAquisition_GetChannelsNumber();
  
  newFilteredSamplesList = (AquisitionBuffer*) calloc( emgChannelsNumber, sizeof(AquisitionBuffer) );
  samplesBuffer = (HistoryBuffer*) calloc( emgChannelsNumber, sizeof(HistoryBuffer) );
  rectifiedBuffer = (FilterBuffer*) calloc( emgChannelsNumber, sizeof(FilterBuffer) );
  filteredBuffer = (FilterBuffer*) calloc( emgChannelsNumber, sizeof(FilterBuffer) );
  
  isRunning = true;
  emgThreadID = Thread_Start( EMGProcessing_AsyncUpdate, NULL, JOINABLE );
}

void EMGProcessing_End()
{
  isRunning = false;
  
  (void) Thread_WaitExit( emgThreadID, 5000 );
  
  ThreadLock_Discard( phasePassCountLock );
  
  if( musclePropertiesList != NULL ) free( musclePropertiesList );
  
  if( newFilteredSamplesList != NULL ) free( newFilteredSamplesList );
  if( samplesBuffer != NULL ) free( samplesBuffer );
  if( rectifiedBuffer != NULL ) free( rectifiedBuffer );
  if( filteredBuffer != NULL ) free( filteredBuffer );
  
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

static AquisitionBuffer* EMGProcessing_GetFilteredSignal()
{
  const double DATA_AQUISITION_SCALE_FACTOR = 909.0;
  
  static int oldSamplesBufferStart = 0, newSamplesBufferStart = OLD_SAMPLES_BUFFER_LEN;
  
  static double previousSamplesMean;
  
  double** newSamplesList = SignalAquisition_Read( SIGNAL_AQUISITION_INFINITE_TIMEOUT, NULL );
  
  if( newSamplesList != NULL ) 
  {
    for( size_t muscleID = 0; muscleID < musclesNumber; muscleID++ )
    {
      MuscleProperties* muscleProperties = &(musclePropertiesList[ muscleID ]);
      
      unsigned int channel = muscleProperties->emgChannel;
      
      if( channel >= SignalAquisition_GetChannelsNumber() ) continue;

      for( int newSampleIndex = 0; newSampleIndex < AQUISITION_BUFFER_LEN; newSampleIndex++ )
        samplesBuffer[ channel ][ ( newSamplesBufferStart + newSampleIndex ) % HISTORY_BUFFER_LEN ] = newSamplesList[ channel ][ newSampleIndex ] / DATA_AQUISITION_SCALE_FACTOR;
      
      previousSamplesMean = 0.0;
      for( int sampleIndex = 0; sampleIndex < AQUISITION_BUFFER_LEN; sampleIndex++ )
      {
        for( int previousSampleIndex = oldSamplesBufferStart + sampleIndex; previousSampleIndex < newSamplesBufferStart + sampleIndex; previousSampleIndex++ )
          previousSamplesMean += samplesBuffer[ channel ][ previousSampleIndex % HISTORY_BUFFER_LEN ] / OLD_SAMPLES_BUFFER_LEN;
        
        rectifiedBuffer[ channel ][ FILTER_EXTRA_SAMPLES_NUMBER + sampleIndex ] = fabs( samplesBuffer[ channel ][ ( newSamplesBufferStart + sampleIndex ) % HISTORY_BUFFER_LEN ] 
                                                                                  - previousSamplesMean );
      }

      for( int sampleIndex = FILTER_EXTRA_SAMPLES_NUMBER; sampleIndex < FILTER_BUFFER_LEN; sampleIndex++ )
      {
        filteredBuffer[ channel ][ sampleIndex ] = 0.0;
        for( int factorIndex = 0; factorIndex < FILTER_ORDER; factorIndex++ )
        {
          filteredBuffer[ channel ][ sampleIndex ] -= filter_A[ factorIndex ] * filteredBuffer[ channel ][ sampleIndex - factorIndex ];
          filteredBuffer[ channel ][ sampleIndex ] += filter_B[ factorIndex ] * rectifiedBuffer[ channel ][ sampleIndex - factorIndex ];
        }
        if( filteredBuffer[ channel ][ sampleIndex ] < 0.0 ) filteredBuffer[ channel ][ sampleIndex ] = 0.0;
      }

      for( int sampleIndex = 0; sampleIndex < FILTER_EXTRA_SAMPLES_NUMBER; sampleIndex++ )
      {
        filteredBuffer[ channel ][ sampleIndex ] = filteredBuffer[ channel ][ AQUISITION_BUFFER_LEN + sampleIndex ];
        rectifiedBuffer[ channel ][ sampleIndex ] = rectifiedBuffer[ channel ][ AQUISITION_BUFFER_LEN + sampleIndex ];
      }

      for( int sampleIndex = 0; sampleIndex < AQUISITION_BUFFER_LEN; sampleIndex++ )
        newFilteredSamplesList[ channel ][ sampleIndex ] = filteredBuffer[ channel ][ FILTER_EXTRA_SAMPLES_NUMBER + sampleIndex ];
    }

    oldSamplesBufferStart += AQUISITION_BUFFER_LEN;
    newSamplesBufferStart += AQUISITION_BUFFER_LEN;

    return newFilteredSamplesList;
  }
  
  return NULL;
  
}

static void* EMGProcessing_AsyncUpdate( void* referenceData )
{
  while( isRunning )
  {
    AquisitionBuffer* filteredSamplesList = EMGProcessing_GetFilteredSignal();
    
    if( filteredSamplesList != NULL )
    {
      for( size_t muscleID = 0; muscleID < musclesNumber; muscleID++ )
      {
        MuscleProperties* muscleProperties = &(musclePropertiesList[ muscleID ]);
      
        unsigned int channel = muscleProperties->emgChannel;
      
        if( channel >= SignalAquisition_GetChannelsNumber() ) continue;
        
        if( muscleProperties->processingPhase == EMG_RELAXATION_PHASE || muscleProperties->processingPhase == EMG_CONTRACTION_PHASE )
        {
          DEBUG_PRINT( "emg filtered: %g", filteredSamplesList[ channel ][ 0 ] );
          
          for( size_t sampleIndex = 0; sampleIndex < AQUISITION_BUFFER_LEN; sampleIndex++ )
            muscleProperties->emgValuesList[ muscleProperties->processingPhase ] += filteredSamplesList[ channel ][ sampleIndex ] / AQUISITION_BUFFER_LEN;

          ThreadLock_Aquire( phasePassCountLock );
          muscleProperties->preparationPassesCount++;
          ThreadLock_Release( phasePassCountLock );
        }
        else if( muscleProperties->processingPhase == EMG_ACTIVATION_PHASE )
        {
          static double normalizedSample;

          for( size_t sampleIndex = 0; sampleIndex < AQUISITION_BUFFER_LEN; sampleIndex++ )
          {
            normalizedSample = 0.0;

            if( muscleProperties->emgValuesList[ EMG_CONTRACTION_PHASE ] > 0.0 )
              normalizedSample = ( filteredSamplesList[ channel ][ sampleIndex ] - muscleProperties->emgValuesList[ EMG_RELAXATION_PHASE ] )
                                 / ( muscleProperties->emgValuesList[ EMG_CONTRACTION_PHASE ] - muscleProperties->emgValuesList[ EMG_RELAXATION_PHASE ] );

            if( normalizedSample > 1.0 ) normalizedSample = 1.0;
            else if( normalizedSample < 0.0 ) normalizedSample = 0.0;

            muscleProperties->emgValuesList[ EMG_ACTIVATION_PHASE ] = ( exp( -2 * normalizedSample ) - 1 ) / ( exp( -2 ) - 1 );

            //if( channel == 0 && sampleIndex == 0 && channelNormalizedSamplesList[ sampleIndex ] > 0.0 )
            //  DEBUG_PRINT( "emg norm: %g - activ: %g", channelNormalizedSamplesList[ sampleIndex ], channelActivationsList[ sampleIndex ] );
          }

          //DEBUG_PRINT( "emg: (%g - %g)/(%g - %g) = %g", filteredSamplesList[ 1 ][ 0 ], muscleProperties->emgValuesList[ EMG_RELAXATION_PHASE ],
          //                                              muscleProperties->emgValuesList[ EMG_CONTRACTION_PHASE ], muscleProperties->emgValuesList[ EMG_RELAXATION_PHASE ], normalizedSample );
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
  
  MuscleProperties* muscleProperties = &(musclePropertiesList[ muscleID ]);

  unsigned int channel = muscleProperties->emgChannel;

  if( channel >= SignalAquisition_GetChannelsNumber() ) return;

  if( muscleProperties->processingPhase == EMG_RELAXATION_PHASE || muscleProperties->processingPhase == EMG_CONTRACTION_PHASE )
  {
    if( muscleProperties->preparationPassesCount > 0 )
    {
      DEBUG_PRINT( "new %s value: %g / %u = %g", ( muscleProperties->processingPhase == EMG_RELAXATION_PHASE ) ? "min" : "max", 
                                                 muscleProperties->emgValuesList[ muscleProperties->processingPhase ],
                                                 muscleProperties->preparationPassesCount, 
                                                 muscleProperties->emgValuesList[ muscleProperties->processingPhase ] / muscleProperties->preparationPassesCount );

      muscleProperties->emgValuesList[ muscleProperties->processingPhase ] /= muscleProperties->preparationPassesCount;
    }

    muscleProperties->preparationPassesCount = 0;
  }

  muscleProperties->emgValuesList[ newProcessingPhase ] = 0.0;
  
  muscleProperties->processingPhase = newProcessingPhase;
  
  ThreadLock_Release( phasePassCountLock );
}

extern inline double EMGProcessing_GetMuscleActivation( int muscleID )
{
  if( muscleID < 0 || muscleID >= (int) musclesNumber ) return 0.0;
  
  return musclePropertiesList[ muscleID ].emgValuesList[ EMG_ACTIVATION_PHASE ];
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
                          * ( measuresList[ ACTIVE_FORCE ] * muscleProperties->emgValuesList[ EMG_ACTIVATION_PHASE ] + measuresList[ PASSIVE_FORCE ] ) 
                          * cos( penationAngle );
  
  return resultingForce * measuresList[ MOMENT_ARM ];
}


#endif /* EMG_PROCESS_H */
