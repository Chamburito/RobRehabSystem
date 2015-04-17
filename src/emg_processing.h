#ifndef EMG_PROCESS_H
#define EMG_PROCESS_H

#include <NIDAQmx.h>

#include <math.h>
#include <stdbool.h>

#include "async_debug.h"

#ifndef _CVI_
  typedef double float64;
#endif

TaskHandle emgReadTask;

Thread_Handle emgThreadID;

const size_t EMG_CHANNELS_NUMBER = 2;

enum { EMG_RELAXATION_PHASE, EMG_CONTRACTION_PHASE, EMG_WORK_PHASE, EMG_PROCESSING_PHASES_NUMBER };
static int processingPhase;

static float64 averageActivationsList[ EMG_CHANNELS_NUMBER ];

static struct
{
  float64 minList[ EMG_CHANNELS_NUMBER ];
  float64 maxList[ EMG_CHANNELS_NUMBER ];
}
emgReference;

static unsigned long preparationPassCount = 0;
static unsigned long workingPassCount = 0;
ThreadLock phasePassCountLock;

static bool isRunning;

const int COEFFS_NUMBER = 5;
const int MUSCLE_NAME_MAX_LEN = 20;
typedef struct _MuscleProperties
{
  char name[ MUSCLE_NAME_MAX_LEN ];
  unsigned int emgChannel;
  double activeForceCoeffs[ COEFFS_NUMBER ];
  double passiveForceCoeffs[ COEFFS_NUMBER ];
  double momentArmCoeffs[ COEFFS_NUMBER ];
  double normLengthCoeffs[ COEFFS_NUMBER ];
  double penationAngle;
  double scaleFactor;
}
MuscleProperties;

static MuscleProperties* musclePropertiesList = NULL;
static size_t musclesNumber = 0;

enum { MUSCLE_ACTIVE_FORCE, MUSCLE_PASSIVE_FORCE, MUSCLE_MOMENT_ARM, MUSCLE_NORM_LENGTH, MUSCLE_MEASURES_NUMBER };

static void LoadMusclesProperties()
{
  char readBuffer[ MUSCLE_NAME_MAX_LEN ];
  
  MuscleProperties* newMuscleProperties = NULL;
  
  FILE* configFile = fopen( "../config/muscles_properties.txt", "r" );
  if( configFile != NULL ) 
  {
    while( fscanf( configFile, "%s", readBuffer ) != EOF )
    {
      if( strcmp( readBuffer, "MUSCLE" ) == 0 )
      {
        musclePropertiesList = (MuscleProperties*) realloc( musclePropertiesList, sizeof(MuscleProperties) * ( musclesNumber + 1 ) );
  
        newMuscleProperties = &(musclePropertiesList[ musclesNumber ]);
        
        fscanf( configFile, "%s", newMuscleProperties->name );
  
        DEBUG_EVENT( 0, "found muscle %s", newMuscleProperties->name );
        
        newMuscleProperties->scaleFactor = 0.0;
        newMuscleProperties->scaleFactor = 1.0;
      }
      
      if( newMuscleProperties == NULL ) continue;
      
      if( strcmp( readBuffer, "ACTIVE_FORCE_CURVE" ) == 0 )
      {
        for( size_t coeffIndex = 0; coeffIndex < COEFFS_NUMBER; coeffIndex++ )
          fscanf( configFile, "%lf", &(newMuscleProperties->activeForceCoeffs[ coeffIndex ]) );
        
        DEBUG_EVENT( 1, "found muscle %s active force curve", newMuscleProperties->name );
      }
      else if( strcmp( readBuffer, "PASSIVE_FORCE_CURVE" ) == 0 )
      {
        for( size_t coeffIndex = 0; coeffIndex < COEFFS_NUMBER; coeffIndex++ )
          fscanf( configFile, "%lf", &(newMuscleProperties->passiveForceCoeffs[ coeffIndex ]) );
        
        DEBUG_EVENT( 2, "found muscle %s passive force curve", newMuscleProperties->name );
      }
      else if( strcmp( readBuffer, "MOMENT_ARM_CURVE" ) == 0 )
      {
        for( size_t coeffIndex = 0; coeffIndex < COEFFS_NUMBER; coeffIndex++ )
          fscanf( configFile, "%lf", &(newMuscleProperties->momentArmCoeffs[ coeffIndex ]) );
        
        DEBUG_EVENT( 3, "found muscle %s moment arm curve", newMuscleProperties->name );
      }
      else if( strcmp( readBuffer, "NORM_LENGTH_CURVE" ) == 0 )
      {
        for( size_t coeffIndex = 0; coeffIndex < COEFFS_NUMBER; coeffIndex++ )
          fscanf( configFile, "%lf", &(newMuscleProperties->normLengthCoeffs[ coeffIndex ]) );
        
        DEBUG_EVENT( 4, "found muscle %s normalized length curve", newMuscleProperties->name );
      }
      else if( strcmp( readBuffer, "PENATION_ANGLE" ) == 0 )
      {
        fscanf( configFile, "%lf", &(newMuscleProperties->penationAngle) );
        
        DEBUG_EVENT( 5, "found muscle %s penation angle", newMuscleProperties->name );
      }
      else if( strcmp( readBuffer, "SCALE_FACTOR" ) == 0 )
      {
        fscanf( configFile, "%lf", &(newMuscleProperties->scaleFactor) );
        
        DEBUG_EVENT( 6, "found muscle %s scale factor", newMuscleProperties->name );
      }
      else if( strcmp( readBuffer, "ENDMUSCLE" ) == 0 )
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

static float64* EMGProcessing_GetFilteredSignal();
static void* EMGProcessing_AsyncUpdate( void* );

void EMGProcessing_Init()
{
  DAQmxLoadTask( "EMGReadTask", &emgReadTask );
  DAQmxStartTask( emgReadTask );
  
  LoadMusclesProperties();
  
  processingPhase = EMG_WORK_PHASE;
  
  phasePassCountLock = ThreadLock_Create();
  
  isRunning = true;
  emgThreadID = Thread_Start( EMGProcessing_AsyncUpdate, NULL, JOINABLE );
}

void EMGProcessing_End()
{
  isRunning = false;
  
  (void) Thread_WaitExit( emgThreadID, 5000 );
  
  ThreadLock_Discard( phasePassCountLock );
  
  DAQmxStopTask( emgReadTask );
  DAQmxClearTask( emgReadTask );
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

const int NEW_SAMPLES_BUFFER_LEN = 10;
const size_t FILTER_ORDER = 3;
const float64 filter_A[ FILTER_ORDER ] = { 1.0, -1.9964, 0.9965 };
const float64 filter_B[ FILTER_ORDER ] = { 1.576e-6, 3.153e-6, 1.576e-6 };
static float64* EMGProcessing_GetFilteredSignal()
{
  const int FILTER_EXTRA_SAMPLES_NUMBER = FILTER_ORDER - 1;
  const int FILTER_BUFFER_LEN = FILTER_EXTRA_SAMPLES_NUMBER + NEW_SAMPLES_BUFFER_LEN;
  
  const int OLD_SAMPLES_BUFFER_LEN = 100;
  const int SAMPLES_BUFFER_LEN = OLD_SAMPLES_BUFFER_LEN + NEW_SAMPLES_BUFFER_LEN;
  
  const float64 DATA_AQUISITION_SCALE_FACTOR = 909.0;
  
  static float64 samplesList[ EMG_CHANNELS_NUMBER * SAMPLES_BUFFER_LEN ];
  static int oldSamplesBufferStart = 0, newSamplesBufferStart = OLD_SAMPLES_BUFFER_LEN;
  
  static float64 newSamplesList[ EMG_CHANNELS_NUMBER * NEW_SAMPLES_BUFFER_LEN ];
  static float64 rectifiedSamplesList[ EMG_CHANNELS_NUMBER * FILTER_BUFFER_LEN ];
  static float64 filteredSamplesList[ EMG_CHANNELS_NUMBER * FILTER_BUFFER_LEN ];
  static float64 processedSamplesList[ EMG_CHANNELS_NUMBER * NEW_SAMPLES_BUFFER_LEN ];
  static float64 previousSamplesMean;
  static int32 aquiredSamples;
  
  int errorCode = DAQmxReadAnalogF64( emgReadTask, NEW_SAMPLES_BUFFER_LEN, DAQmx_Val_WaitInfinitely, DAQmx_Val_GroupByChannel, 
                                      newSamplesList, EMG_CHANNELS_NUMBER * NEW_SAMPLES_BUFFER_LEN, &aquiredSamples, NULL );
  
  if( errorCode == 0 ) 
  {
    for( size_t channel = 0; channel < EMG_CHANNELS_NUMBER; channel++ )
    {
      float64* channelSamplesList = samplesList + channel * SAMPLES_BUFFER_LEN;
      
      float64* channelNewSamplesList = newSamplesList + channel * NEW_SAMPLES_BUFFER_LEN;
      float64* channelRectifiedSamplesList = rectifiedSamplesList + channel * FILTER_BUFFER_LEN;
      float64* channelFilteredSamplesList = filteredSamplesList + channel * FILTER_BUFFER_LEN;
      
      float64* channelProcessedSamplesList = processedSamplesList + channel * NEW_SAMPLES_BUFFER_LEN;

      for( int newSampleIndex = 0; newSampleIndex < NEW_SAMPLES_BUFFER_LEN; newSampleIndex++ )
        channelSamplesList[ ( newSamplesBufferStart + newSampleIndex ) % SAMPLES_BUFFER_LEN ] = channelNewSamplesList[ newSampleIndex ] / DATA_AQUISITION_SCALE_FACTOR;
      
      previousSamplesMean = 0.0;
      for( int sampleIndex = 0; sampleIndex < NEW_SAMPLES_BUFFER_LEN; sampleIndex++ )
      {
        for( int previousSampleIndex = oldSamplesBufferStart + sampleIndex; previousSampleIndex < newSamplesBufferStart + sampleIndex; previousSampleIndex++ )
          previousSamplesMean += channelSamplesList[ previousSampleIndex % SAMPLES_BUFFER_LEN ] / OLD_SAMPLES_BUFFER_LEN;
        
        channelRectifiedSamplesList[ FILTER_EXTRA_SAMPLES_NUMBER + sampleIndex ] = fabs( channelSamplesList[ ( newSamplesBufferStart + sampleIndex ) % SAMPLES_BUFFER_LEN ] - previousSamplesMean );
      }

      for( int sampleIndex = FILTER_EXTRA_SAMPLES_NUMBER; sampleIndex < FILTER_BUFFER_LEN; sampleIndex++ )
      {
        channelFilteredSamplesList[ sampleIndex ] = 0.0;
        for( int coeffIndex = 0; coeffIndex < FILTER_ORDER; coeffIndex++ )
        {
          channelFilteredSamplesList[ sampleIndex ] -= filter_A[ coeffIndex ] * channelFilteredSamplesList[ sampleIndex - coeffIndex ];
          channelFilteredSamplesList[ sampleIndex ] += filter_B[ coeffIndex ] * channelRectifiedSamplesList[ sampleIndex - coeffIndex ];
        }
        if( channelFilteredSamplesList[ sampleIndex ] < 0.0 ) channelFilteredSamplesList[ sampleIndex ] = 0.0;
      }

      for( int sampleIndex = 0; sampleIndex < FILTER_EXTRA_SAMPLES_NUMBER; sampleIndex++ )
      {
        channelFilteredSamplesList[ sampleIndex ] = channelFilteredSamplesList[ NEW_SAMPLES_BUFFER_LEN + sampleIndex ];
        channelRectifiedSamplesList[ sampleIndex ] = channelRectifiedSamplesList[ NEW_SAMPLES_BUFFER_LEN + sampleIndex ];
      }

      for( int sampleIndex = 0; sampleIndex < NEW_SAMPLES_BUFFER_LEN; sampleIndex++ )
        channelProcessedSamplesList[ sampleIndex ] = channelFilteredSamplesList[ FILTER_EXTRA_SAMPLES_NUMBER + sampleIndex ];
    }

    oldSamplesBufferStart += NEW_SAMPLES_BUFFER_LEN;
    newSamplesBufferStart += NEW_SAMPLES_BUFFER_LEN;

    return (double*) processedSamplesList;
  }
  else
  {
    static char errorMessage[ DEBUG_MESSAGE_LENGTH ];
    DAQmxGetErrorString( errorCode, errorMessage, DEBUG_MESSAGE_LENGTH );
    ERROR_EVENT( "error aquiring EMG data: %s", errorMessage );
  }
  
  return NULL;
  
}

static void* EMGProcessing_AsyncUpdate( void* referenceData )
{
  static float64 normalizedSamplesList[ EMG_CHANNELS_NUMBER * NEW_SAMPLES_BUFFER_LEN ];
  static float64 activationsList[ EMG_CHANNELS_NUMBER * NEW_SAMPLES_BUFFER_LEN ];
  
  while( isRunning )
  {
    float64* filteredSamplesList = EMGProcessing_GetFilteredSignal();
    
    if( filteredSamplesList != NULL )
    {
      switch( processingPhase )
      {
        case EMG_RELAXATION_PHASE:
        
          for( size_t channel = 0; channel < EMG_CHANNELS_NUMBER; channel++ )
          {
            float64* channelFilteredSamplesList = filteredSamplesList + channel * NEW_SAMPLES_BUFFER_LEN;
      
            for( size_t sampleIndex = 0; sampleIndex < NEW_SAMPLES_BUFFER_LEN; sampleIndex++ )
              emgReference.minList[ channel ] += channelFilteredSamplesList[ sampleIndex ];
          }
        
          ThreadLock_Aquire( phasePassCountLock );
          preparationPassCount++;
          ThreadLock_Release( phasePassCountLock );
          
          break;
        
        case EMG_CONTRACTION_PHASE:
        
          for( size_t channel = 0; channel < EMG_CHANNELS_NUMBER; channel++ )
          {
            float64* channelFilteredSamplesList = filteredSamplesList + channel * NEW_SAMPLES_BUFFER_LEN;
      
            for( size_t sampleIndex = 0; sampleIndex < NEW_SAMPLES_BUFFER_LEN; sampleIndex++ )
              emgReference.maxList[ channel ] += channelFilteredSamplesList[ sampleIndex ];
          }
        
          ThreadLock_Aquire( phasePassCountLock );
          preparationPassCount++;
          ThreadLock_Release( phasePassCountLock );
          
          break;
        
        case EMG_WORK_PHASE:
        
          for( size_t channel = 0; channel < EMG_CHANNELS_NUMBER; channel++ )
          {
            float64* channelFilteredSamplesList = filteredSamplesList + channel * NEW_SAMPLES_BUFFER_LEN;
            float64* channelNormalizedSamplesList = normalizedSamplesList + channel * NEW_SAMPLES_BUFFER_LEN;
            float64* channelActivationsList = activationsList + channel * NEW_SAMPLES_BUFFER_LEN;
      
            for( size_t sampleIndex = 0; sampleIndex < NEW_SAMPLES_BUFFER_LEN; sampleIndex++ )
            {
              if( emgReference.maxList[ channel ] > 0.0 )
                channelNormalizedSamplesList[ sampleIndex ] = ( channelFilteredSamplesList[ sampleIndex ] - emgReference.minList[ channel ] ) 
                                                              / ( emgReference.maxList[ channel ] - emgReference.minList[ channel ] );
              else
                channelNormalizedSamplesList[ sampleIndex ] = 0.0;
          
              if( channelNormalizedSamplesList[ sampleIndex ] > 1.0 ) channelNormalizedSamplesList[ sampleIndex ] = 1.0;
              else if( channelNormalizedSamplesList[ sampleIndex ] < 0.0 ) channelNormalizedSamplesList[ sampleIndex ] = 0.0;
            
              channelActivationsList[ sampleIndex ] = ( exp( -2 * channelNormalizedSamplesList[ sampleIndex ] ) - 1 ) / ( exp( -2 ) - 1 );
              
              //if( channel == 0 && sampleIndex == 0 && channelNormalizedSamplesList[ sampleIndex ] > 0.0 ) 
              //  DEBUG_PRINT( "emg norm: %g - activ: %g", channelNormalizedSamplesList[ sampleIndex ], channelActivationsList[ sampleIndex ] );
              
              averageActivationsList[ channel ] += channelActivationsList[ sampleIndex ];
            }
          }
          
          //DEBUG_PRINT( "emg: (%g - %g)/(%g - %g) = %g", filteredSamplesList[ 0 ], emgReference.minList[ 0 ], 
          //                                              emgReference.maxList[ 0 ], emgReference.minList[ 0 ], normalizedSamplesList[ 0 ] );
        
          ThreadLock_Aquire( phasePassCountLock );
          workingPassCount++;
          ThreadLock_Release( phasePassCountLock );
          
          break;
      }
    }
    
  }
  
  Thread_Exit( 0 );
  return NULL;
}

void EMGProcessing_ChangePhase( int newProcessingPhase )
{
  if( newProcessingPhase < 0 || newProcessingPhase >= EMG_PROCESSING_PHASES_NUMBER ) return;
  
  ThreadLock_Aquire( phasePassCountLock );
  
  if( preparationPassCount > 0 )
  {
    switch( processingPhase )
    {
      case EMG_RELAXATION_PHASE:
      
        for( size_t channel = 0; channel < EMG_CHANNELS_NUMBER; channel++ )
          emgReference.minList[ channel ] /= ( NEW_SAMPLES_BUFFER_LEN * preparationPassCount );

        DEBUG_PRINT( "new min value: %g (%lu passes)", emgReference.minList[ 0 ], preparationPassCount );
      
        break;

      case EMG_CONTRACTION_PHASE:
      
        for( size_t channel = 0; channel < EMG_CHANNELS_NUMBER; channel++ )
          emgReference.maxList[ channel ] /= ( NEW_SAMPLES_BUFFER_LEN * preparationPassCount );

        DEBUG_PRINT( "new max value: %g (%lu passes)", emgReference.maxList[ 0 ], preparationPassCount );
      
        break;
    }
  }
  
  preparationPassCount = 0;
  
  switch( newProcessingPhase )
    {
      case EMG_RELAXATION_PHASE:
      
        for( size_t channel = 0; channel < EMG_CHANNELS_NUMBER; channel++ )
          emgReference.minList[ channel ] = 0.0;
      
        break;

      case EMG_CONTRACTION_PHASE:
      
        for( size_t channel = 0; channel < EMG_CHANNELS_NUMBER; channel++ )
          emgReference.maxList[ channel ] = 0.0;
      
        break;
        
      case EMG_WORK_PHASE:
        
        for( size_t channel = 0; channel < EMG_CHANNELS_NUMBER; channel++ )
          averageActivationsList[ channel ] = 0.0;
        
        break;
    }
  
  processingPhase = newProcessingPhase;
  
  ThreadLock_Release( phasePassCountLock );
}

extern inline float64* EMGProcessing_GetActivations()
{
  ThreadLock_Aquire( phasePassCountLock );
  
  if( workingPassCount > 0 )
  {
    for( size_t channel = 0; channel < EMG_CHANNELS_NUMBER; channel++ )
      averageActivationsList[ channel ] /= ( NEW_SAMPLES_BUFFER_LEN * workingPassCount );
  }
  
  workingPassCount = 0;
  
  ThreadLock_Release( phasePassCountLock );
  
  return (float64*) averageActivationsList;
}

double* EMGProcessing_GetMuscleMeasures( int muscleID, double jointAngle )
{
  static double muscleMeasuresList[ MUSCLE_MEASURES_NUMBER ];
  
  if( 
  
  muscleMeasuresList[ MUSCLE_ACTIVE_FORCE ] = 0.0;
  for( size_t coeffIndex = 0; coeffIndex < COEFFS_NUMBER; coeffIndex++ )
    muscleMeasuresList[ MUSCLE_ACTIVE_FORCE ] += 
  
  //Rectus Femoris 
  forceActiveRF = 0.0;
  for (int i=0; i < COEFFS_NUMBER; i++)
    forceActiveRF += coeffs_forceActiveRF[i] * pow( jointAngle, i );
  
  forcePassiveRF = 0.0;
  for (int i=0; i < COEFFS_NUMBER; i++)
    forcePassiveRF += coeffs_forcePassiveRF[i] * pow( jointAngle, i );
  
  momentArmRF = 0.0;
  for (int i=0; i < COEFFS_NUMBER; i++)
    momentArmRF += coeffs_momentArmRF[i] * pow( jointAngle, i );
  
  normLengthRF = 0.0;
  for (int i=0; i < COEFFS_NUMBER; i++)
    normLengthRF += coeffs_normLengthRF[i] * pow( jointAngle, i );
  
  phiRF = asin(sin(phiZeroRF)/normLengthRF);
  
  forceRF = alphaRF * ( forceActiveRF * activationMeasure_1 + forcePassiveRF ) * cos( phiRF );
  torqueRF = forceRF*momentArmRF;
  
  
  //Biceps Femoris
  forceActiveBF = 0.0;
  for (int i=0; i < COEFFS_NUMBER; i++)
    forceActiveBF += coeffs_forceActiveBF[i] * pow( jointAngle, i );
  
  forcePassiveBF = 0.0;
  for (int i=0; i < COEFFS_NUMBER; i++)
    forcePassiveBF += coeffs_forcePassiveBF[i] * pow( jointAngle, i );
  
  momentArmBF = 0.0;
  for (int i=0; i < COEFFS_NUMBER; i++)
    momentArmBF += coeffs_momentArmBF[i] * pow( jointAngle, i );
  
  normLengthBF = 0.0;
  for (int i=0; i < COEFFS_NUMBER; i++)
    normLengthBF += coeffs_normLengthBF[i] * pow( jointAngle, i );
  
  phiBF = asin(sin(phiZeroBF)/normLengthBF); 
  
  forceBF = alphaBF * ( forceActiveBF * activationMeasure_2 + forcePassiveBF ) * cos( phiBF );
  torqueBF = forceBF * momentArmBF;
  
  
  //Torque Total
  torqueMuscle = torqueRF - torqueBF;
  
  return torqueMuscle;
}


#endif /* EMG_PROCESS_H */
