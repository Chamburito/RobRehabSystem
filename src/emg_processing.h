#ifndef EMG_PROCESS
#define EMG_PROCESS

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

static float64* emgValuesList = NULL;

static struct
{
  float64 zerosList[ EMG_CHANNELS_NUMBER ];
  float64 maxList[ EMG_CHANNELS_NUMBER ];
}
emgReference;

static bool isRunning;

const struct { int RELAXATION = 0, CONTRACTION = 1, WORKING = 2; } EMGProcessingPhase;
const size_t EMG_PROCESSING_PHASES_NUMBER = sizeof(EMGProcessingPhase) / sizeof(int);

static uint8_t processingPhase = EMGProcessingPhase.RELAXATION;
static unsigned int preparationPassCount = 0;

static float64* EMGProcessing_GetFilteredSignal();
static double* EMGProcessing_GetNormalizedSignal( EMGReference* );
static void* EMGProcessing_AsyncUpdate( void* );

void EMGProcessing_Init()
{
  DAQmxLoadTask( "EMGReadTask", &emgReadTask );
  DAQmxStartTask( emgReadTask );
  
  isRunning = true;
  emgThreadID = Thread_Start( EMGProcessing_AsyncUpdate, NULL, JOINABLE );
}

const int NEW_SAMPLES_BUFFER_LEN = 10;
const size_t FILTER_ORDER = 3;
const float64 filter_A[ FILTER_ORDER ] = { 1.0, -1.9964, 0.9965 };
const float64 filter_B[ FILTER_ORDER ] = { 1.5763e-6, 3.1527e-6, 1.5763e-6 };
static double* EMGProcessing_Update( EMGReference* reference )
{
  const int FILTER_EXTRA_SAMPLES_NUMBER = FILTER_ORDER - 1;
  const int FILTER_BUFFER_LEN = FILTER_EXTRA_SAMPLES_NUMBER + NEW_SAMPLES_BUFFER_LEN;
  
  const int OLD_SAMPLES_BUFFER_LEN = 100;
  const int SAMPLES_BUFFER_LEN = OLD_SAMPLES_BUFFER_LEN + NEW_SAMPLES_BUFFER_LEN;
  
  static float64 samplesList[ EMG_CHANNELS_NUMBER * SAMPLES_BUFFER_LEN ];
  static int oldSamplesBufferStart = 0, newSamplesBufferStart = OLD_SAMPLES_BUFFER_LEN;
  
  static float64 newSamplesList[ EMG_CHANNELS_NUMBER * NEW_SAMPLES_BUFFER_LEN ];
  static float64 rectifiedSamplesList[ EMG_CHANNELS_NUMBER * FILTER_BUFFER_LEN ];
  static float64 filteredSamplesList[ EMG_CHANNELS_NUMBER * FILTER_BUFFER_LEN ];
  static float64 processedSamplesList[ EMG_CHANNELS_NUMBER * NEW_SAMPLES_BUFFER_LEN ];
  static float64 previousSamplesMean;
  static int32 aquiredSamples;
  
  int errorCode = DAQmxReadAnalogF64( emgReadTask, NEW_SAMPLES_BUFFER_LEN, DAQmx_Val_WaitInfinitely, DAQmx_Val_GroupByChannel, 
                                        samplesList, EMG_CHANNELS_NUMBER * NEW_SAMPLES_BUFFER_LEN, &aquiredSamples, NULL );
  
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
                channelSamplesList[ newSamplesBufferStart + newSampleIndex ] = channelNewSamplesList[ newSampleIndex ];
      
      previousSamplesMean = 0.0;
      for( int sampleIndex = 0; sampleIndex < NEW_SAMPLES_BUFFER_LEN; sampleIndex++ )
      {
        for( int previousSampleIndex = oldSamplesBufferStart + sampleIndex; previousSampleIndex < newSamplesBufferStart + sampleIndex; previousSampleIndex++ )
                    previousSamplesMean += channelSamplesList[ previousSampleIndex ] / OLD_SAMPLES_BUFFER_LEN;
        
                channelRectifiedSamplesList[ sampleIndex ] = fabs( channelSamplesList[ sampleIndex ] - previousSamplesMean );
      }

      for( int sampleIndex = FILTER_EXTRA_SAMPLES_NUMBER; sampleIndex < FILTER_BUFFER_LEN; sampleIndex++ )
      {
                channelFilteredSamplesList[ sampleIndex ] = 0.0;
        for( int coeffIndex = 0; coeffIndex < FILTER_ORDER; coeffIndex++ )
        {
                    channelFilteredSamplesList[ sampleIndex ] -= filter_A[ coeffIndex ] * channelFilteredSamplesList[ sampleIndex - coeffIndex ];
                    channelFilteredSamplesList[ sampleIndex ] += filter_B[ coeffIndex ] * channelRectifiedSamplesList[ sampleIndex - coeffIndex ];
        }
      }

      for( int sampleIndex = 0; sampleIndex < FILTER_EXTRA_SAMPLES_NUMBER; sampleIndex++ )
      {
                channelFilteredSamplesList[ sampleIndex ] = channelFilteredSamplesList[ NEW_SAMPLES_BUFFER_LEN + sampleIndex ];
                channelRectifiedSamplesList[ sampleIndex ] = channelRectifiedSamplesList[ NEW_SAMPLES_BUFFER_LEN + sampleIndex ];
      }

      for( int sampleIndex = 0; sampleIndex < NEW_SAMPLES_BUFFER_LEN; sampleIndex++ )
                channelProcessedSamplesList[ sampleIndex ] = channelFilteredSamplesList[ FILTER_EXTRA_SAMPLES_NUMBER + sampleIndex ];
    }
    
    oldSamplesBufferStart = ( oldSamplesBufferStart + NEW_SAMPLES_BUFFER_LEN ) % SAMPLES_BUFFER_LEN;
    newSamplesBufferStart = ( newSamplesBufferStart + NEW_SAMPLES_BUFFER_LEN ) % SAMPLES_BUFFER_LEN;      

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
        case EMGProcessingPhase.RELAXATION:
        
          for( size_t channel = 0; channel < EMG_CHANNELS_NUMBER; channel++ )
          {
            float64* channelFilteredSamplesList = emgFilteredSamplesList + channel * NEW_SAMPLES_BUFFER_LEN;
      
            for( size_t sampleIndex = 0; sampleIndex < NEW_SAMPLES_BUFFER_LEN; sampleIndex++ )
              emgReference.zerosList[ channel ] += channelFilteredSamplesList[ sampleIndex ];
          }
          
          preparationPassCount++;
        
          break;
        
        case EMGProcessingPhase.CONTRACTION:
        
          for( size_t channel = 0; channel < EMG_CHANNELS_NUMBER; channel++ )
          {
            float64* channelFilteredSamplesList = emgFilteredSamplesList + channel * NEW_SAMPLES_BUFFER_LEN;
      
            for( size_t sampleIndex = 0; sampleIndex < NEW_SAMPLES_BUFFER_LEN; sampleIndex++ )
              emgReference.maxList[ channel ] += channelFilteredSamplesList[ sampleIndex ] / ( NEW_SAMPLES_BUFFER_LEN * PREPARATION_PASSES_NUMBER );
          }
          
          preparationPassCount++;
        
          break;
        
        case EMGProcessingPhase.WORKING:
        
          for( size_t channel = 0; channel < EMG_CHANNELS_NUMBER; channel++ )
          {
            float64* channelFilteredSamplesList = emgFilteredSamplesList + channel * NEW_SAMPLES_BUFFER_LEN;
            float64* channelNormalizedSamplesList = normalizedSamplesList + channel * NEW_SAMPLES_BUFFER_LEN;
            float64* channelActivationsList = activationsList + channel * NEW_SAMPLES_BUFFER_LEN;
      
            for( size_t sampleIndex = 0; sampleIndex < NEW_SAMPLES_BUFFER_LEN; sampleIndex++ )
            {
              if( emgReference.maxList[ channel ] > 0.0 )
                            channelNormalizedSamplesList[ sampleIndex ] = ( channelFilteredSamplesList[ sampleIndex ] - emgReference.zerosList[ channel ] ) / emgReference.maxList[ channel ];
              else
                            channelNormalizedSamplesList[ sampleIndex ] = 1.0;
          
              float64 normalizedSample = channelNormalizedSamplesList[ sampleIndex ];
            
                        channelActivationsList[ sampleIndex ] = ( exp( -2 * normalizedSample ) - 1 ) / ( exp( -2 ) - 1 );
            }
          }
          
          emgValuesList = normalizedSamplesList;
          DEBUG_PRINT( "emg: ( %.6f - %.6f ) / %.6f = %.3f", emgFilteredSamplesList[ 0 ], emgReference.zerosList[ 0 ], emgReference.maxList[ 0 ], normalizedSamplesList[ 0 ] );
        
          break;
      }
    }
    
    // Atualiza o vetor a ser retornado por "EMGProcessing_GetValues()" (ativações dos músculos) em "robrehab_network.h"
    
    //emgValuesList = emgActivationsList;
  }
  
  Thread_Exit( 0 );
  return NULL;
}

void EMGProcessing_ChangePhase( uint8_t newProcessingPhase )
{
  switch( processingPhase )
  {
    case EMGProcessingPhase.RELAXATION:
      
      for( size_t channel = 0; channel < EMG_CHANNELS_NUMBER; channel++ )
      {
        if( preparationPassCount > 0 )
          emgReference.zerosList[ channel ] /= ( NEW_SAMPLES_BUFFER_LEN * preparationPassCount );
        else
          emgReference.zerosList[ channel ] = 0;
      }

      break;

    case EMGProcessingPhase.CONTRACTION:
      
      for( size_t channel = 0; channel < EMG_CHANNELS_NUMBER; channel++ )
      {
        if( preparationPassCount > 0 )
          emgReference.maxList[ channel ] /= ( NEW_SAMPLES_BUFFER_LEN * preparationPassCount );
        else
          emgReference.maxList[ channel ] = 0;
      }

      break;
  }
  
  processingPhase = newProcessingPhase;
  
  preparationPassCount = 0;
}

extern inline float64* EMGProcessing_GetValues()
{
  float64* emgReturnValuesList = emgValuesList;
  
  //emgValuesList = NULL;
  
  return emgReturnValuesList;
}

void EMGProcessing_End()
{
  isRunning = false;
  
  (void) Thread_WaitExit( emgThreadID, 5000 );
  emgValuesList = NULL;
  
  DAQmxStopTask( emgReadTask );
  DAQmxClearTask( emgReadTask );
}


#endif /* EMG_PROCESS */
