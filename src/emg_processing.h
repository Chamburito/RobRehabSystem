#ifndef EMG_PROCESS
#define EMG_PROCESS

#include <NIDAQmx.h>

#include <math.h>
#include <stdbool.h>

#include "async_debug.h"

TaskHandle emgReadTask;

Thread_Handle emgThreadID;

const int EMG_NEW_SAMPLES_BUFFER_LEN = 10;
const int EMG_CHANNELS_NUMBER = 2;

static float64* emgValuesList = NULL;

typedef struct _EMGReference
{
  float64 zerosList[ EMG_CHANNELS_NUMBER ];
  float64 maxList[ EMG_CHANNELS_NUMBER ];
}
EMGReference;

static bool isRunning;

enum EMGProcessingPhase { EMG_RELAXATION_PHASE, EMG_CONTRACTION_PHASE, EMG_WORKING_PHASE };
static uint8_t processingPhase = EMG_WORKING_PHASE;
static unsigned int preparationPassCount = 0;

static double* EMGProcessing_GetFilteredSignal();
static double* EMGProcessing_GetNormalizedSignal( EMGReference* );
static void* EMGProcessing_AsyncUpdate( void* );

void EMGProcessing_Init()
{
  static EMGReference reference;
  
  DAQmxLoadTask( "EMGReadTask", &emgReadTask );
  DAQmxStartTask( emgReadTask );
  
  isRunning = true;
  emgThreadID = Thread_Start( EMGProcessing_AsyncUpdate, NULL, JOINABLE );
}

const int EMG_FILTER_ORDER = 3;
const float64 filter_A[ EMG_FILTER_ORDER ] = { 1.0, -1.9964, 0.9965 };
const float64 filter_B[ EMG_FILTER_ORDER ] = { 1.5763e-6, 3.1527e-6, 1.5763e-6 };
static double* EMGProcessing_Update( EMGReference* reference )
{
  const int EMG_FILTER_EXTRA_SAMPLES_NUMBER = EMG_FILTER_ORDER - 1;
  const int EMG_FILTER_BUFFER_LEN = EMG_FILTER_EXTRA_SAMPLES_NUMBER + EMG_NEW_SAMPLES_BUFFER_LEN;
  
  const int EMG_OLD_SAMPLES_BUFFER_LEN = 100;
  const int EMG_SAMPLES_BUFFER_LEN = EMG_OLD_SAMPLES_BUFFER_LEN + EMG_NEW_SAMPLES_BUFFER_LEN;
  
  static float64 emgSamplesList[ EMG_CHANNELS_NUMBER * EMG_SAMPLES_BUFFER_LEN ];
  static int oldSamplesBufferStart = 0, newSamplesBufferStart = EMG_OLD_SAMPLES_BUFFER_LEN;
  
  static float64 emgNewSamplesList[ EMG_CHANNELS_NUMBER * EMG_NEW_SAMPLES_BUFFER_LEN ];
  static float64 emgRectifiedSamplesList[ EMG_CHANNELS_NUMBER * EMG_FILTER_BUFFER_LEN ];
  static float64 emgFilteredSamplesList[ EMG_CHANNELS_NUMBER * EMG_FILTER_BUFFER_LEN ];
  static float64 emgProcessedSamplesList[ EMG_CHANNELS_NUMBER * EMG_NEW_SAMPLES_BUFFER_LEN ];
  static float64 emgPreviousSamplesMean;
  static int32 aquiredSamples;
  
  int errorCode = DAQmxReadAnalogF64( emgReadTask, EMG_NEW_SAMPLES_BUFFER_LEN, DAQmx_Val_WaitInfinitely, DAQmx_Val_GroupByChannel, 
                                      emgSamplesList, EMG_CHANNELS_NUMBER * EMG_NEW_SAMPLES_BUFFER_LEN, &aquiredSamples, NULL );
  
  if( errorCode == 0 ) 
  {
    for( size_t channel = 0; channel < EMG_CHANNELS_NUMBER; channel++ )
    {
      float64* emgChannelSamplesList = emgSamplesList + channel * EMG_SAMPLES_BUFFER_LEN;
      
      float64* emgChannelNewSamplesList = emgNewSamplesList + channel * EMG_NEW_SAMPLES_BUFFER_LEN;
      float64* emgChannelRectifiedSamplesList = emgRectifiedSamplesList + channel * EMG_FILTER_BUFFER_LEN;
      float64* emgChannelFilteredSamplesList = emgFilteredSamplesList + channel * EMG_FILTER_BUFFER_LEN;
      
      float64* emgChannelProcessedSamplesList = emgProcessedSamplesList + channel * EMG_NEW_SAMPLES_BUFFER_LEN;

      for( int newSampleIndex = 0; newSampleIndex < EMG_NEW_SAMPLES_BUFFER_LEN; newSampleIndex++ )
        emgChannelSamplesList[ newSamplesBufferStart + newSampleIndex ] = emgChannelNewSamplesList[ newSampleIndex ];
      
      emgPreviousSamplesMean = 0.0;
      for( int sampleIndex = 0; sampleIndex < EMG_NEW_SAMPLES_BUFFER_LEN; sampleIndex++ )
      {
        for( int previousSampleIndex = oldSamplesBufferStart + sampleIndex; previousSampleIndex < newSamplesBufferStart + sampleIndex; previousSampleIndex++ )
          emgPreviousSamplesMean += emgChannelSamplesList[ previousSampleIndex ] / EMG_OLD_SAMPLES_BUFFER_LEN;
        
        emgChannelRectifiedSamplesList[ sampleIndex ] = fabs( emgChannelSamplesList[ sampleIndex ] - emgPreviousSamplesMean );
      }

      for( int sampleIndex = EMG_FILTER_EXTRA_SAMPLES_NUMBER; sampleIndex < EMG_FILTER_BUFFER_LEN; sampleIndex++ )
      {
        emgChannelFilteredSamplesList[ sampleIndex ] = 0.0;
        for( int coeffIndex = 0; coeffIndex < EMG_FILTER_ORDER; coeffIndex++ )
        {
          emgChannelFilteredSamplesList[ sampleIndex ] -= filter_A[ coeffIndex ] * emgChannelFilteredSamplesList[ sampleIndex - coeffIndex ];
          emgChannelFilteredSamplesList[ sampleIndex ] += filter_B[ coeffIndex ] * emgChannelRectifiedSamplesList[ sampleIndex - coeffIndex ];
        }
      }

      for( int sampleIndex = 0; sampleIndex < EMG_FILTER_EXTRA_SAMPLES_NUMBER; sampleIndex++ )
      {
        emgChannelFilteredSamplesList[ sampleIndex ] = emgChannelFilteredSamplesList[ EMG_NEW_SAMPLES_BUFFER_LEN + sampleIndex ];
        emgChannelRectifiedSamplesList[ sampleIndex ] = emgChannelRectifiedSamplesList[ EMG_NEW_SAMPLES_BUFFER_LEN + sampleIndex ];
      }

      for( int sampleIndex = 0; sampleIndex < EMG_NEW_SAMPLES_BUFFER_LEN; sampleIndex++ )
        emgChannelProcessedSamplesList[ sampleIndex ] = emgChannelFilteredSamplesList[ EMG_FILTER_EXTRA_SAMPLES_NUMBER + sampleIndex ];
    }
    
    oldSamplesBufferStart = ( oldSamplesBufferStart + EMG_NEW_SAMPLES_BUFFER_LEN ) % EMG_SAMPLES_BUFFER_LEN;
    newSamplesBufferStart = ( newSamplesBufferStart + EMG_NEW_SAMPLES_BUFFER_LEN ) % EMG_SAMPLES_BUFFER_LEN;
    
    if( reference != NULL ) 
      

    return (double*) emgProcessedSamplesList;
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
  static float64 emgNormalizedSamplesList[ EMG_CHANNELS_NUMBER * EMG_NEW_SAMPLES_BUFFER_LEN ];
  static float64 emgActivationsList[ EMG_CHANNELS_NUMBER * EMG_NEW_SAMPLES_BUFFER_LEN ];
  
  static float64 emgChannelsZerosList[ EMG_CHANNELS_NUMBER ];
  static float64 emgChannelsMaxList[ EMG_CHANNELS_NUMBER ];
  
  while( isRunning )
  {
    float64* emgFilteredSamplesList = EMGProcessing_GetFilteredSignal();
    
    if( emgFilteredSamplesList != NULL )
    {
      switch( processingPhase )
      {
        case EMG_RELAXATION_PHASE:
        
          for( size_t channel = 0; channel < EMG_CHANNELS_NUMBER; channel++ )
          {
            float64* emgChannelFilteredSamplesList = emgFilteredSamplesList + channel * EMG_NEW_SAMPLES_BUFFER_LEN;
      
            for( size_t sampleIndex = 0; sampleIndex < EMG_NEW_SAMPLES_BUFFER_LEN; sampleIndex++ )
              emgChannelsZerosList[ channel ] += emgChannelFilteredSamplesList[ sampleIndex ];
          }
          
          preparationPassCount++;
        
          break;
        
        case EMG_CONTRACTION_PHASE:
        
          for( size_t channel = 0; channel < EMG_CHANNELS_NUMBER; channel++ )
          {
            float64* emgChannelFilteredSamplesList = emgFilteredSamplesList + channel * EMG_NEW_SAMPLES_BUFFER_LEN;
      
            for( size_t sampleIndex = 0; sampleIndex < EMG_NEW_SAMPLES_BUFFER_LEN; sampleIndex++ )
              emgChannelsMaxList[ channel ] += emgChannelFilteredSamplesList[ sampleIndex ] / ( EMG_NEW_SAMPLES_BUFFER_LEN * PREPARATION_PASSES_NUMBER );
          }
          
          preparationPassCount++;
        
          break;
        
        case EMG_WORKING_PHASE:
        
          for( size_t channel = 0; channel < EMG_CHANNELS_NUMBER; channel++ )
          {
            float64* emgChannelFilteredSamplesList = emgFilteredSamplesList + channel * EMG_NEW_SAMPLES_BUFFER_LEN;
            float64* emgChannelNormalizedSamplesList = emgNormalizedSamplesList + channel * EMG_NEW_SAMPLES_BUFFER_LEN;
            float64* emgChannelActivationsList = emgActivationsList + channel * EMG_NEW_SAMPLES_BUFFER_LEN;
      
            for( size_t sampleIndex = 0; sampleIndex < EMG_NEW_SAMPLES_BUFFER_LEN; sampleIndex++ )
            {
              if( reference->maxList[ channel ] > 0.0 )
                emgChannelNormalizedSamplesList[ sampleIndex ] = ( emgChannelFilteredSamplesList[ sampleIndex ] - emgChannelsZerosList[ channel ] ) / emgChannelsMaxList[ channel ];
              else
                emgChannelNormalizedSamplesList[ sampleIndex ] = 1.0;
          
              float64 emgNormalizedSample = emgChannelNormalizedSamplesList[ sampleIndex ];
            
              emgChannelActivationsList[ sampleIndex ] = ( exp( -2 * emgNormalizedSample ) - 1 ) / ( exp( -2 ) - 1 );
            }
          }
          
          emgValuesList = emgNormalizedSamplesList;
          DEBUG_PRINT( "emg: ( %.6f - %.6f ) / %.6f = %.3f", emgFilteredSamplesList[ 0 ], emgChannelsZerosList[ 0 ], emgChannelsMaxList[ 0 ], emgNormalizedSamplesList[ 0 ] );
        
          break;
      }
    }
    
    // Atualiza o vetor a ser retornado por "EMGProcessing_GetValues()" (ativações dos músculos) em "robrehab_network.h"
    
    //emgValuesList = emgActivationsList;
  }
  
  Thread_Exit( 0 );
  return NULL;
}

void EMGProcessing_ChangePhase( enum EMGProcessingPhase )
{
  switch( processingPhase )
  {
    case EMG_RELAXATION_PHASE:
      
      for( size_t channel = 0; channel < EMG_CHANNELS_NUMBER; channel++ )
        emgChannelsZerosList[ channel ] /= ( EMG_NEW_SAMPLES_BUFFER_LEN * preparationPassCount );

      break;

    case EMG_CONTRACTION_PHASE:
      
      for( size_t channel = 0; channel < EMG_CHANNELS_NUMBER; channel++ )
        emgChannelsMaxList[ channel ] /= ( EMG_NEW_SAMPLES_BUFFER_LEN * preparationPassCount );

      break;
  }
  
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
