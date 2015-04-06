#ifndef EMG_PROCESS
#define EMG_PROCESS

#include <NIDAQmx.h>

#include <math.h>
#include <stdbool.h>

#include "async_debug.h"

TaskHandle emgReadTask;

Thread_Handle emgThreadID;

const int EMG_SAMPLES_NUMBER = 10;
const int EMG_CHANNELS_NUMBER = 2;

static float64* emgValuesList = NULL;

typedef struct _EMGReference
{
  float64 zerosList[ EMG_CHANNELS_NUMBER * EMG_SAMPLES_NUMBER ];
  float64 maxList[ EMG_CHANNELS_NUMBER ];
}
EMGReference;

static bool isRunning;

static double* EMGProcessing_Update( float64*, float64* );
static void* EMGProcessing_AsyncUpdate( void* );

void EMGProcessing_Init()
{
  static EMGReference reference;
  
  DAQmxLoadTask( "EMGReadTask", &emgReadTask );
  DAQmxStartTask( emgReadTask );
  
  const unsigned int PREPARATION_PASSES_NUMBER = 1000;
  unsigned int execTime = Timing_GetExecTimeSeconds(); 
  
  DEBUG_PRINT( "Relaxation time begin (%u passes)", PREPARATION_PASSES_NUMBER );
  
  for( unsigned int passNumber = 0; passNumber < PREPARATION_PASSES_NUMBER; passNumber++ )
  {
    float64* emgFilteredSamplesList = EMGProcessing_Update( NULL, NULL );
    
    for( size_t channel = 0; channel < EMG_CHANNELS_NUMBER; channel++ )
    {
      float64* emgChannelZerosList = reference.zerosList + channel * EMG_SAMPLES_NUMBER;
      float64* emgChannelFilteredSamplesList = emgFilteredSamplesList + channel * EMG_SAMPLES_NUMBER;
      
      for( size_t sampleIndex = 0; sampleIndex < EMG_SAMPLES_NUMBER; sampleIndex++ )
        emgChannelZerosList[ sampleIndex ] += emgChannelFilteredSamplesList[ sampleIndex ] / PREPARATION_PASSES_NUMBER;
    }
  }
  
  DEBUG_PRINT( "Relaxation time end (%u seconds)", Timing_GetExecTimeSeconds() - execTime );
  
  execTime = Timing_GetExecTimeSeconds();
  
  DEBUG_PRINT( "Contraction time begin (%u passes)", PREPARATION_PASSES_NUMBER );
  
  for( unsigned int passNumber = 0; passNumber < PREPARATION_PASSES_NUMBER; passNumber++ )
    EMGProcessing_Update( NULL, (float64*) reference.maxList );
  
  DEBUG_PRINT( "Contraction time end (%u seconds)", Timing_GetExecTimeSeconds() - execTime );
  
  isRunning = true;
  emgThreadID = Thread_Start( EMGProcessing_AsyncUpdate, (void*) &reference, JOINABLE );
}

const int FILTER_COEFF_NUMBER = 3;
const float64 filter_A[ FILTER_COEFF_NUMBER ] = { 1.0, -1.9964, 0.9965 };
const float64 filter_B[ FILTER_COEFF_NUMBER ] = { 1.5763e-6, 3.1527e-6, 1.5763e-6 };
static double* EMGProcessing_Update( float64* emgSamplesZerosList, float64* emgSamplesMaxList )
{
  static float64 emgSamplesList[ EMG_CHANNELS_NUMBER * EMG_SAMPLES_NUMBER ];
  static float64 emgFilteredSamplesList[ EMG_CHANNELS_NUMBER * EMG_SAMPLES_NUMBER ];
  static float64 emgSamplesMeansList[ EMG_CHANNELS_NUMBER ];
  static int32 aquiredSamples;
  
  int errorCode = DAQmxReadAnalogF64( emgReadTask, EMG_SAMPLES_NUMBER, DAQmx_Val_WaitInfinitely, DAQmx_Val_GroupByChannel, 
                                      emgSamplesList, EMG_CHANNELS_NUMBER * EMG_SAMPLES_NUMBER, &aquiredSamples, NULL );
  
  if( errorCode == 0 ) 
  {
    for( size_t channel = 0; channel < EMG_CHANNELS_NUMBER; channel++ )
    {
      float64* emgChannelSamplesList = emgSamplesList + channel * EMG_SAMPLES_NUMBER;
      float64* emgChannelFilteredSamplesList = emgFilteredSamplesList + channel * EMG_SAMPLES_NUMBER;
      
      float64* emgChannelZerosList = ( emgSamplesZerosList != NULL ) ? emgSamplesZerosList + channel * EMG_SAMPLES_NUMBER : NULL;

      emgSamplesMeansList[ channel ] = 0.0;
      for( int sampleIndex = 0; sampleIndex < EMG_SAMPLES_NUMBER; sampleIndex++ )
      {
        emgSamplesMeansList[ channel ] += emgChannelSamplesList[ sampleIndex ] / EMG_SAMPLES_NUMBER;
      
        if( emgSamplesMaxList != NULL )
        {
          if( emgChannelSamplesList[ sampleIndex ] > emgSamplesMaxList[ channel ] ) 
            emgSamplesMaxList[ channel ] = emgChannelSamplesList[ sampleIndex ];
        }
      }

      for( int sampleIndex = 0; sampleIndex < EMG_SAMPLES_NUMBER; sampleIndex++ )
        emgChannelSamplesList[ sampleIndex ] = fabs( emgChannelSamplesList[ sampleIndex ] - emgSamplesMeansList[ channel ] );

      for( int sampleIndex = EMG_SAMPLES_NUMBER - FILTER_COEFF_NUMBER; sampleIndex >= 0; sampleIndex-- )
      {
        emgChannelFilteredSamplesList[ sampleIndex ] = 0.0;
        for( int coeffIndex = 0; coeffIndex < FILTER_COEFF_NUMBER; coeffIndex++ )
        {
          emgChannelFilteredSamplesList[ sampleIndex ] -= filter_A[ coeffIndex ] * emgChannelFilteredSamplesList[ sampleIndex + coeffIndex ];
          emgChannelFilteredSamplesList[ sampleIndex ] += filter_B[ coeffIndex ] * emgChannelSamplesList[ sampleIndex + coeffIndex ];
        }
        if( emgChannelZerosList != NULL ) emgChannelFilteredSamplesList[ sampleIndex ] -= emgChannelZerosList[ sampleIndex ];
      }

      for( int sampleIndex = 0; sampleIndex < FILTER_COEFF_NUMBER - 1; sampleIndex++ )
      {
        int oldSampleIndex = EMG_SAMPLES_NUMBER - FILTER_COEFF_NUMBER + 1 + sampleIndex;
        emgChannelFilteredSamplesList[ oldSampleIndex ] = emgChannelFilteredSamplesList[ sampleIndex ];
      }
    }
      
    return (double*) emgFilteredSamplesList;
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
  static float64 emgActivationsList[ EMG_CHANNELS_NUMBER * EMG_SAMPLES_NUMBER ];
  
  EMGReference* reference = (EMGReference*) referenceData;
  
  while( isRunning )
  {
    float64* emgFilteredSamplesList = EMGProcessing_Update( reference->zerosList, reference->maxList );
    
    if( emgFilteredSamplesList != NULL )
    {
      for( size_t channel = 0; channel < EMG_CHANNELS_NUMBER; channel++ )
      {
        float64* emgChannelFilteredSamplesList = emgFilteredSamplesList + channel * EMG_SAMPLES_NUMBER;
        float64* emgChannelActivationsList = emgActivationsList + channel * EMG_SAMPLES_NUMBER;
      
        for( size_t sampleIndex = 0; sampleIndex < EMG_SAMPLES_NUMBER; sampleIndex++ )
        {
          float64 emgNormalizedSample = emgChannelFilteredSamplesList[ sampleIndex ] / reference->maxList[ channel ];
          emgChannelActivationsList[ sampleIndex ] = ( exp( -2 * emgNormalizedSample ) - 1 ) / ( exp( -2 ) - 1 );
        }
      }
    }
    
    emgValuesList = emgActivationsList;
  }
  
  Thread_Exit( 0 );
  return NULL;
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
