#ifndef SIGNAL_PROCESSING_H
#define SIGNAL_PROCESSING_H

#include <math.h>
#include <stdbool.h>

#include "filters.h"

#include "debug/async_debug.h"

enum SignalProcessingPhase { SIGNAL_PROCESSING_PHASE_MEASUREMENT, SIGNAL_PROCESSING_PHASE_CALIBRATION, SIGNAL_PROCESSING_PHASE_OFFSET, SIGNAL_PROCESSING_PHASES_NUMBER };

const uint8_t SIGNAL_PROCESSING_RECTIFY = 0x0F, SIGNAL_PROCESSING_NORMALIZE = 0xF0;

typedef struct _SignalData
{
  double samplingTime;
  double calibrationMax, calibrationMin;
  double samplesMean, offset;
  size_t recordedSamplesCount;
  enum SignalProcessingPhase processingPhase;
  bool isRectified, isNormalized;
}
SignalData;

typedef struct _SignalFilterData
{
  SignalData signalData;
  KalmanFilter kalmanFilter;
}
SignalFilterData;

typedef SignalFilterData* SignalFilter;


#define SIGNAL_PROCESSING_FUNCTIONS( namespace, function_init ) \
        function_init( SignalFilter, namespace, CreateFilter, uint8_t ) \
        function_init( void, namespace, DiscardFilter, SignalFilter ) \
        function_init( double*, namespace, UpdateFilter, SignalFilter, double ) \
        function_init( double, namespace, GetFilterOffset, SignalFilter ) \
        function_init( void, namespace, SetFilterState, SignalFilter, enum SignalProcessingPhase ) 

INIT_NAMESPACE_INTERFACE( SignalProcessing, SIGNAL_PROCESSING_FUNCTIONS )


SignalFilter SignalProcessing_CreateFilter( uint8_t flags )
{
  DEBUG_PRINT( "Trying to create filter type %u", flags );
  
  SignalFilter newSensor = (SignalFilter) malloc( sizeof(SignalFilterData) );
  memset( newSensor, 0, sizeof(SignalFilterData) );
  
  newSensor->signalData.processingPhase = SIGNAL_PROCESSING_PHASE_MEASUREMENT;
  
  newSensor->signalData.isRectified = (bool) ( flags & SIGNAL_PROCESSING_RECTIFY );
  newSensor->signalData.isNormalized = (bool) ( flags & SIGNAL_PROCESSING_NORMALIZE );
        
  DEBUG_PRINT( "measure properties: rect: %u - norm: %u", newSensor->signalData.isRectified, newSensor->signalData.isNormalized ); 
        
  newSensor->kalmanFilter = SimpleKalman.CreateFilter( 3, 0.0 );
  
  return newSensor;
}

void SignalProcessing_DiscardFilter( SignalFilter filter )
{
  if( filter == NULL ) return;
  
  SimpleKalman.DiscardFilter( filter->kalmanFilter );
  
  free( filter );
}

double* SignalProcessing_UpdateFilter( SignalFilter filter, double newSignalValue )
{
  if( filter == NULL ) return NULL;
  
  SignalData* signal = &(filter->signalData);
  
  double* filterOutput = NULL;
  
  DEBUG_PRINT( "updating filter %p", filter );
    
  if( signal->processingPhase == SIGNAL_PROCESSING_PHASE_OFFSET )
  {
    signal->samplesMean += newSignalValue;
    signal->recordedSamplesCount++;
  }
  else
  {
    newSignalValue -= signal->offset;

    if( signal->isRectified ) newSignalValue = fabs( newSignalValue );

    if( signal->processingPhase == SIGNAL_PROCESSING_PHASE_CALIBRATION )
    {
      if( newSignalValue > signal->calibrationMax ) signal->calibrationMax = newSignalValue;
      else if( newSignalValue < signal->calibrationMin ) signal->calibrationMin = newSignalValue;
    }
    else if( signal->processingPhase == SIGNAL_PROCESSING_PHASE_MEASUREMENT )
    {
      if( signal->isNormalized && ( signal->calibrationMin != signal->calibrationMax ) )
      {
        if( newSignalValue > signal->calibrationMax ) newSignalValue = signal->calibrationMax;
        else if( newSignalValue < signal->calibrationMin ) newSignalValue = signal->calibrationMin;

        newSignalValue = newSignalValue / ( signal->calibrationMax - signal->calibrationMin );
      }

      double deltaTime = Timing.GetExecTimeSeconds() - signal->samplingTime;
      filterOutput = SimpleKalman.Update( filter->kalmanFilter, newSignalValue, deltaTime );
    }
  }
  
  signal->samplingTime = Timing.GetExecTimeSeconds();
    
  return filterOutput;
}

double SignalProcessing_GetFilterOffset( SignalFilter filter )
{
  if( filter == NULL ) return 0.0;
  
  return filter->signalData.offset;
}

void SignalProcessing_SetFilterState( SignalFilter filter, enum SignalProcessingPhase newProcessingPhase )
{
  if( filter == NULL ) return;
  
  if( newProcessingPhase < 0 || newProcessingPhase >= SIGNAL_PROCESSING_PHASES_NUMBER ) return;
  
  SignalData* signal = &(filter->signalData);

  if( signal->processingPhase == SIGNAL_PROCESSING_PHASE_OFFSET )
  {
    if( signal->recordedSamplesCount > 0 ) 
      signal->offset = signal->samplesMean / signal->recordedSamplesCount;
  }
  
  if( newProcessingPhase == SIGNAL_PROCESSING_PHASE_CALIBRATION )
  {
    signal->calibrationMax = 0.0;
    signal->calibrationMin = 0.0;
  }
  else if( newProcessingPhase == SIGNAL_PROCESSING_PHASE_OFFSET )
  {
    signal->samplesMean = 0.0;
    signal->recordedSamplesCount = 0;
  }

  SimpleKalman.Reset( filter->kalmanFilter, 0.0 );
  signal->samplingTime = Timing.GetExecTimeSeconds();
  
  signal->processingPhase = newProcessingPhase;
}


#endif // SIGNAL_PROCESSING_H
