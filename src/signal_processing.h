#ifndef SIGNAL_PROCESSING_H
#define SIGNAL_PROCESSING_H

#include <math.h>
#include <stdbool.h>

#include "filters.h"

#include "debug/async_debug.h"

enum SignalProcessingPhase { SIGNAL_PROCESSING_PHASE_MEASUREMENT, SIGNAL_PROCESSING_PHASE_CALIBRATION, SIGNAL_PROCESSING_PHASE_OFFSET, SIGNAL_PROCESSING_PHASES_NUMBER };

const uint8_t SIGNAL_PROCESSING_RECTIFY = 0x0F, SIGNAL_PROCESSING_NORMALIZE = 0xF0;

typedef struct _SignalProcessorData
{
  double calibrationMax, calibrationMin;
  double samplesMean, offset;
  size_t recordedSamplesCount;
  enum SignalProcessingPhase processingPhase;
  bool isRectified, isNormalized;
}
SignalProcessorData;

typedef SignalProcessorData* SignalProcessor;


#define SIGNAL_PROCESSING_FUNCTIONS( namespace, function_init ) \
        function_init( SignalProcessor, namespace, CreateProcessor, uint8_t ) \
        function_init( void, namespace, DiscardProcessor, SignalProcessor ) \
        function_init( double, namespace, UpdateSignal, SignalProcessor, double ) \
        function_init( double, namespace, GetSignalOffset, SignalProcessor ) \
        function_init( void, namespace, SetProcessorState, SignalProcessor, enum SignalProcessingPhase ) 

INIT_NAMESPACE_INTERFACE( SignalProcessing, SIGNAL_PROCESSING_FUNCTIONS )


SignalProcessor SignalProcessing_CreateProcessor( uint8_t flags )
{
  DEBUG_PRINT( "Trying to create processor type %u", flags );
  
  SignalProcessor newProcessor = (SignalProcessor) malloc( sizeof(SignalProcessorData) );
  memset( newProcessor, 0, sizeof(SignalProcessorData) );
  
  newProcessor->processingPhase = SIGNAL_PROCESSING_PHASE_MEASUREMENT;
  
  newProcessor->isRectified = (bool) ( flags & SIGNAL_PROCESSING_RECTIFY );
  newProcessor->isNormalized = (bool) ( flags & SIGNAL_PROCESSING_NORMALIZE );
        
  DEBUG_PRINT( "measure properties: rect: %u - norm: %u", newProcessor->isRectified, newProcessor->isNormalized ); 
  
  return newProcessor;
}

void SignalProcessing_DiscardProcessor( SignalProcessor processor )
{
  if( processor == NULL ) return;
  
  free( processor );
}

double SignalProcessing_UpdateProcessor( SignalProcessor processor, double newSignalValue )
{
  if( processor == NULL ) return 0.0;
    
  if( processor->processingPhase == SIGNAL_PROCESSING_PHASE_OFFSET )
  {
    processor->samplesMean += newSignalValue;
    processor->recordedSamplesCount++;
  }
  else
  {
    newSignalValue -= processor->offset;

    if( processor->isRectified ) newSignalValue = fabs( newSignalValue );

    if( processor->processingPhase == SIGNAL_PROCESSING_PHASE_CALIBRATION )
    {
      if( newSignalValue > processor->calibrationMax ) processor->calibrationMax = newSignalValue;
      else if( newSignalValue < processor->calibrationMin ) processor->calibrationMin = newSignalValue;
    }
    else if( processor->processingPhase == SIGNAL_PROCESSING_PHASE_MEASUREMENT )
    {
      if( processor->isNormalized && ( processor->calibrationMin != processor->calibrationMax ) )
      {
        if( newSignalValue > processor->calibrationMax ) newSignalValue = processor->calibrationMax;
        else if( newSignalValue < processor->calibrationMin ) newSignalValue = processor->calibrationMin;

        newSignalValue = newSignalValue / ( processor->calibrationMax - processor->calibrationMin );
      }
    }
  }
    
  return newSignalValue;
}

double SignalProcessing_GetProcessorOffset( SignalProcessor processor )
{
  if( processor == NULL ) return 0.0;
  
  return processor->offset;
}

void SignalProcessing_SetProcessorState( SignalProcessor processor, enum SignalProcessingPhase newProcessingPhase )
{
  if( processor == NULL ) return;
  
  if( newProcessingPhase < 0 || newProcessingPhase >= SIGNAL_PROCESSING_PHASES_NUMBER ) return;

  if( processor->processingPhase == SIGNAL_PROCESSING_PHASE_OFFSET )
  {
    if( processor->recordedSamplesCount > 0 ) 
      processor->offset = processor->samplesMean / processor->recordedSamplesCount;
  }
  
  if( newProcessingPhase == SIGNAL_PROCESSING_PHASE_CALIBRATION )
  {
    processor->calibrationMax = 0.0;
    processor->calibrationMin = 0.0;
  }
  else if( newProcessingPhase == SIGNAL_PROCESSING_PHASE_OFFSET )
  {
    processor->samplesMean = 0.0;
    processor->recordedSamplesCount = 0;
  }
  
  processor->processingPhase = newProcessingPhase;
}


#endif // SIGNAL_PROCESSING_H
