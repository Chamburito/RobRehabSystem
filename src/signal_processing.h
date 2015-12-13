#ifndef SIGNAL_PROCESSING_H
#define SIGNAL_PROCESSING_H

#include <math.h>
#include <stdbool.h>

#include "debug/async_debug.h"

enum SignalProcessingPhase { SIGNAL_PROCESSING_PHASE_MEASUREMENT, SIGNAL_PROCESSING_PHASE_CALIBRATION, SIGNAL_PROCESSING_PHASE_OFFSET, SIGNAL_PROCESSING_PHASES_NUMBER };

#define FILTER_LENGTH 3

const uint8_t SIGNAL_PROCESSING_RECTIFY = 0x0F, SIGNAL_PROCESSING_NORMALIZE = 0xF0;

typedef struct _SignalProcessorData
{
  double signalLimitsList[ 2 ];
  double signalMean, signalOffset;
  size_t recordedSamplesCount;
  enum SignalProcessingPhase processingPhase;
  bool rectify, normalize;
  double inputFilterCoeffs[ FILTER_LENGTH ], outputFilterCoeffs[ FILTER_LENGTH ];
  double inputSamplesList[ FILTER_LENGTH ], outputSamplesList[ FILTER_LENGTH ];
}
SignalProcessorData;

typedef SignalProcessorData* SignalProcessor;


#define SIGNAL_PROCESSING_FUNCTIONS( namespace, function_init ) \
        function_init( SignalProcessor, namespace, CreateProcessor, uint8_t ) \
        function_init( void, namespace, DiscardProcessor, SignalProcessor ) \
        function_init( void, namespace, SetMaxFrequency, SignalProcessor, double ) \
        function_init( double, namespace, UpdateSignal, SignalProcessor, double ) \
        function_init( double, namespace, RevertTransformation, SignalProcessor, double ) \
        function_init( void, namespace, SetProcessorState, SignalProcessor, enum SignalProcessingPhase ) 

INIT_NAMESPACE_INTERFACE( SignalProcessing, SIGNAL_PROCESSING_FUNCTIONS )


SignalProcessor SignalProcessing_CreateProcessor( uint8_t flags )
{
  DEBUG_PRINT( "Trying to create processor type %u", flags );
  
  SignalProcessor newProcessor = (SignalProcessor) malloc( sizeof(SignalProcessorData) );
  memset( newProcessor, 0, sizeof(SignalProcessorData) );
  
  newProcessor->processingPhase = SIGNAL_PROCESSING_PHASE_MEASUREMENT;
  
  newProcessor->rectify = (bool) ( flags & SIGNAL_PROCESSING_RECTIFY );
  newProcessor->normalize = (bool) ( flags & SIGNAL_PROCESSING_NORMALIZE );
  
  newProcessor->inputFilterCoeffs[ 0 ] = 1.0;
        
  DEBUG_PRINT( "measure properties: rect: %u - norm: %u", newProcessor->rectify, newProcessor->normalize ); 
  
  return newProcessor;
}

void SignalProcessing_DiscardProcessor( SignalProcessor processor )
{
  if( processor == NULL ) return;
  
  free( processor );
}

void SignalProcessing_SetMaxFrequency( SignalProcessor processor, double relativeFrequency )
{
  if( processor == NULL ) return;
  
  if( relativeFrequency <= 0.0 ) return;
  
  if( relativeFrequency >= 0.5 ) relativeFrequency = 0.49;
  
  relativeFrequency *= 6.28;
  
  double outputGain = 4 + 2 * sqrt( 2.0 ) * relativeFrequency + relativeFrequency * relativeFrequency;
  
  processor->outputFilterCoeffs[ 1 ] = ( -8 + 2 * relativeFrequency * relativeFrequency ) / outputGain;
  processor->outputFilterCoeffs[ 2 ] = ( 4 - 2 * sqrt( 2.0 ) * relativeFrequency + relativeFrequency * relativeFrequency ) / outputGain;
  
  processor->inputFilterCoeffs[ 0 ] = relativeFrequency * relativeFrequency / outputGain;
  processor->inputFilterCoeffs[ 1 ] = 2 * processor->inputFilterCoeffs[ 0 ];
  processor->inputFilterCoeffs[ 2 ] = processor->inputFilterCoeffs[ 0 ];
}

double SignalProcessing_UpdateSignal( SignalProcessor processor, double newInputValue )
{
  if( processor == NULL ) return 0.0;
    
  if( processor->processingPhase == SIGNAL_PROCESSING_PHASE_OFFSET )
  {
    processor->signalMean += newInputValue;
    processor->recordedSamplesCount++;
  }
  else
  {
    newInputValue -= processor->signalOffset;

    if( processor->rectify ) newInputValue = fabs( newInputValue );

    for( int sampleIndex = FILTER_LENGTH - 1; sampleIndex > 0; sampleIndex++ )
    {
      processor->inputSamplesList[ sampleIndex ] = processor->inputSamplesList[ sampleIndex - 1 ];
      processor->outputSamplesList[ sampleIndex ] = processor->outputSamplesList[ sampleIndex - 1 ];
    }
    processor->inputSamplesList[ 0 ] = newInputValue;
    
    processor->outputSamplesList[ 0 ] = 0.0;
    for( size_t sampleIndex = 0; sampleIndex < FILTER_LENGTH; sampleIndex++ )
    {
      processor->outputSamplesList[ 0 ] -= processor->outputFilterCoeffs[ sampleIndex ] * processor->outputSamplesList[ sampleIndex ];
      processor->outputSamplesList[ 0 ] += processor->inputFilterCoeffs[ sampleIndex ] * processor->inputSamplesList[ sampleIndex ];
    }
    newInputValue = processor->outputSamplesList[ 0 ];
    
    if( processor->processingPhase == SIGNAL_PROCESSING_PHASE_CALIBRATION )
    {
      if( newInputValue > processor->signalLimitsList[ 1 ] ) processor->signalLimitsList[ 1 ] = newInputValue;
      else if( newInputValue < processor->signalLimitsList[ 0 ] ) processor->signalLimitsList[ 0 ] = newInputValue;
    }
    else if( processor->processingPhase == SIGNAL_PROCESSING_PHASE_MEASUREMENT )
    {
      if( processor->normalize && ( processor->signalLimitsList[ 0 ] != processor->signalLimitsList[ 1 ] ) )
      {
        if( newInputValue > processor->signalLimitsList[ 1 ] ) newInputValue = processor->signalLimitsList[ 1 ];
        else if( newInputValue < processor->signalLimitsList[ 0 ] ) newInputValue = processor->signalLimitsList[ 0 ];

        newInputValue = newInputValue / ( processor->signalLimitsList[ 1 ] - processor->signalLimitsList[ 0 ] );
      }
    }
  }
    
  return newInputValue;
}

double SignalProcessing_RevertTransformation( SignalProcessor processor, double value )
{
  if( processor == NULL ) return 0.0;
  
  if( processor->normalize )
  {
    value *= ( processor->signalLimitsList[ 1 ] - processor->signalLimitsList[ 0 ] );
    
    if( value > processor->signalLimitsList[ 1 ] ) value = processor->signalLimitsList[ 1 ];
    else if( value < processor->signalLimitsList[ 0 ] ) value = processor->signalLimitsList[ 0 ];
  }
  
  value += processor->signalOffset;
  
  return value;
}

void SignalProcessing_SetProcessorState( SignalProcessor processor, enum SignalProcessingPhase newProcessingPhase )
{
  if( processor == NULL ) return;
  
  if( newProcessingPhase < 0 || newProcessingPhase >= SIGNAL_PROCESSING_PHASES_NUMBER ) return;

  if( processor->processingPhase == SIGNAL_PROCESSING_PHASE_OFFSET )
  {
    if( processor->recordedSamplesCount > 0 ) 
      processor->signalOffset = processor->signalMean / processor->recordedSamplesCount;
  }
  
  if( newProcessingPhase == SIGNAL_PROCESSING_PHASE_CALIBRATION )
  {
    processor->signalLimitsList[ 1 ] = 0.0;
    processor->signalLimitsList[ 0 ] = 0.0;
  }
  else if( newProcessingPhase == SIGNAL_PROCESSING_PHASE_OFFSET )
  {
    processor->signalMean = 0.0;
    processor->recordedSamplesCount = 0;
  }
  
  processor->processingPhase = newProcessingPhase;
}

#endif // SIGNAL_PROCESSING_H