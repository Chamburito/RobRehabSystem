////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  Copyright (c) 2016 Leonardo José Consoni                                  //
//                                                                            //
//  This file is part of RobRehabSystem.                                      //
//                                                                            //
//  RobRehabSystem is free software: you can redistribute it and/or modify    //
//  it under the terms of the GNU Lesser General Public License as published  //
//  by the Free Software Foundation, either version 3 of the License, or      //
//  (at your option) any later version.                                       //
//                                                                            //
//  RobRehabSystem is distributed in the hope that it will be useful,         //
//  but WITHOUT ANY WARRANTY; without even the implied warranty of            //
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the              //
//  GNU Lesser General Public License for more details.                       //
//                                                                            //
//  You should have received a copy of the GNU Lesser General Public License  //
//  along with RobRehabSystem. If not, see <http://www.gnu.org/licenses/>.    //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////


#include <math.h>
#include <stdbool.h>

#include "debug/async_debug.h"

#include "signal_processing.h"


#define FILTER_LENGTH 3

struct _SignalProcessorData
{
  double inputGain;
  double signalLimitsList[ 2 ];
  double signalOffset;
  size_t recordedSamplesCount;
  enum SignalProcessingPhase processingPhase;
  bool rectify, normalize;
  double inputFilterCoeffs[ FILTER_LENGTH ], outputFilterCoeffs[ FILTER_LENGTH ];
  double inputSamplesList[ FILTER_LENGTH ], outputSamplesList[ FILTER_LENGTH ];
};

DEFINE_NAMESPACE_INTERFACE( SignalProcessing, SIGNAL_PROCESSING_INTERFACE )


SignalProcessor SignalProcessing_CreateProcessor( uint8_t flags )
{
  DEBUG_PRINT( "Trying to create processor type %u", flags );
  
  SignalProcessor newProcessor = (SignalProcessor) malloc( sizeof(SignalProcessorData) );
  memset( newProcessor, 0, sizeof(SignalProcessorData) );
  
  newProcessor->inputGain = 1.0;
  
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

void SignalProcessing_SetInputGain( SignalProcessor processor, double inputGain )
{
  if( processor == NULL ) return;
  
  processor->inputGain = inputGain;
  
  DEBUG_PRINT( "setting input gain for processor %p: %g", processor, processor->inputGain );
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
  
  DEBUG_PRINT( "filter: y(%+g %+g %+g) x(%+g %+g %+g)", -processor->outputFilterCoeffs[ 0 ], -processor->outputFilterCoeffs[ 1 ], -processor->outputFilterCoeffs[ 2 ],
                                                        processor->inputFilterCoeffs[ 0 ], processor->inputFilterCoeffs[ 1 ], processor->inputFilterCoeffs[ 2 ] );
}

double SignalProcessing_UpdateSignal( SignalProcessor processor, double* newInputValuesList, size_t newValuesNumber )
{
  if( processor == NULL ) return 0.0;
  
  double newInputValue = processor->outputSamplesList[ 0 ];
  
  if( processor->processingPhase == SIGNAL_PROCESSING_PHASE_OFFSET )
  {
    if( newValuesNumber > 0 )
    {
      processor->signalOffset *= processor->recordedSamplesCount;
      for( size_t valueIndex = 0; valueIndex < newValuesNumber; valueIndex++ )
      {
        processor->signalOffset += newInputValuesList[ valueIndex ] * processor->inputGain;
        processor->recordedSamplesCount++;
      }
      processor->signalOffset /= processor->recordedSamplesCount;
    }
    newInputValue = processor->signalOffset;
  }
  else
  {
    for( size_t valueIndex = 0; valueIndex < newValuesNumber; valueIndex++ )
    {
      newInputValue = newInputValuesList[ valueIndex ] * processor->inputGain - processor->signalOffset;

      if( processor->rectify ) newInputValue = fabs( newInputValue );

      for( int sampleIndex = FILTER_LENGTH - 1; sampleIndex > 0; sampleIndex-- )
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
  
  if( newProcessingPhase >= SIGNAL_PROCESSING_PHASES_NUMBER ) return;

  DEBUG_PRINT( "current: %x - new: %x", processor->processingPhase, newProcessingPhase );
  
  if( processor->processingPhase == SIGNAL_PROCESSING_PHASE_OFFSET )
    DEBUG_PRINT( "new signal offset: %g", processor->signalOffset );
  
  if( processor->processingPhase == SIGNAL_PROCESSING_PHASE_CALIBRATION )
    DEBUG_PRINT( "new signal limits: %g %g", processor->signalLimitsList[ 0 ], processor->signalLimitsList[ 1 ] );
  
  if( newProcessingPhase == SIGNAL_PROCESSING_PHASE_CALIBRATION )
  {
    processor->signalLimitsList[ 1 ] = 0.0;
    processor->signalLimitsList[ 0 ] = 0.0;
  }
  else if( newProcessingPhase == SIGNAL_PROCESSING_PHASE_OFFSET )
  {
    processor->signalOffset = 0.0;
    processor->recordedSamplesCount = 0;
  }
  
  processor->processingPhase = newProcessingPhase;
}
