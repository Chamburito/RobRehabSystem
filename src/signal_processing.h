#ifndef SIGNAL_PROCESSING_H
#define SIGNAL_PROCESSING_H

#include <stdint.h>

#include "namespaces.h"


enum SignalProcessingPhase { SIGNAL_PROCESSING_PHASE_MEASUREMENT, SIGNAL_PROCESSING_PHASE_CALIBRATION, SIGNAL_PROCESSING_PHASE_OFFSET, SIGNAL_PROCESSING_PHASES_NUMBER };

#define SIGNAL_PROCESSING_RECTIFY 0x0F
#define SIGNAL_PROCESSING_NORMALIZE 0xF0

typedef struct _SignalProcessorData SignalProcessorData;
typedef SignalProcessorData* SignalProcessor;

#define SIGNAL_PROCESSING_INTERFACE( Namespace, INIT_FUNCTION ) \
        INIT_FUNCTION( SignalProcessor, Namespace, CreateProcessor, uint8_t ) \
        INIT_FUNCTION( void, Namespace, DiscardProcessor, SignalProcessor ) \
        INIT_FUNCTION( void, Namespace, SetInputGain, SignalProcessor, double ) \
        INIT_FUNCTION( void, Namespace, SetMaxFrequency, SignalProcessor, double ) \
        INIT_FUNCTION( double, Namespace, UpdateSignal, SignalProcessor, double*, size_t ) \
        INIT_FUNCTION( double, Namespace, RevertTransformation, SignalProcessor, double ) \
        INIT_FUNCTION( void, Namespace, SetProcessorState, SignalProcessor, enum SignalProcessingPhase ) 

DECLARE_NAMESPACE_INTERFACE( SignalProcessing, SIGNAL_PROCESSING_INTERFACE )


#endif // SIGNAL_PROCESSING_H
