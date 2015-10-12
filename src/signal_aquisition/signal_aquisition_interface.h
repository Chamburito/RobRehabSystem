#ifndef SIGNAL_AQUISITION_INTERFACE_H
#define SIGNAL_AQUISITION_INTERFACE_H

#include "interface.h"

#include <stdbool.h>
#include <stdint.h>

#define SignalAquisition( function_init ) \
        function_init( int, InitTask, const char* ) \
        function_init( void, EndTask, int ) \
        function_init( double*, Read, int, unsigned int, size_t* ) \
        function_init( bool, AquireChannel, int, unsigned int ) \
        function_init( void, ReleaseChannel, int, unsigned int ) \
        function_init( size_t, GetMaxSamplesNumber, int )

DEFINE_INTERFACE( SignalAquisition )

#endif // SIGNAL_AQUISITION_INTERFACE_H 
