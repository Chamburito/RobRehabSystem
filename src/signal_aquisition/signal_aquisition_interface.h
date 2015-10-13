#ifndef SIGNAL_AQUISITION_INTERFACE_H
#define SIGNAL_AQUISITION_INTERFACE_H

#include "interface.h"

#include <stdbool.h>
#include <stdint.h>

#define SIGNAL_AQUISITION_FUNCTIONS( interface, function_init ) \
        function_init( int, interface, InitTask, const char* ) \
        function_init( void, interface, EndTask, int ) \
        function_init( double*, interface, Read, int, unsigned int, size_t* ) \
        function_init( bool, interface, AquireChannel, int, unsigned int ) \
        function_init( void, interface, ReleaseChannel, int, unsigned int ) \
        function_init( size_t, interface, GetMaxSamplesNumber, int )

DEFINE_INTERFACE( SignalAquisition, SIGNAL_AQUISITION_FUNCTIONS )

#endif // SIGNAL_AQUISITION_INTERFACE_H 
