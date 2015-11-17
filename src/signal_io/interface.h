#ifndef SIGNAL_IO_INTERFACE_H
#define SIGNAL_IO_INTERFACE_H

#include "interfaces.h"

#define SIGNAL_INPUT_CHANNEL_MAX_USES 5

const int SIGNAL_IO_TASK_INVALID_ID = -1;

#define SIGNAL_IO_FUNCTIONS( interface, function_init ) \
        function_init( int, interface, InitTask, const char* ) \
        function_init( void, interface, EndTask, int ) \
        function_init( void, interface, Reset, int ) \
        function_init( bool, interface, HasError, int ) \
        function_init( bool, interface, Read, int, unsigned int, double* ) \
        function_init( bool, interface, AquireInputChannel, int, unsigned int ) \
        function_init( void, interface, ReleaseInputChannel, int, unsigned int ) \
        function_init( void, interface, EnableOutput, int, bool ) \
        function_init( bool, interface, IsOutputEnabled, int ) \
        function_init( bool, interface, Write, int, unsigned int, double ) \
        function_init( bool, interface, AquireOutputChannel, int, unsigned int ) \
        function_init( void, interface, ReleaseOutputChannel, int, unsigned int ) 

DEFINE_INTERFACE( SignalIO, SIGNAL_IO_FUNCTIONS )

#endif // SIGNAL_IO_INTERFACE_H 
