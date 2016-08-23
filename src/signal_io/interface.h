#ifndef SIGNAL_IO_INTERFACE_H
#define SIGNAL_IO_INTERFACE_H

#include <math.h>
#ifndef M_PI
#define M_PI 3.14159
#endif

#include "modules.h"

#define SIGNAL_INPUT_CHANNEL_MAX_USES 5

#define SIGNAL_IO_TASK_INVALID_ID -1

#define SIGNAL_IO_INTERFACE( Namespace, INIT_FUNCTION ) \
        INIT_FUNCTION( int, Namespace, InitTask, const char* ) \
        INIT_FUNCTION( void, Namespace, EndTask, int ) \
        INIT_FUNCTION( void, Namespace, Reset, int ) \
        INIT_FUNCTION( bool, Namespace, HasError, int ) \
        INIT_FUNCTION( size_t, Namespace, GetMaxInputSamplesNumber, int ) \
        INIT_FUNCTION( size_t, Namespace, Read, int, unsigned int, double* ) \
        INIT_FUNCTION( bool, Namespace, AquireInputChannel, int, unsigned int ) \
        INIT_FUNCTION( void, Namespace, ReleaseInputChannel, int, unsigned int ) \
        INIT_FUNCTION( void, Namespace, EnableOutput, int, bool ) \
        INIT_FUNCTION( bool, Namespace, IsOutputEnabled, int ) \
        INIT_FUNCTION( bool, Namespace, Write, int, unsigned int, double ) \
        INIT_FUNCTION( bool, Namespace, AquireOutputChannel, int, unsigned int ) \
        INIT_FUNCTION( void, Namespace, ReleaseOutputChannel, int, unsigned int ) 

#endif // SIGNAL_IO_INTERFACE_H 
