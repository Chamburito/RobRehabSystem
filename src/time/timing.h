///////////////////////////////////////////////////////////////////////////////
///// Wrapper library for time measurement and thread sleeping (blocking) ///// 
///// using low level operating system native methods                     /////
///////////////////////////////////////////////////////////////////////////////

#ifndef TIMING_H
#define TIMING_H

#include "interfaces.h"

#define TIMING_INTERFACE( Namespace, INIT_FUNCTION ) \
        INIT_FUNCTION( void, Namespace, Delay, unsigned long ) \
        INIT_FUNCTION( unsigned long, Namespace, GetExecTimeMilliseconds, void ) \
        INIT_FUNCTION( double, Namespace, GetExecTimeSeconds, void )

DECLARE_NAMESPACE_INTERFACE( Timing, TIMING_INTERFACE )

#endif /* TIMING_H */
