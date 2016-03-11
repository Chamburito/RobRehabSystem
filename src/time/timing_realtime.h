///////////////////////////////////////////////////////////////////////////////
///// Wrapper library for time measurement and thread sleeping (blocking) ///// 
///// using low level operating system native methods (Real-Time Version) /////
///////////////////////////////////////////////////////////////////////////////

#ifndef TIMING_H
#define TIMING_H

#include "interfaces.h"

#include <rtutil.h>

#define TIMING_FUNCTIONS( namespace, function_init ) \
        function_init( void, namespace, Delay, unsigned long ) \
        function_init( unsigned long, namespace, GetExecTimeMilliseconds, void ) \
        function_init( double, namespace, GetExecTimeSeconds, void )

INIT_NAMESPACE_INTERFACE( Timing, TIMING_FUNCTIONS )

// Make the calling thread wait for the given time ( in milliseconds )
inline void Timing_Delay( unsigned long milliseconds )
{
  SleepUS( 1000 * milliseconds );
    
  return;
}

// Get system time in milliseconds
inline unsigned long Timing_GetExecTimeMilliseconds()
{
  return ( (unsigned long) ( GetTimeUS() / 1000 ) );
}

// Get system time in seconds
inline double Timing_GetExecTimeSeconds()
{
  return ( (double) GetTimeUS() ) / 1000000.0;
}

#endif /* TIMING_H */
