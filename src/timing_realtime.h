///////////////////////////////////////////////////////////////////////////////
///// Wrapper library for time measurement and thread sleeping (blocking) ///// 
///// using low level operating system native methods (Real-Time Version) /////
///////////////////////////////////////////////////////////////////////////////

#ifndef TIMING_H
#define TIMING_H

#include <rtutil.h>

// Make the calling thread wait for the given time ( in milliseconds )
extern inline void Timing_Delay( unsigned long milliseconds )
{
  SleepUS( 1000 * milliseconds );
    
  return;
}

// Get system time in milliseconds
extern inline unsigned long Timing_GetExecTimeMilliseconds()
{
  return ( (unsigned long) ( GetTimeUS() / 1000 ) );
}

// Get system time in seconds
extern inline unsigned int Timing_GetExecTimeSeconds()
{
  return ( (unsigned int) ( GetTimeUS() / 1000000 ) );
}

#endif /* TIMING_H */
