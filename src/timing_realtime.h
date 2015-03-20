///////////////////////////////////////////////////////////////////////////////
///// Wrapper library for time measurement and thread sleeping (blocking) ///// 
///// using low level operating system native methods (Real-Time Version) /////
///////////////////////////////////////////////////////////////////////////////

#ifndef TIMING_H
#define TIMING_H

#include <rtutil.h>

// Make the calling thread wait for the given time ( in milisseconds )
extern inline void delay( unsigned long milisseconds )
{
  SleepUS( 1000 * milisseconds );
    
  return;
}

// Get system time in milisseconds
extern inline unsigned long get_exec_time_milisseconds()
{
  return ( (unsigned long) ( 1000 * GetTimeUS() ) );
}

// Get system time in seconds
extern inline unsigned int get_exec_time_seconds()
{
  return ( (unsigned long) ( 1000000 * GetTimeUS() ) );
}

#endif /* TIMING_H */
