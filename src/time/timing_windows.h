///////////////////////////////////////////////////////////////////////////////
///// Wrapper library for time measurement and thread sleeping (blocking) ///// 
///// using low level operating system native methods (Windows Version)   /////
///////////////////////////////////////////////////////////////////////////////

#ifndef TIMING_H
#define TIMING_H

#include <Windows.h>

#include "interfaces.h"

#define TIMING_FUNCTIONS( namespace, function_init ) \
        function_init( void, namespace, Delay, unsigned long ) \
        function_init( unsigned long, namespace, GetExecTimeMilliseconds, void ) \
        function_init( double, namespace, GetExecTimeSeconds, void )

INIT_NAMESPACE_INTERFACE( Timing, TIMING_FUNCTIONS )

LARGE_INTEGER TICKS_PER_SECOND;

// Make the calling thread wait for the given time ( in milliseconds )
void Timing_Delay( unsigned long milliseconds )
{
    Sleep( milliseconds );
    
    return;
}

// Get system time in milliseconds
unsigned long Timing_GetExecTimeMilliseconds()
{
    LARGE_INTEGER ticks;
	  double exec_time;
    
    QueryPerformanceFrequency( &TICKS_PER_SECOND );
    QueryPerformanceCounter( &ticks );
    
    exec_time = (double) ticks.QuadPart / TICKS_PER_SECOND.QuadPart;
    
    return ( (unsigned long) (1000 * exec_time) );
}

// Get system time in seconds
double Timing_GetExecTimeSeconds()
{
    LARGE_INTEGER ticks;
    double exec_time;
    
    QueryPerformanceFrequency( &TICKS_PER_SECOND );
    QueryPerformanceCounter( &ticks );
    
    exec_time = (double) ticks.QuadPart / TICKS_PER_SECOND.QuadPart;
    
    return exec_time;
}

#endif /* TIMING_H */
