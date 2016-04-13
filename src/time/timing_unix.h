///////////////////////////////////////////////////////////////////////////////
///// Wrapper library for time measurement and thread sleeping (blocking) ///// 
///// using low level operating system native methods (Posix Version)     /////
///////////////////////////////////////////////////////////////////////////////

#ifndef TIMING_H
#define TIMING_H

#include <time.h>

#include "interfaces.h"

#define TIMING_FUNCTIONS( namespace, function_init ) \
        function_init( void, namespace, Delay, unsigned long ) \
        function_init( unsigned long, namespace, GetExecTimeMilliseconds, void ) \
        function_init( double, namespace, GetExecTimeSeconds, void )

INIT_NAMESPACE_INTERFACE( Timing, TIMING_FUNCTIONS )

// Make the calling thread wait for the given time ( in milliseconds )
void Timing_Delay( unsigned long milliseconds )
{
    static struct timespec delayTime;// = { milliseconds / 1000, ( milliseconds % 1000 ) * 1000000 };
    static struct timespec remainingTime;
    
    delayTime.tv_sec = milliseconds / 1000;
    delayTime.tv_nsec = ( milliseconds % 1000 ) * 1000000;
    
    nanosleep( &delayTime, &remainingTime );
    
    return;
}

// Get system time in milisseconds
unsigned long Timing_GetExecTimeMilliseconds()
{
    struct timespec systemTime;
    
    //clock_getres( CLOCK_MONOTONIC, &ts );
    //printf( "time resolution: %ld s - %ld ns\n", ts.tv_sec, ts.tv_nsec );
  
    clock_gettime( CLOCK_MONOTONIC, &systemTime );
    
    unsigned long execTime = (unsigned long) ( 1000 * systemTime.tv_sec ) + (unsigned long) ( systemTime.tv_nsec / 1000000 );
    
    return execTime;
}

// Get system time in seconds
double Timing_GetExecTimeSeconds()
{
    struct timespec systemTime;
  
    clock_gettime( CLOCK_MONOTONIC, &systemTime );
    
    double execTime = (double) systemTime.tv_sec + ((double) systemTime.tv_nsec) / 1000000000.0;
    
    return execTime;
}

#endif /* TIMING_H */
