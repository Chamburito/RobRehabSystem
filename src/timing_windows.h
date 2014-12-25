///////////////////////////////////////////////////////////////////////////////
///// Wrapper library for time measurement and thread sleeping (blocking) ///// 
///// using low level operating system native methods (Windows Version)   /////
///////////////////////////////////////////////////////////////////////////////

#ifndef TIMING_H
#define TIMING_H

#include <Windows.h>

LARGE_INTEGER TICKS_PER_SECOND;

// Make the calling thread wait for the given time ( in milisseconds )
void delay( unsigned long milisseconds )
{
    Sleep( milisseconds );
    
    return;
}

// Get system time in milisseconds
unsigned long get_exec_time_milisseconds()
{
    LARGE_INTEGER ticks;
	double exec_time;
    
    QueryPerformanceFrequency( &TICKS_PER_SECOND );
    QueryPerformanceCounter( &ticks );
    
    exec_time = (double) ticks.QuadPart / TICKS_PER_SECOND.QuadPart;
    
    return ( (unsigned long) (1000 * exec_time) );
}

// Get system time in seconds
unsigned int get_exec_time_seconds()
{
    LARGE_INTEGER ticks;
    unsigned int exec_time;
    
    QueryPerformanceFrequency( &TICKS_PER_SECOND );
    QueryPerformanceCounter( &ticks );
    
    exec_time = (unsigned int) ( ticks.QuadPart / TICKS_PER_SECOND.QuadPart );
    
    return exec_time;
}

#endif /* TIMING_H */
