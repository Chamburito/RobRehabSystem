#ifndef TIMING_H
#define TIMING_H

#include <Windows.h>

LARGE_INTEGER TICKS_PER_SECOND;

void delay( unsigned long milisseconds )
{
    Sleep( milisseconds );
    
    return;
}

unsigned long get_exec_time_milisseconds()
{
    LARGE_INTEGER ticks;
	double exec_time;
    
    QueryPerformanceFrequency( &TICKS_PER_SECOND );
    QueryPerformanceCounter( &ticks );
    
    exec_time = (double) ticks.QuadPart / TICKS_PER_SECOND.QuadPart;
    
    return ( (unsigned long) (1000 * exec_time) );
}

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
