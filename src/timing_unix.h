#ifndef TIMING_H
#define TIMING_H

#include <time.h>
#include <unistd.h>

void delay( unsigned long milisseconds )
{
    struct timespec delay_ts = { milisseconds / 1000, ( milisseconds % 1000 ) * 1000000 };
    static struct timespec remain_ts;
    
    nanosleep( &delay_ts, &remain_ts );
    
    return;
}

unsigned long get_exec_time_milisseconds()
{
    struct timespec ts;
    
    //clock_getres( CLOCK_MONOTONIC, &ts );
    //printf( "time resolution: %ld s - %ld ns\n", ts.tv_sec, ts.tv_nsec );
  
    clock_gettime( CLOCK_MONOTONIC, &ts );
    
    unsigned long exec_time = (unsigned long) ( 1000 * ts.tv_sec ) + (unsigned long) ( ts.tv_nsec / 1000000 );
    
    return exec_time;
}

unsigned int get_exec_time_seconds()
{
    struct timespec ts;
  
    clock_gettime( CLOCK_MONOTONIC, &ts );
    
    unsigned int exec_time = (unsigned int) ts.tv_sec + (unsigned int) ( ts.tv_nsec / 1000000000 );
    
    return exec_time;
}

#endif /* TIMING_H */
