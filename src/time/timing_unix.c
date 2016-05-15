////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  Copyright (c) 2016 Leonardo José Consoni                                  //
//                                                                            //
//  This file is part of RobRehabSystem.                                      //
//                                                                            //
//  RobRehabSystem is free software: you can redistribute it and/or modify    //
//  it under the terms of the GNU Lesser General Public License as published  //
//  by the Free Software Foundation, either version 3 of the License, or      //
//  (at your option) any later version.                                       //
//                                                                            //
//  RobRehabSystem is distributed in the hope that it will be useful,         //
//  but WITHOUT ANY WARRANTY; without even the implied warranty of            //
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the              //
//  GNU Lesser General Public License for more details.                       //
//                                                                            //
//  You should have received a copy of the GNU Lesser General Public License  //
//  along with RobRehabSystem. If not, see <http://www.gnu.org/licenses/>.    //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
///// Wrapper library for time measurement and thread sleeping (blocking) ///// 
///// using low level operating system native methods (Posix Version)     /////
///////////////////////////////////////////////////////////////////////////////

#include "time/timing.h"

#include <time.h>

DEFINE_NAMESPACE_INTERFACE( Timing, TIMING_INTERFACE )

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
