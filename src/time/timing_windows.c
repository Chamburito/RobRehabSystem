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
///// using low level operating system native methods (Windows Version)   /////
///////////////////////////////////////////////////////////////////////////////

#include "time/timing.h"

#include <Windows.h>

DEFINE_NAMESPACE_INTERFACE( Timing, TIMING_INTERFACE )

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
