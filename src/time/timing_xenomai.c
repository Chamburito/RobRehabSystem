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
///// using low level operating system native methods (Xenomai Version)   /////
///////////////////////////////////////////////////////////////////////////////

#include "time/timing.h"

#include <native/task.h>
#include <native/timer.h>

DEFINE_NAMESPACE_INTERFACE( Timing, TIMING_INTERFACE )

// Make the calling thread wait for the given time ( in milliseconds )
void Timing_Delay( unsigned long waitMilliseconds )
{
  SRTIME waitNanoseconds = 1000000 * waitMilliseconds;
  SRTIME waitTicks = rt_timer_ns2ticks( waitNanoseconds ); 
  
  rt_task_sleep( (RTIME) waitTicks );
}

// Get system time in milisseconds
unsigned long Timing_GetExecTimeMilliseconds()
{
  SRTIME execTimeNanoseconds = rt_timer_tsc2ns( rt_timer_tsc() ) ;
  
  unsigned long execTimeMilliseconds = (unsigned long) ( execTimeNanoseconds / 1000000 );
    
  return execTime;
}

// Get system time in seconds
double Timing_GetExecTimeSeconds()
{
    SRTIME execTimeNanoseconds = rt_timer_tsc2ns( rt_timer_tsc() ) ;
    
    double execTimeSeconds = (double) ( execTimeNanoseconds / 1000000000.0 );
    
    return execTimeSeconds;
}
