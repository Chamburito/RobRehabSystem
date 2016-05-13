////////////////////////////////////////////////////////////////////////////////
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
//  along with Foobar. If not, see <http://www.gnu.org/licenses/>.            //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////


#define _XOPEN_SOURCE 700

#include <stdio.h>
#include <stdbool.h>
#include <time.h>
#include <signal.h>

#include "debug/async_debug.h"

#include "robrehab_subsystem.h"


static volatile bool isRunning = true;

void HandleExit( int dummyData )
{
  DEBUG_PRINT( "received exit signal: %d", dummyData );
  isRunning = false;
}

/* Program entry-point */
int main( int argc, char* argv[] )
{
  const struct timespec UPDATE_TIMESPEC = { .tv_nsec = 1000000 * UPDATE_INTERVAL_MS };
  
  time_t rawTime;
  time( &rawTime );
  DEBUG_PRINT( "starting control program at time: %s", ctime( &rawTime ) );
  
  signal( SIGINT, HandleExit );
  
  if( argc > 1 )
  {
    if( SubSystem.Init( argv[ 1 ] ) != -1 )
    {
      while( isRunning ) // Check for program termination conditions
      {
        SubSystem.Update();
      
        nanosleep( &UPDATE_TIMESPEC, NULL ); // Sleep to give the desired loop rate.
      }
    }
  }
  
  time( &rawTime );
  DEBUG_PRINT( "ending control program at time: %s", ctime( &rawTime ) );

  SubSystem.End();
  
  exit( 0 );
}
