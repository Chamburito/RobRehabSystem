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


#include <native/task.h>
#include <native/queue.h>
#include <native/intr.h>
#include <native/pipe.h>

#include <stdio.h>
#include <stdbool.h>
#include <time.h>
#include <signal.h>

#include "debug/async_debug.h"

#include "robrehab_subsystem.h"

const int STACK_SIZE 8192;
const int PRIORITY 1;

static RT_TASK updateTask;

static volatile bool isRunning = true;

void HandleExit( int dummyData )
{
  DEBUG_PRINT( "received exit signal: %d", dummyData );
  isRunning = false;
}

void RunUpdateLoop( void* args )
{
  rt_task_set_periodic( NULL. TM_NOW, 1000000 * UPDATE_INTERVAL_MS );
  
  while( isRunning ) // Check for program termination conditions
  {
    SubSystem.Update();
      
    rt_task_wait_period(); // Sleep to give the desired loop rate.
  }
}

int main( int argc, char* argv[] )
{ 
  time_t rawTime;
  time( &rawTime );
  DEBUG_PRINT( "starting control program at time: %s", ctime( &rawTime ) );
  
  signal( SIGINT, HandleExit );
  
  if( argc > 1 )
  {
    if( SubSystem.Init( argv[ 1 ], NULL, NULL ) != -1 )
    {
      rt_task_spawn( &updateTask, "RobRehabControlUpdate", STACK_SIZE, PRIORITY, 0, &RunUpdateLoop, NULL );
      
      rt_task_join( &updateTask );
    }
  }
  
  time( &rawTime );
  DEBUG_PRINT( "ending control program at time: %s", ctime( &rawTime ) );

  SubSystem.End();
  
  exit( 0 );
}
