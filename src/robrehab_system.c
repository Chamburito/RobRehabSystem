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
