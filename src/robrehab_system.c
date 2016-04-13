#define _XOPEN_SOURCE 700

#ifdef ROBREHAB_SERVER
  #include "robrehab_network.h"
#elif ROBREHAB_CONTROL
  #include "robrehab_control.h"
#elif ROBREHAB_EMG
  #include "robrehab_emg.h"
#endif

#include <stdio.h>
#include <stdbool.h>
#include <time.h>
#include <signal.h>

const struct timespec UPDATE_TIMESPEC = { .tv_nsec = 1000000 * UPDATE_INTERVAL_MS };

static volatile bool isRunning = true;

void HandleExit( int dummyData )
{
  DEBUG_PRINT( "received exit signal: %d", dummyData );
  isRunning = false;
}

/* Program entry-point */
int main( int argc, char* argv[] )
{  
  time_t rawTime;
  time( &rawTime );
  DEBUG_PRINT( "starting control program at time: %s", ctime( &rawTime ) );
  
  signal( SIGINT, HandleExit );
  
  if( SUBSYSTEM.Init( "JSON" ) != -1 )
  {
    while( isRunning ) // Check for program termination conditions
    {
      SUBSYSTEM.Update();
    
      nanosleep( &UPDATE_TIMESPEC, NULL ); // Sleep to give the desired loop rate.
    }
  }
  
  time( &rawTime );
  DEBUG_PRINT( "ending control program at time: %s", ctime( &rawTime ) );

  SUBSYSTEM.End();
}
