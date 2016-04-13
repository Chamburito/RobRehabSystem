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

const struct timespec UPDATE_TIMESPEC = { .tv_nsec = 1000000 * UPDATE_INTERVAL_MS };

/* Program entry-point */
int main( int argc, char* argv[] )
{  
  //SetDir( "C:\\ni-rt" );
  
  if( SUBSYSTEM.Init( "JSON" ) != -1 )
  {
    while( true ) // Check for program termination conditions
    {
      SUBSYSTEM.Update();
    
      nanosleep( &UPDATE_TIMESPEC, NULL ); // Sleep to give the desired loop rate.
    }
  }

  SUBSYSTEM.End();
}
