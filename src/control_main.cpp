///////////////////////////////////////////////////////////////////////////////////
/////          Minimal application for ExoKanguera DC motors control          /////
///////////////////////////////////////////////////////////////////////////////////

#include "control.h"
#include "robrehab_network.h"
#include <iostream>
#include <Windows.h>

using namespace std;


/////////////////////////////////////////////////////////////
//////////          Main program function          //////////
/////////////////////////////////////////////////////////////

int main( int argc, char** argv ) 
{
    // Start CAN network transmission
    for( int nodeId = 1; nodeId <= N_EPOS; nodeId++ )
      eposNetwork.StartPDOS( nodeId );

	while( true )
	{
    if( GetAsyncKeyState( VK_ESCAPE ) )
      break;
    else if( GetAsyncKeyState( VK_NUMPAD1 ) )
    {
      if( EPOS[0].active ) EPOS[0].Disable(); 
      else EPOS[0].Enable();
    }
    else if( GetAsyncKeyState( VK_NUMPAD2 ) )
    {
      if( EPOS[1].active ) EPOS[1].Disable(); 
      else EPOS[1].Enable();
    }
    else if( GetAsyncKeyState( VK_NUMPAD3 ) )
    {
      if( EPOS[2].active ) EPOS[2].Disable(); 
      else EPOS[2].Enable();
    }

    get_axes_data();
    proccess_incoming_messages();
    send_data_messages();
  }

    // End CAN network transmission
    eposNetwork.StopPDOS(1);

    delay( 2000 );
    
	  // End threading subsystem
    end_threading();

    return 0;
}


