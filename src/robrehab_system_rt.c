/*******************************************************************************
* This example uses the Network Variable library to communicate between a
* DLL running on a real-time target and an executable running on a host machine.
* The real-time DLL generates and publishes sine wave data based on the 
* amplitude and frequency specified by the host executable. When the host 
* executable exits it signals the real-time DLL to exit as well.
*
* This example uses several network variables. Two network variables communicate 
* the amplitude and frequency information from the host executable to the real-
* time DLL. Another network variable publishes the sine wave data from the real-
* time DLL to the host executable. One more network variable indicates that the 
* host executable is exiting so the real-time DLL can exit as well.
*
* The real-time DLL uses multiple threads. The main thread publishes data while
* Network Variable Library threads run callbacks when network variable values
* are updated. The real-time DLL uses thread safe variables to pass data between
* the network variable callbacks and the main thread.
*
* NOTE: This example requires the LabWindows/CVI Real-Time Module.
*
* This example consists of two projects:
*
* NetworkVariableHost.prj - This project builds an executable with a user 
* interface. This executable communicates with the real-time DLL over TCP via 
* the Network Variable Library.
*
* NetworkVariableRT.prj - This project builds a DLL that will run on a real-time
* target. This DLL interacts with the host executable over TCP via the Network
* Variable Library. Download and run the DLL on a real-time target. See the 
* LabWindows/CVI documentation for detailed instructions.
*
* Run the real-time DLL project first and then run the host executable. When you
* exit the host executable the real-time DLL project will also exit.
*******************************************************************************/

/* Include files */

#ifdef ROBREHAB_SERVER
  #include "robrehab_network.h"
#elif ROBREHAB_CONTROL
  #include "robrehab_control.h"
#elif ROBREHAB_EMG
  #include "robrehab_emg.h"
#endif

#include <utility.h>
#include <rtutil.h>
#include <analysis.h>
#include <cvinetv.h>
#include <cvirte.h>
#include <ansi_c.h>

/* Program entry-point */
void CVIFUNC_C RTmain( void )
{
	if( InitCVIRTE( 0, 0, 0 ) == 0 )
		return;
	
	/*int status;
	int systemStarted = 0;
	
	while( !RTIsShuttingDown() && !systemStarted )
	{
		status = CNVProcessIsRunning( "system", &systemStarted );
		SleepUS( 10000 );
	}
  
  DEBUG_PRINT( "Network variables ready on thread %lx", THREAD_ID );*/
  
  SetDir( "C:\\ni-rt" );
  
  if( SUBSYSTEM.Init() != -1 )
  {
  	while( !RTIsShuttingDown() ) // Check for program termination conditions
  	{
      SUBSYSTEM.Update();
    
      SleepUntilNextMultipleUS( 1000 * UPDATE_INTERVAL_MS ); // Sleep to give the desired loop rate.
  	}
  }

  SUBSYSTEM.End();

	CloseCVIRTE();
}

