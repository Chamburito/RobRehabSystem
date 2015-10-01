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
//#include <ansi_c.h>
#include <stdbool.h>
#include <cvinetv.h>
#include <cvirte.h>		
#include <userint.h>

#include "ip_network/cvirte_ip_connection.h"
#include "RobRehabControlGUI.h"
#include "common.h"

enum SHMControlFloats { SHM_CONTROL_POSITION, SHM_CONTROL_VELOCITY, SHM_CONTROL_FORCE, SHM_CONTROL_STIFFNESS = SHM_CONTROL_FORCE,
                        SHM_CONTROL_ACCELERATION, SHM_CONTROL_DAMPING = SHM_CONTROL_ACCELERATION, SHM_CONTROL_TIME, SHM_CONTROL_FLOATS_NUMBER };

enum SHMControlBytes { SHM_COMMAND_DISABLE, SHM_COMMAND_ENABLE, SHM_COMMAND_RESET, SHM_COMMAND_CALIBRATE,
                       SHM_STATE_DISABLED = SHM_COMMAND_DISABLE, SHM_STATE_ENABLED, SHM_STATE_ERROR, SHM_CONTROL_BYTES_NUMBER };

/* Global variables */
static int panel;

/* Function prototypes */
static void* UpdateData( void* );

int axisID = 0;

int motorEnabled = 0;
double maxStiffness = 0.0;

int calibrationRunning = 0;
double maxReach = 0.0, minReach = 0.0;
double positionOffset = 0.0;

Thread dataConnectionThreadID;
bool isDataUpdateRunning = false;

char address[ 256 ] = "169.254.110.158";

int infoClientID, dataClientID;

const double TOTAL_CURVE_INTERVAL = 2.22;
const double CONTROL_SAMPLING_INTERVAL = 0.005;
const double NORMALIZED_SAMPLING_INTERVAL = CONTROL_SAMPLING_INTERVAL / TOTAL_CURVE_INTERVAL;

static double referenceValues[ NUM_POINTS ];
size_t setpointIndex = 0;

void InitUserInterface( void );

/* Program entry-point */
int main( int argc, char *argv[] )
{
	if( InitCVIRTE( 0, argv, 0 ) == 0 )
		return -1;
  
  // Get address of real-time target.
	//PromptPopup("Prompt", "Enter Real-Time Target Name/IP Address:", address, sizeof(address) - 1);
  
  infoClientID = AsyncIPConnection_Open( address, "50000", TCP );
  char* infoMessage = NULL;
  while( infoMessage == NULL )
    infoMessage = AsyncIPConnection_ReadMessage( infoClientID );
  
  fprintf( stderr, "received info message: %s\n", infoMessage );
  
  char resetMessage[ 256 ] = { 1, 0, SHM_COMMAND_RESET };
  AsyncIPConnection_WriteMessage( infoClientID, resetMessage );
  
  infoMessage = NULL;
  while( infoMessage == NULL )
    infoMessage = AsyncIPConnection_ReadMessage( infoClientID );
  
  fprintf( stderr, "received info message: %s\n", infoMessage );
  
  dataConnectionThreadID = Threading_StartThread( UpdateData, NULL, THREAD_JOINABLE );
  
	if( (panel = LoadPanel( 0, "RobRehabControlGUI.uir", PANEL )) < 0 )
		return -1;

	InitUserInterface();
	DisplayPanel( panel );
	RunUserInterface();
	
  isDataUpdateRunning = false;
  Threading_WaitExit( dataConnectionThreadID, 5000 );
  
	AsyncIPConnection_WriteMessage( infoClientID, resetMessage );
  
  AsyncIPConnection_Close( infoClientID ); 
  
	DiscardPanel( panel );
	
  CloseCVIRTE();
  
	return 0;
}

static void* UpdateData( void* callbackData )
{
  const size_t WAIT_SAMPLES = 10;
  const double SETPOINT_UPDATE_INTERVAL = WAIT_SAMPLES * CONTROL_SAMPLING_INTERVAL;
  
  float measuresList[ SHM_CONTROL_FLOATS_NUMBER ], setpointsList[ SHM_CONTROL_FLOATS_NUMBER ];
  
  double positionValues[ NUM_POINTS ], velocityValues[ NUM_POINTS ], accelerationValues[ NUM_POINTS ], torqueValues[ NUM_POINTS ];
  size_t measureIndex = 0;
  
  char dataMessageOut[ IP_MAX_MESSAGE_LENGTH ];
  dataMessageOut[ 0 ] = 1;
  dataMessageOut[ 1 ] = 0;
  dataMessageOut[ 2 ] = 0xFF;
  
  double initialTime = Timing_GetExecTimeSeconds();
  double elapsedTime = 0.0, deltaTime = 0.0, absoluteTime = initialTime; 
  
  size_t setpointIndex = 0;
  
  int dataClientID = AsyncIPConnection_Open( address, "50001", UDP );
  
  if( dataClientID != -1 ) isDataUpdateRunning = true;
  
  while( isDataUpdateRunning )
  {
    deltaTime += ( Timing_GetExecTimeSeconds()  - absoluteTime );
    absoluteTime = Timing_GetExecTimeSeconds();
    elapsedTime = absoluteTime - initialTime;
    
    char* messageIn = AsyncIPConnection_ReadMessage( dataClientID );
    if( messageIn != NULL )
    {
      if( messageIn[ 0 ] == 1 && messageIn[ 1 ] == 0 )
      {
        memcpy( measuresList, messageIn + 2, sizeof(float) * SHM_CONTROL_FLOATS_NUMBER );
        
        positionValues[ measureIndex ] = measuresList[ SHM_CONTROL_POSITION ];
        velocityValues[ measureIndex ] = measuresList[ SHM_CONTROL_VELOCITY ];
        accelerationValues[ measureIndex ] = measuresList[ SHM_CONTROL_ACCELERATION ];
        torqueValues[ measureIndex ] = measuresList[ SHM_CONTROL_FORCE ];
        
        if( ++measureIndex >= NUM_POINTS )
        {
          DeleteGraphPlot( panel, PANEL_GRAPH_1, -1, VAL_DELAYED_DRAW );
          PlotY( panel, PANEL_GRAPH_1, positionValues, NUM_POINTS, VAL_DOUBLE, VAL_FAT_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_YELLOW );
          PlotY( panel, PANEL_GRAPH_1, velocityValues, NUM_POINTS, VAL_DOUBLE, VAL_THIN_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_GREEN );
          PlotY( panel, PANEL_GRAPH_1, accelerationValues, NUM_POINTS, VAL_DOUBLE, VAL_THIN_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_BLUE );
          
          DeleteGraphPlot( panel, PANEL_GRAPH_2, -1, VAL_DELAYED_DRAW );
          PlotY( panel, PANEL_GRAPH_2, torqueValues, NUM_POINTS, VAL_DOUBLE, VAL_THIN_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_RED );
          
          measureIndex = 0;
        }
      }
    }
    
    if( deltaTime >= SETPOINT_UPDATE_INTERVAL )
    {
      setpointsList[ SHM_CONTROL_POSITION ] = fmod( elapsedTime / TOTAL_CURVE_INTERVAL, 1.0 );
      setpointsList[ SHM_CONTROL_VELOCITY ] = 1.0 / TOTAL_CURVE_INTERVAL;
      setpointsList[ SHM_CONTROL_STIFFNESS ] = maxStiffness;
      setpointsList[ SHM_CONTROL_TIME ] = SETPOINT_UPDATE_INTERVAL;
      
      for( size_t i = 0; i < WAIT_SAMPLES; i++ )
        referenceValues[ ( setpointIndex + i ) % NUM_POINTS ] = setpointsList[ SHM_CONTROL_POSITION ];
      setpointIndex += WAIT_SAMPLES;
    
      memcpy( dataMessageOut + 3, setpointsList, sizeof(float) * SHM_CONTROL_FLOATS_NUMBER );
    
      deltaTime = 0.0;
      
      AsyncIPConnection_WriteMessage( dataClientID, dataMessageOut );
    }
  }
  
  AsyncIPConnection_Close( dataClientID );
  
  return NULL;
}

static void InitUserInterface( void )
{
	// Manually call the control callbacks to write the initial values for
	// amplitude and frequency to the network variables.
  SetCtrlVal( panel, PANEL_MOTOR_TOGGLE, 0 );
  ChangeStateCallback( panel, PANEL_MOTOR_TOGGLE, EVENT_COMMIT, 0, 0, 0 );
  
	ChangeValueCallback( panel, PANEL_STIFFNESS, EVENT_COMMIT, 0, 0, 0 );
}

int CVICALLBACK ChangeStateCallback( int panel, int control, int event, void* callbackData, int eventData1, int eventData2 )
{
  static char commandMessage[ IP_MAX_MESSAGE_LENGTH ];
  commandMessage[ 0 ] = 1;
  commandMessage[ 1 ] = 0;
  
  int enabled;
  
	switch( event )
	{
		case EVENT_COMMIT:
			// Get the new value.
			GetCtrlVal( panel, control, &enabled );
		
      commandMessage[ 2 ] = SHM_CONTROL_BYTES_NUMBER; 
      
			// Write the new value to the appropriate network variable.
			if( control == PANEL_MOTOR_TOGGLE )
      {
        if( enabled == 1 )
          commandMessage[ 2 ] = SHM_COMMAND_ENABLE;
        else
          commandMessage[ 2 ] = SHM_COMMAND_DISABLE;
      }
      else if( control == PANEL_OFFSET_TOGGLE )
      {
        commandMessage[ 2 ] = SHM_COMMAND_CALIBRATE;
      }
      
			AsyncIPConnection_WriteMessage( infoClientID, commandMessage );
      
			break;
	}
  
	return 0;
}

int CVICALLBACK ChangeValueCallback( int panel, int control, int event, void* callbackData, int eventData1, int eventData2 )
{
	switch( event )
	{
		case EVENT_COMMIT:
			
			if( control == PANEL_STIFFNESS )
			{
        // Get the new value.
			  GetCtrlVal( panel, control, &maxStiffness );
      }
      
			break;
	}
	return 0;
}

int CVICALLBACK QuitCallback( int panel, int event, void *callbackData, int eventData1, int eventData2 )
{
  switch( event )
	{
		case EVENT_CLOSE:
			
      QuitUserInterface( 0 );
      
			break;
	}
	return 0;
}

