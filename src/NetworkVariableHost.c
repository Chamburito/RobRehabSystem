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
#include <cvinetv.h>
#include <cvirte.h>		
#include <userint.h>

#include "async_debug.h"
#include "cvirte_ip_connection.h"
#include "NetworkVariable.h"
#include "common.h"

#include "data_logging.h"

/* Global variables */
static int panel;
static CNVSubscriber gWaveSubscriber;
static CNVWriter gStiffnessWriter, gDampingWriter, gEnableWriter;

/* Function prototypes */
static void ConnectToNetworkVariables( void );
int CVICALLBACK GainCallback( int, int, int, void*, int, int );
void CVICALLBACK DataCallback( void*, CNVData, void* );

int axisLogID;
enum { POSITION, VELOCITY, CURRENT, TENSION, POSITION_SETPOINT, VELOCITY_SETPOINT, PROPORTIONAL_GAIN, DERIVATIVE_GAIN, DISPLAY_N_VALUES };

/* Program entry-point */
int main( int argc, char *argv[] )
{
	CNVData enableData;
	
	if( InitCVIRTE( 0, argv, 0 ) == 0 )
		return -1;
  
  //int clientID = AsyncIPConnection_Open( "169.254.110.158", "50000", TCP );
  
  //AsyncIPConnection_WriteMessage( clientID, "Teste" );
	
  axisLogID = DataLogging_CreateLog( NULL, "axis", DISPLAY_N_VALUES );
  
	if( (panel = LoadPanel( 0, "NetworkVariable.uir", PANEL )) < 0 )
		return -1;

	ConnectToNetworkVariables();
  
  CNVCreateScalarDataValue( &enableData, CNVBool, (char) 1 );
	CNVWrite( gEnableWriter, enableData, CNVDoNotWait );
	
	DisplayPanel( panel );
	RunUserInterface();
	
	CNVCreateScalarDataValue( &enableData, CNVBool, (char) 0 );
	CNVWrite( gEnableWriter, enableData, CNVDoNotWait );
	
  //AsyncIPConnection_Close( clientID );
  
  DataLogging_CloseLog( axisLogID );
  
	// Cleanup
	CNVDisposeData( enableData );
	CNVDispose( gWaveSubscriber );
	CNVDispose( gStiffnessWriter );
	CNVDispose( gDampingWriter );
	CNVDispose( gEnableWriter );
	CNVFinish();
  
	DiscardPanel( panel );
	
	return 0;
}

static void ConnectToNetworkVariables( void )
{
	char address[ 256 ] = "169.254.110.158" , path[ 512 ];
	
	// Get address of real-time target.
	//PromptPopup("Prompt", "Enter Real-Time Target Name/IP Address:", address, sizeof(address) - 1);
	
	// Connect to network variables.
	sprintf( path, "\\\\%s\\" PROCESS "\\%s", address, STIFFNESS_VARIABLE );
	CNVCreateWriter( path, 0, 0, 10000, 0, &gStiffnessWriter );
	
	sprintf( path, "\\\\%s\\" PROCESS "\\%s", address, DAMPING_VARIABLE );
	CNVCreateWriter( path, 0, 0, 10000, 0, &gDampingWriter );

	sprintf( path, "\\\\%s\\" PROCESS "\\%s", address, ENABLE_VARIABLE );
	CNVCreateWriter( path, 0, 0, 10000, 0, &gEnableWriter );

	sprintf( path, "\\\\%s\\" PROCESS "\\%s", address, WAVE_VARIABLE );
	CNVCreateSubscriber( path, DataCallback, 0, 0, 10000, 0, &gWaveSubscriber );
	
	// Manually call the control callbacks to write the initial values for
	// amplitude and frequency to the network variables.
	GainCallback( panel, PANEL_STIFFNESS, EVENT_COMMIT, 0, 0, 0 );
	GainCallback( panel, PANEL_DAMPING, EVENT_COMMIT, 0, 0, 0 );
}

int CVICALLBACK GainCallback( int panel, int control, int event, void* callbackData, int eventData1, int eventData2 )
{
	CNVData	data;
	double value;
	
	switch( event )
	{
		case EVENT_COMMIT:
			// Get the new value.
			GetCtrlVal( panel, control, &value );
			CNVCreateScalarDataValue( &data, CNVDouble, value );
		
			// Write the new value to the appropriate network variable.
			if( control == PANEL_STIFFNESS )
				CNVWrite( gStiffnessWriter, data, CNVDoNotWait );
			else if( control == PANEL_DAMPING )
				CNVWrite( gDampingWriter, data, CNVDoNotWait );
		
			CNVDisposeData(data);
			break;
	}
	return 0;
}

void CVICALLBACK DataCallback( void* handle, CNVData data, void* callbackData )
{
	static double dataArray[ DISPLAY_N_VALUES * NUM_POINTS ];
  static double positionValues[ NUM_POINTS ], velocityValues[ NUM_POINTS ], setpointValues[ NUM_POINTS ], emgValues[ NUM_POINTS ];
	
	// Get the published data.
	CNVGetArrayDataValue( data, CNVDouble, dataArray, DISPLAY_N_VALUES * NUM_POINTS );
  
  DataLogging_SaveData( axisLogID, dataArray, DISPLAY_N_VALUES * NUM_POINTS );
  
  for( size_t pointIndex = 0; pointIndex < NUM_POINTS; pointIndex++ )
  {
    positionValues[ pointIndex ] = dataArray[ pointIndex * DISPLAY_N_VALUES + POSITION ];
    velocityValues[ pointIndex ] = dataArray[ pointIndex * DISPLAY_N_VALUES + VELOCITY ];
    setpointValues[ pointIndex ] = dataArray[ pointIndex * DISPLAY_N_VALUES + POSITION_SETPOINT ];
    emgValues[ pointIndex ] = dataArray[ pointIndex * DISPLAY_N_VALUES + TENSION ];
  }
  
	// Plot the data to the graph.
	DeleteGraphPlot( panel, PANEL_GRAPH_POSITION, -1, VAL_DELAYED_DRAW );
	PlotY( panel, PANEL_GRAPH_POSITION, setpointValues, NUM_POINTS, VAL_DOUBLE, VAL_THIN_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_RED );
  PlotY( panel, PANEL_GRAPH_POSITION, positionValues, NUM_POINTS, VAL_DOUBLE, VAL_FAT_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_YELLOW );
  
  DeleteGraphPlot( panel, PANEL_GRAPH_EMG, -1, VAL_DELAYED_DRAW );
  PlotY( panel, PANEL_GRAPH_EMG, emgValues, NUM_POINTS, VAL_DOUBLE, VAL_FAT_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_BLUE );
		
	CNVDisposeData( data );
}

int CVICALLBACK QuitCallback( int panel, int control, int event, void *callbackData, int eventData1, int eventData2 )
{
	switch( event )
	{
		case EVENT_COMMIT:
			QuitUserInterface( 0 );
			break;
	}
	return 0;
}

