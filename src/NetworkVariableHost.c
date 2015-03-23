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
#include "cvirte_connection.h"
#include "NetworkVariable.h"
#include "common.h"

/* Global variables */
static int				panel;
static CNVSubscriber	gWaveSubscriber;
static CNVWriter		gAmplitudeWriter, gFrequencyWriter, gStopWriter;

/* Function prototypes */
static void ConnectToNetworkVariables(void);
int CVICALLBACK AmplitudeFrequencyCallback(int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2);
void CVICALLBACK DataCallback(void * handle, CNVData data, void * callbackData);

/* Program entry-point */
int main(int argc, char *argv[])
{
	CNVData stopData;
	
	if (InitCVIRTE(0, argv, 0) == 0)
		return -1;
  
  int clientID = async_connection_open( "169.254.110.158", "50000", TCP );
  
  async_connection_write_message( clientID, "Teste" );
	
	if ((panel = LoadPanel(0, "NetworkVariable.uir", PANEL)) < 0)
		return -1;

	ConnectToNetworkVariables();
	
	DisplayPanel(panel);
	RunUserInterface();
	
	// The user chose to quit. Signal real-time program to stop as well.
	CNVCreateScalarDataValue(&stopData, CNVBool, (char)1);
	CNVWrite(gStopWriter, stopData, CNVDoNotWait);
	
  async_connection_close( clientID );
  
	// Cleanup
	CNVDisposeData(stopData);
	CNVDispose(gWaveSubscriber);
	CNVDispose(gAmplitudeWriter);
	CNVDispose(gFrequencyWriter);
	CNVDispose(gStopWriter);
	CNVFinish();
	DiscardPanel(panel);
	
	return 0;
}

static void ConnectToNetworkVariables(void)
{
	char		address[256], path[512];
	
	// Get address of real-time target.
	PromptPopup("Prompt", "Enter Real-Time Target Name/IP Address:", 
		address, sizeof(address) - 1);
	
	// Connect to network variables.
	sprintf(path, "\\\\%s\\" PROCESS "\\%s", address, AMPLITUDE_VARIABLE);
	CNVCreateWriter(path, 0, 0, 10000, 0, &gAmplitudeWriter);
	
	sprintf(path, "\\\\%s\\" PROCESS "\\%s", address, FREQUENCY_VARIABLE);
	CNVCreateWriter(path, 0, 0, 10000, 0, &gFrequencyWriter);

	sprintf(path, "\\\\%s\\" PROCESS "\\%s", address, STOP_VARIABLE);
	CNVCreateWriter(path, 0, 0, 10000, 0, &gStopWriter);

	sprintf(path, "\\\\%s\\" PROCESS "\\%s", address, WAVE_VARIABLE);
	CNVCreateSubscriber(path, DataCallback, 0, 0, 10000, 0, &gWaveSubscriber);
	
	// Manually call the control callbacks to write the initial values for
	// amplitude and frequency to the network variables.
	AmplitudeFrequencyCallback(panel, PANEL_AMPLITUDE, EVENT_COMMIT, 0, 0, 0);
	AmplitudeFrequencyCallback(panel, PANEL_FREQUENCY, EVENT_COMMIT, 0, 0, 0);
}

int CVICALLBACK AmplitudeFrequencyCallback(int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	CNVData	data;
	double	value;
	
	switch (event)
	{
		case EVENT_COMMIT:
			// Get the new value.
			GetCtrlVal(panel, control, &value);
			CNVCreateScalarDataValue(&data, CNVDouble, value);
		
			// Write the new value to the appropriate network variable.
			if (control == PANEL_AMPLITUDE)
				CNVWrite(gAmplitudeWriter, data, CNVDoNotWait);
			else if (control == PANEL_FREQUENCY)
				CNVWrite(gFrequencyWriter, data, CNVDoNotWait);
		
			CNVDisposeData(data);
			break;
	}
	return 0;
}

void CVICALLBACK DataCallback(void * handle, CNVData data, void * callbackData)
{
	double wave[NUM_POINTS];
	
	// Get the published data.
	CNVGetArrayDataValue(data, CNVDouble, wave, NUM_POINTS);
	
	// Plot the data to the graph.
	DeleteGraphPlot(panel, PANEL_GRAPH, -1, VAL_DELAYED_DRAW);
	PlotY(panel, PANEL_GRAPH, wave, NUM_POINTS, VAL_DOUBLE, VAL_THIN_LINE, 
		VAL_NO_POINT, VAL_SOLID, 1, VAL_YELLOW);
		
	CNVDisposeData(data);
}

int CVICALLBACK QuitCallback(int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
			QuitUserInterface (0);
			break;
	}
	return 0;
}

