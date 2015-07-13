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

#include "async_debug.h"
#include "cvirte_ip_connection.h"
#include "NetworkVariable.h"
#include "common.h"

#include "data_logging.h"

/* Global variables */
static int panel;
static CNVSubscriber gWaveSubscriber;
static CNVWriter gMaxToggleWriter, gMinToggleWriter;

/* Function prototypes */
static void* UpdateData( void* );
static void ConnectToNetworkVariables( void );
int CVICALLBACK GainCallback( int, int, int, void*, int, int );
void CVICALLBACK DataCallback( void*, CNVData, void* );

int axisLogID;

enum { ROBOT_POSITION, ROBOT_VELOCITY, ROBOT_SETPOINT, ROBOT_ERROR, ROBOT_STIFFNESS, ROBOT_TORQUE, MAX_STIFFNESS, PATIENT_STIFFNESS, PATIENT_TORQUE, EMG_AGONIST, EMG_ANTAGONIST, DISPLAY_VALUES_NUMBER };

enum { AGONIST, ANTAGONIST };
int muscleGroup = AGONIST;

int axisID = 0;

int motorEnabled = 0;
double maxStiffness = 0.0;

Thread_Handle dataConnectionThreadID;
bool isDataUpdateRunning = false;

char address[ 256 ] = "169.254.110.158";

int infoClientID, dataClientID;

const double TOTAL_CURVE_INTERVAL = 2.22;
const double CONTROL_SAMPLING_INTERVAL = 0.005;
const double NORMALIZED_SAMPLING_INTERVAL = CONTROL_SAMPLING_INTERVAL / TOTAL_CURVE_INTERVAL;

static double clientDispatchTime;

static double referenceValues[ NUM_POINTS ];
size_t setpointIndex = 0;

/* Program entry-point */
int main( int argc, char *argv[] )
{
	if( InitCVIRTE( 0, argv, 0 ) == 0 )
		return -1;
  
  infoClientID = AsyncIPConnection_Open( address, "50000", TCP );
  char* infoMessage = NULL;
  while( infoMessage == NULL )
    infoMessage = AsyncIPConnection_ReadMessage( infoClientID );
  
  fprintf( stderr, "received info message: %s\n", infoMessage );
  
  char resetMessage[ IP_CONNECTION_MSG_LEN ];
  strcpy( resetMessage, "0 0 1 0.000 0.000" );
  AsyncIPConnection_WriteMessage( infoClientID, resetMessage );
  
  infoMessage = NULL;
  while( infoMessage == NULL )
    infoMessage = AsyncIPConnection_ReadMessage( infoClientID );
  
  fprintf( stderr, "received info message: %s\n", infoMessage );
  
  dataConnectionThreadID = Thread_Start( UpdateData, NULL, THREAD_JOINABLE );
	
  axisLogID = DataLogging_CreateLog( NULL, "axis", DISPLAY_VALUES_NUMBER );
  
	if( (panel = LoadPanel( 0, "NetworkVariable.uir", PANEL )) < 0 )
		return -1;

	ConnectToNetworkVariables();
	
	DisplayPanel( panel );
	RunUserInterface();
	
  isDataUpdateRunning = false;
  Thread_WaitExit( dataConnectionThreadID, 5000 );
  
	AsyncIPConnection_WriteMessage( infoClientID, resetMessage );
  
  DataLogging_CloseLog( axisLogID );
  
  AsyncIPConnection_Close( infoClientID ); 
  
	// Cleanup
	CNVDispose( gWaveSubscriber );
	CNVDispose( gMaxToggleWriter );
	CNVDispose( gMinToggleWriter );
	CNVFinish();
  
	DiscardPanel( panel );
	
  CloseCVIRTE();
  
	return 0;
}

static void* UpdateData( void* callbackData )
{
  const size_t WAIT_SAMPLES = 10;
  const double SETPOINT_UPDATE_INTERVAL = WAIT_SAMPLES * CONTROL_SAMPLING_INTERVAL;
  
  char dataMessageOut[ IP_CONNECTION_MSG_LEN ];
  
  double serverDispatchTime = 0.0, clientReceiveTime = 0.0, serverReceiveTime = 0.0;
  double latency = 0.0;
  
  double initialTime = Timing_GetExecTimeSeconds();
  
  double elapsedTime = 0.0, absoluteTime = initialTime; 
  
  double deltaTime = 0.0;
  
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
      char* timeData = strtok( messageIn, "|" );
    
      clientDispatchTime = strtod( timeData, &timeData );
      serverReceiveTime = strtod( timeData, &timeData );
      serverDispatchTime = strtod( timeData, &timeData );
      clientReceiveTime = Timing_GetExecTimeSeconds();
      
      char* axesData = strtok( NULL, "|" );
      
      for( char* axisData = strtok( axesData, ":" ); axisData != NULL; axisData = strtok( NULL, ":" ) )
      {
        if( (unsigned int) strtoul( axisData, &axisData, 0 ) == 0 )
        {
          latency = ( ( clientReceiveTime - clientDispatchTime ) - ( serverDispatchTime - serverReceiveTime ) ) / 2;
        }
      }
    }
    
    if( deltaTime >= SETPOINT_UPDATE_INTERVAL )
    {
      double setpointValue = elapsedTime / TOTAL_CURVE_INTERVAL;
    
      double setpointDerivative = 1.0 / TOTAL_CURVE_INTERVAL;
      
      for( size_t i = 0; i < WAIT_SAMPLES; i++ )
        referenceValues[ ( setpointIndex + i ) % NUM_POINTS ] = fmod( setpointValue, 1.0 );
      setpointIndex += WAIT_SAMPLES;
    
      sprintf( dataMessageOut, "%g %g %g|0 %g %g %g", serverDispatchTime, clientReceiveTime, absoluteTime, setpointValue, setpointDerivative, SETPOINT_UPDATE_INTERVAL );
    
      deltaTime = 0.0;
      
      AsyncIPConnection_WriteMessage( dataClientID, dataMessageOut );
    }
  }
  
  AsyncIPConnection_Close( dataClientID );
  
  Thread_Exit( 0 );
  return NULL;
}

static void ConnectToNetworkVariables( void )
{
	char path[ 512 ];
	
	// Get address of real-time target.
	//PromptPopup("Prompt", "Enter Real-Time Target Name/IP Address:", address, sizeof(address) - 1);
	
	// Connect to network variables.
	sprintf( path, "\\\\%s\\" PROCESS "\\%s", address, MAX_TOGGLE_VARIABLE );
	CNVCreateWriter( path, 0, 0, 10000, 0, &gMaxToggleWriter );
	
	sprintf( path, "\\\\%s\\" PROCESS "\\%s", address, MIN_TOGGLE_VARIABLE );
	CNVCreateWriter( path, 0, 0, 10000, 0, &gMinToggleWriter );

	sprintf( path, "\\\\%s\\" PROCESS "\\%s", address, WAVE_VARIABLE );
	CNVCreateSubscriber( path, DataCallback, 0, 0, 10000, 0, &gWaveSubscriber );
	
	// Manually call the control callbacks to write the initial values for
	// amplitude and frequency to the network variables.
  SetCtrlVal( panel, PANEL_MAX_TOGGLE, 0 );
  SetCtrlVal( panel, PANEL_MIN_TOGGLE, 0 );
  SetCtrlVal( panel, PANEL_MOTOR_TOGGLE, 0 );
	ChangeStateCallback( panel, PANEL_MAX_TOGGLE, EVENT_COMMIT, 0, 0, 0 );
	ChangeStateCallback( panel, PANEL_MIN_TOGGLE, EVENT_COMMIT, 0, 0, 0 );
  ChangeStateCallback( panel, PANEL_MOTOR_TOGGLE, EVENT_COMMIT, 0, 0, 0 );
  
	ChangeValueCallback( panel, PANEL_STIFFNESS, EVENT_COMMIT, 0, 0, 0 );
}

int CVICALLBACK ChangeMuscleGroupCallback( int panel, int control, int event, void* callbackData, int eventData1, int eventData2 )
{
  switch( event )
	{
		case EVENT_COMMIT:
			// Get the new value.
			GetCtrlVal( panel, control, &muscleGroup );
			break;
	}
	return 0;
}

int CVICALLBACK ChangeStateCallback( int panel, int control, int event, void* callbackData, int eventData1, int eventData2 )
{
	CNVData	data;
  
  int enabled = 0;
  int messageCode = 0;
  
	switch( event )
	{
		case EVENT_COMMIT:
			// Get the new value.
			GetCtrlVal( panel, control, &enabled );
      
      messageCode |= ( axisID & 0x000000ff );
      messageCode |= ( ( enabled * 0x100 ) & 0x0000ff00 );
      messageCode |= ( ( muscleGroup * 0x10000 ) & 0x00ff0000 );
      
			CNVCreateScalarDataValue( &data, CNVInt32, messageCode );
		
			// Write the new value to the appropriate network variable.
			if( control == PANEL_MAX_TOGGLE )
				CNVWrite( gMaxToggleWriter, data, CNVDoNotWait );
			else if( control == PANEL_MIN_TOGGLE )
				CNVWrite( gMinToggleWriter, data, CNVDoNotWait );
      else if( control == PANEL_MOTOR_TOGGLE )
      {
        char enableMessage[ IP_CONNECTION_MSG_LEN ];
        motorEnabled = enabled;
        sprintf( enableMessage, "0 %d 0 %.3f 0.000", motorEnabled, maxStiffness );
        AsyncIPConnection_WriteMessage( infoClientID, enableMessage );
      }
		
			CNVDisposeData( data );
      
			break;
	}
	return 0;
}

int  CVICALLBACK ChangeValueCallback( int panel, int control, int event, void* callbackData, int eventData1, int eventData2 )
{
	switch( event )
	{
		case EVENT_COMMIT:
			// Get the new value.
			GetCtrlVal( panel, control, &maxStiffness );

			if( control == PANEL_STIFFNESS )
			{
        char impedanceMessage[ IP_CONNECTION_MSG_LEN ];
        sprintf( impedanceMessage, "0 %d 0 %.3f 0.000", motorEnabled, maxStiffness );
        AsyncIPConnection_WriteMessage( infoClientID, impedanceMessage );
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

void CVICALLBACK DataCallback( void* handle, CNVData data, void* callbackData )
{
  static double execTime, elapsedTime;
  static double responseTime;
  
  elapsedTime = Timing_GetExecTimeSeconds()  - execTime;
  execTime = Timing_GetExecTimeSeconds();
  
  responseTime = execTime - clientDispatchTime;
  
  size_t referencePointsNumber = (size_t) ( elapsedTime / CONTROL_SAMPLING_INTERVAL );
  if( referencePointsNumber > NUM_POINTS ) referencePointsNumber = NUM_POINTS;
  else if( referencePointsNumber <= 0 ) referencePointsNumber = 1;
  
  //size_t pointLength = NUM_POINTS / referencePointsNumber;
  
  size_t setpointOffset = 0;//(size_t) ( responseTime / CONTROL_SAMPLING_INTERVAL );
  
  //fprintf( stderr, "offset: 2 * %f / %.3f = %u\r", responseTime, CONTROL_SAMPLING_INTERVAL, setpointOffset );
  
  //size_t setpointStart = ( setpointIndex > NUM_POINTS + setpointOffset ) ? ( setpointIndex - setpointOffset - NUM_POINTS + 1 ) : 0;
  
	static double dataArray[ DISPLAY_VALUES_NUMBER * NUM_POINTS ];
  static double positionValues[ NUM_POINTS ], velocityValues[ NUM_POINTS ], setpointValues[ NUM_POINTS ], errorValues[ NUM_POINTS ];
  static double robotTorqueValues[ NUM_POINTS ], patientTorqueValues[ NUM_POINTS ];
  static double robotStiffnessValues[ NUM_POINTS ], patientStiffnessValues[ NUM_POINTS ], maxStiffnessValues[ NUM_POINTS ];
  static double emgAgonistValues[ NUM_POINTS ], emgAntagonistValues[ NUM_POINTS ];
  
  static double originalValues[ NUM_POINTS ];
	
	// Get the published data.
	CNVGetArrayDataValue( data, CNVDouble, dataArray, DISPLAY_VALUES_NUMBER * NUM_POINTS );
  
  for( size_t pointIndex = 0; pointIndex < NUM_POINTS; pointIndex++ )
  {
    positionValues[ pointIndex ] = -dataArray[ pointIndex * DISPLAY_VALUES_NUMBER + ROBOT_POSITION ];
    velocityValues[ pointIndex ] = -dataArray[ pointIndex * DISPLAY_VALUES_NUMBER + ROBOT_VELOCITY ];
    setpointValues[ pointIndex ] = -dataArray[ pointIndex * DISPLAY_VALUES_NUMBER + ROBOT_SETPOINT ];
    errorValues[ pointIndex ] = dataArray[ pointIndex * DISPLAY_VALUES_NUMBER + ROBOT_ERROR ];
    robotStiffnessValues[ pointIndex ] = dataArray[ pointIndex * DISPLAY_VALUES_NUMBER + ROBOT_STIFFNESS ];
    maxStiffnessValues[ pointIndex ] = dataArray[ pointIndex * DISPLAY_VALUES_NUMBER + MAX_STIFFNESS ];
    patientStiffnessValues[ pointIndex ] = dataArray[ pointIndex * DISPLAY_VALUES_NUMBER + PATIENT_STIFFNESS ];
    robotTorqueValues[ pointIndex ] = dataArray[ pointIndex * DISPLAY_VALUES_NUMBER + ROBOT_TORQUE ];
    patientTorqueValues[ pointIndex ] = dataArray[ pointIndex * DISPLAY_VALUES_NUMBER + PATIENT_TORQUE ];
    emgAgonistValues[ pointIndex ] = dataArray[ pointIndex * DISPLAY_VALUES_NUMBER + EMG_AGONIST ];
    emgAntagonistValues[ pointIndex ] = dataArray[ pointIndex * DISPLAY_VALUES_NUMBER + EMG_ANTAGONIST ];
    
    //if( pointIndex >= setpointOffset )
    //  originalValues[ pointIndex ] = referenceValues[ pointIndex - setpointOffset ];
    //dataArray[ pointIndex * DISPLAY_VALUES_NUMBER + ROBOT_VELOCITY ] = -referenceValues[ pointIndex ];
  }
  
  //if( emgAgonistValues[ NUM_POINTS - 1 ] > 0.0 && emgAntagonistValues[ NUM_POINTS - 1 ] > 0.0 )
  DataLogging_SaveData( axisLogID, dataArray, DISPLAY_VALUES_NUMBER * NUM_POINTS );
  
	// Plot the data to the graph.
	DeleteGraphPlot( panel, PANEL_GRAPH_1, -1, VAL_DELAYED_DRAW );
  //PlotY( panel, PANEL_GRAPH_1, originalValues, NUM_POINTS - setpointOffset, VAL_DOUBLE, VAL_THIN_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_CYAN );
  //PlotY( panel, PANEL_GRAPH_1, setpointValues + setpointOffset, NUM_POINTS - setpointOffset, VAL_DOUBLE, VAL_THIN_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_CYAN );
	PlotY( panel, PANEL_GRAPH_1, setpointValues, NUM_POINTS, VAL_DOUBLE, VAL_THIN_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_RED );
  PlotY( panel, PANEL_GRAPH_1, errorValues, NUM_POINTS, VAL_DOUBLE, VAL_FAT_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_BLUE );
  PlotY( panel, PANEL_GRAPH_1, positionValues, NUM_POINTS, VAL_DOUBLE, VAL_FAT_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_YELLOW );
  PlotY( panel, PANEL_GRAPH_1, velocityValues, NUM_POINTS, VAL_DOUBLE, VAL_THIN_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_GREEN );
  PlotY( panel, PANEL_GRAPH_1, emgAgonistValues, NUM_POINTS, VAL_DOUBLE, VAL_THIN_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_DK_MAGENTA );
  PlotY( panel, PANEL_GRAPH_1, emgAntagonistValues, NUM_POINTS, VAL_DOUBLE, VAL_THIN_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_MAGENTA );
  
  DeleteGraphPlot( panel, PANEL_GRAPH_2, -1, VAL_DELAYED_DRAW );
  PlotY( panel, PANEL_GRAPH_2, maxStiffnessValues, NUM_POINTS, VAL_DOUBLE, VAL_THIN_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_MAGENTA );
  PlotY( panel, PANEL_GRAPH_2, robotTorqueValues, NUM_POINTS, VAL_DOUBLE, VAL_THIN_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_CYAN );
  PlotY( panel, PANEL_GRAPH_2, patientTorqueValues, NUM_POINTS, VAL_DOUBLE, VAL_THIN_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_GREEN );
  PlotY( panel, PANEL_GRAPH_2, robotStiffnessValues, NUM_POINTS, VAL_DOUBLE, VAL_FAT_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_RED );
  PlotY( panel, PANEL_GRAPH_2, patientStiffnessValues, NUM_POINTS, VAL_DOUBLE, VAL_FAT_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_YELLOW );
		
	CNVDisposeData( data );
}

