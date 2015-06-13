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
static CNVWriter gMaxToggleWriter, gMinToggleWriter, gMotorToggleWriter, gMaxStiffnessWriter;

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

Thread_Handle dataConnectionThreadID;
bool isDataUpdateRunning = false;

char address[ 256 ] = "169.254.110.158";

int dataClientID;

const int FILE_SETPOINTS_NUMBER = 444;
static double fileSetpointsList[ FILE_SETPOINTS_NUMBER ];
static double fileSetpointsDerivativeList[ FILE_SETPOINTS_NUMBER ];
static double fileSetpointsDerivative2List[ FILE_SETPOINTS_NUMBER ];
static size_t setpointIndex;

const double CONTROL_SAMPLING_INTERVAL = 0.005;           // Sampling interval

static double clientDispatchTime;

/* Program entry-point */
int main( int argc, char *argv[] )
{
	CNVData enableData;
	
	if( InitCVIRTE( 0, argv, 0 ) == 0 )
		return -1;
  
  FILE* setpointsFile = fopen( "./setpoints.dat", "r" );
  
  for( size_t i = 0; i < FILE_SETPOINTS_NUMBER; i++ )
    fscanf( setpointsFile, "%lf\n", &(fileSetpointsList[ i ]) );
            
  fclose( setpointsFile );
  
  for( size_t i = 0; i < FILE_SETPOINTS_NUMBER; i++ )
    fileSetpointsDerivativeList[ i ] = ( fileSetpointsList[ ( i + 1 ) % FILE_SETPOINTS_NUMBER ] - fileSetpointsList[ i ] ) / CONTROL_SAMPLING_INTERVAL;
    
  for( size_t i = 0; i < FILE_SETPOINTS_NUMBER; i++ )
    fileSetpointsDerivative2List[ i ] = ( fileSetpointsDerivativeList[ ( i + 1 ) % FILE_SETPOINTS_NUMBER ] - fileSetpointsDerivativeList[ i ] ) / CONTROL_SAMPLING_INTERVAL;
  
  int infoClientID = AsyncIPConnection_Open( address, "50000", TCP );
  char* infoMessage = NULL;
  while( infoMessage == NULL )
    infoMessage = AsyncIPConnection_ReadMessage( infoClientID );
  
  fprintf( stderr, "received info message: %s", infoMessage );
  
  dataConnectionThreadID = Thread_Start( UpdateData, NULL, THREAD_JOINABLE );
	
  axisLogID = DataLogging_CreateLog( NULL, "axis", DISPLAY_VALUES_NUMBER );
  
	if( (panel = LoadPanel( 0, "NetworkVariable.uir", PANEL )) < 0 )
		return -1;

	ConnectToNetworkVariables();
	
	DisplayPanel( panel );
	RunUserInterface();
	
  isDataUpdateRunning = false;
  Thread_WaitExit( dataConnectionThreadID, 5000 );
  
	CNVCreateScalarDataValue( &enableData, CNVBool, (char) 0 );
	CNVWrite( gMotorToggleWriter, enableData, CNVDoNotWait );
  
  DataLogging_CloseLog( axisLogID );
  
	// Cleanup
	CNVDisposeData( enableData );
	CNVDispose( gWaveSubscriber );
	CNVDispose( gMaxToggleWriter );
	CNVDispose( gMinToggleWriter );
	CNVDispose( gMotorToggleWriter );
  CNVDispose( gMaxStiffnessWriter );
	CNVFinish();
  
	DiscardPanel( panel );
	
  CloseCVIRTE();
  
	return 0;
}

static void* UpdateData( void* callbackData )
{
  const double SETPOINT_UPDATE_INTERVAL = 10 * CONTROL_SAMPLING_INTERVAL;
  
  char dataMessageOut[ IP_CONNECTION_MSG_LEN ];
  
  double serverDispatchTime = 0.0, clientReceiveTime = 0.0, serverReceiveTime = 0.0;
  double latency = 0.0;
  
  double initialTime = ( (double) Timing_GetExecTimeMilliseconds() ) / 1000.0;
  
  double elapsedTime = 0.0, absoluteTime = initialTime; 
  
  double deltaTime = 0.0;
  
  int dataClientID = AsyncIPConnection_Open( address, "50001", UDP );
  
  if( dataClientID != -1 ) isDataUpdateRunning = true;
  
  while( isDataUpdateRunning )
  {
    deltaTime += ( ( (double) Timing_GetExecTimeMilliseconds() ) / 1000.0 - absoluteTime );
    absoluteTime = ( (double) Timing_GetExecTimeMilliseconds() ) / 1000.0;
    elapsedTime = absoluteTime - initialTime;
    
    char* messageIn = AsyncIPConnection_ReadMessage( dataClientID );
    if( messageIn != NULL )
    {
      for( char* axisData = strtok( messageIn, ":" ); axisData != NULL; axisData = strtok( NULL, ":" ) )
      {
        if( (unsigned int) strtoul( axisData, &axisData, 0 ) == 0 )
        {
          clientDispatchTime = strtod( axisData, &axisData );
          serverReceiveTime = strtod( axisData, &axisData );
          serverDispatchTime = strtod( axisData, &axisData );
          clientReceiveTime = absoluteTime;
          
          latency = ( ( clientReceiveTime - clientDispatchTime ) - ( serverDispatchTime - serverReceiveTime ) ) / 2;
        }
      }
    }
    
    setpointIndex = (size_t) ( elapsedTime / CONTROL_SAMPLING_INTERVAL );
    
    //if( fileSetpointsDerivativeList[ ( setpointIndex - 1 ) % FILE_SETPOINTS_NUMBER ] * fileSetpointsDerivativeList[ ( setpointIndex + 1 ) % FILE_SETPOINTS_NUMBER ] < 0.0
    //    || fileSetpointsDerivative2List[ ( setpointIndex - 1 ) % FILE_SETPOINTS_NUMBER ] * fileSetpointsDerivative2List[ ( setpointIndex + 1 ) % FILE_SETPOINTS_NUMBER ] < 0.0 )
    if( deltaTime >= SETPOINT_UPDATE_INTERVAL )
    {
      size_t dataIndex = setpointIndex % FILE_SETPOINTS_NUMBER;
    
      strcpy( dataMessageOut, "0" );
      sprintf( &dataMessageOut[ strlen( dataMessageOut ) ], " %g %g %g %g %g %g", serverDispatchTime, clientReceiveTime, absoluteTime,
                                                                                  -fileSetpointsList[ dataIndex ], -fileSetpointsDerivativeList[ dataIndex ], deltaTime );
    
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

	sprintf( path, "\\\\%s\\" PROCESS "\\%s", address, ENABLE_VARIABLE );
	CNVCreateWriter( path, 0, 0, 10000, 0, &gMotorToggleWriter );
  
  sprintf( path, "\\\\%s\\" PROCESS "\\%s", address, STIFFNESS_VARIABLE );
	CNVCreateWriter( path, 0, 0, 10000, 0, &gMaxStiffnessWriter );

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
	int enabled;
  
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
        CNVWrite( gMotorToggleWriter, data, CNVDoNotWait );
		
			CNVDisposeData( data );
      
			break;
	}
	return 0;
}

int  CVICALLBACK ChangeValueCallback( int panel, int control, int event, void* callbackData, int eventData1, int eventData2 )
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
				CNVWrite( gMaxStiffnessWriter, data, CNVDoNotWait );
		
			CNVDisposeData( data );
      
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
  
  elapsedTime = ( (double) Timing_GetExecTimeMilliseconds() ) / 1000.0 - execTime;
  execTime = ( (double) Timing_GetExecTimeMilliseconds() ) / 1000.0;
  
  responseTime = execTime - clientDispatchTime;
  
  size_t referencePointsNumber = (size_t) ( elapsedTime / CONTROL_SAMPLING_INTERVAL );
  if( referencePointsNumber > NUM_POINTS ) referencePointsNumber = NUM_POINTS;
  else if( referencePointsNumber <= 0 ) referencePointsNumber = 1;
  
  size_t pointLength = NUM_POINTS / referencePointsNumber;
  
  size_t setpointOffset = (size_t) ( 2 * responseTime / CONTROL_SAMPLING_INTERVAL );
  
  //fprintf( stderr, "offset: 2 * %f / %.3f = %u\r", responseTime, CONTROL_SAMPLING_INTERVAL, setpointOffset );
  
  size_t setpointStart = ( setpointIndex > NUM_POINTS + setpointOffset ) ? ( setpointIndex - setpointOffset - NUM_POINTS + 1 ) : 0;
  
  static double referenceValues[ NUM_POINTS ];
  
	static double dataArray[ DISPLAY_VALUES_NUMBER * NUM_POINTS ];
  static double positionValues[ NUM_POINTS ], velocityValues[ NUM_POINTS ], setpointValues[ NUM_POINTS ], errorValues[ NUM_POINTS ];
  static double robotTorqueValues[ NUM_POINTS ], patientTorqueValues[ NUM_POINTS ];
  static double robotStiffnessValues[ NUM_POINTS ], patientStiffnessValues[ NUM_POINTS ], maxStiffnessValues[ NUM_POINTS ];
  static double emgAgonistValues[ NUM_POINTS ], emgAntagonistValues[ NUM_POINTS ]; 
	
	// Get the published data.
	CNVGetArrayDataValue( data, CNVDouble, dataArray, DISPLAY_VALUES_NUMBER * NUM_POINTS );
  
  for( size_t pointIndex = 0; pointIndex < NUM_POINTS; pointIndex++ )
  {
    positionValues[ pointIndex ] = -dataArray[ pointIndex * DISPLAY_VALUES_NUMBER + ROBOT_POSITION ];
    velocityValues[ pointIndex ] = dataArray[ pointIndex * DISPLAY_VALUES_NUMBER + ROBOT_VELOCITY ];
    setpointValues[ pointIndex ] = -dataArray[ pointIndex * DISPLAY_VALUES_NUMBER + ROBOT_SETPOINT ];
    errorValues[ pointIndex ] = dataArray[ pointIndex * DISPLAY_VALUES_NUMBER + ROBOT_ERROR ];
    robotStiffnessValues[ pointIndex ] = dataArray[ pointIndex * DISPLAY_VALUES_NUMBER + ROBOT_STIFFNESS ];
    maxStiffnessValues[ pointIndex ] = dataArray[ pointIndex * DISPLAY_VALUES_NUMBER + MAX_STIFFNESS ];
    patientStiffnessValues[ pointIndex ] = dataArray[ pointIndex * DISPLAY_VALUES_NUMBER + PATIENT_STIFFNESS ];
    robotTorqueValues[ pointIndex ] = dataArray[ pointIndex * DISPLAY_VALUES_NUMBER + ROBOT_TORQUE ];
    patientTorqueValues[ pointIndex ] = dataArray[ pointIndex * DISPLAY_VALUES_NUMBER + PATIENT_TORQUE ];
    emgAgonistValues[ pointIndex ] = dataArray[ pointIndex * DISPLAY_VALUES_NUMBER + EMG_AGONIST ];
    emgAntagonistValues[ pointIndex ] = dataArray[ pointIndex * DISPLAY_VALUES_NUMBER + EMG_ANTAGONIST ];
    
    referenceValues[ pointIndex ] = fileSetpointsList[ ( setpointStart + pointIndex / pointLength ) % FILE_SETPOINTS_NUMBER ];
  }
  
  //if( emgAgonistValues[ NUM_POINTS - 1 ] > 0.0 && emgAntagonistValues[ NUM_POINTS - 1 ] > 0.0 )
  DataLogging_SaveData( axisLogID, dataArray, DISPLAY_VALUES_NUMBER * NUM_POINTS );
  
	// Plot the data to the graph.
	DeleteGraphPlot( panel, PANEL_GRAPH_1, -1, VAL_DELAYED_DRAW );
  PlotY( panel, PANEL_GRAPH_1, referenceValues, NUM_POINTS, VAL_DOUBLE, VAL_THIN_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_CYAN );
	PlotY( panel, PANEL_GRAPH_1, setpointValues, NUM_POINTS, VAL_DOUBLE, VAL_THIN_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_RED );
  PlotY( panel, PANEL_GRAPH_1, errorValues, NUM_POINTS, VAL_DOUBLE, VAL_FAT_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_BLUE );
  PlotY( panel, PANEL_GRAPH_1, positionValues, NUM_POINTS, VAL_DOUBLE, VAL_FAT_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_YELLOW );
  PlotY( panel, PANEL_GRAPH_1, emgAgonistValues, NUM_POINTS, VAL_DOUBLE, VAL_THIN_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_GREEN );
  PlotY( panel, PANEL_GRAPH_1, emgAntagonistValues, NUM_POINTS, VAL_DOUBLE, VAL_THIN_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_MAGENTA );
  
  DeleteGraphPlot( panel, PANEL_GRAPH_2, -1, VAL_DELAYED_DRAW );
  PlotY( panel, PANEL_GRAPH_2, maxStiffnessValues, NUM_POINTS, VAL_DOUBLE, VAL_THIN_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_MAGENTA );
  PlotY( panel, PANEL_GRAPH_2, robotTorqueValues, NUM_POINTS, VAL_DOUBLE, VAL_THIN_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_CYAN );
  PlotY( panel, PANEL_GRAPH_2, patientTorqueValues, NUM_POINTS, VAL_DOUBLE, VAL_THIN_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_GREEN );
  PlotY( panel, PANEL_GRAPH_2, robotStiffnessValues, NUM_POINTS, VAL_DOUBLE, VAL_FAT_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_RED );
  PlotY( panel, PANEL_GRAPH_2, patientStiffnessValues, NUM_POINTS, VAL_DOUBLE, VAL_FAT_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_YELLOW );
		
	CNVDisposeData( data );
}

