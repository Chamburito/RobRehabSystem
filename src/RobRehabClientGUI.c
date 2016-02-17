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
#include <ansi_c.h>
#include <stdbool.h>
//#include <cvinetv.h>
#include <cvirte.h>		
#include <userint.h>

#include "shm_control.h"
#include "shm_axis_control.h"
#include "shm_emg_control.h"

#include "RobRehabClientGUI.h"

#define NUM_POINTS			20

/* Global variables */
static int panel;

/* Function prototypes */
static void* UpdateData( void* );

SHMController axisMotorController;
SHMController jointEMGController;

Thread dataConnectionThreadID = INVALID_THREAD_HANDLE;
bool isDataUpdateRunning = false;

const double CONTROL_SAMPLING_INTERVAL = 0.005;

static double referenceValues[ NUM_POINTS ];
size_t setpointIndex = 0;

double maxStiffness;

void InitUserInterface( void );

/* Program entry-point */
int main( int argc, char *argv[] )
{
	if( InitCVIRTE( 0, argv, 0 ) == 0 )
		return -1;
  
	if( (panel = LoadPanel( 0, "RobRehabClientGUI.uir", PANEL )) < 0 )
		return -1;

	InitUserInterface();
	DisplayPanel( panel );
	RunUserInterface();
	
  isDataUpdateRunning = false;
  Threading.WaitExit( dataConnectionThreadID, 5000 );
  
  SHMControl.EndData( axisMotorController );
  SHMControl.EndData( jointEMGController );
  
	DiscardPanel( panel );
	
  CloseCVIRTE();
  
	return 0;
}

static void* UpdateData( void* callbackData )
{
  const size_t WAIT_SAMPLES = 2;
  const double SETPOINT_UPDATE_INTERVAL = WAIT_SAMPLES * CONTROL_SAMPLING_INTERVAL;
  
  float measuresList[ SHM_CONTROL_MAX_FLOATS_NUMBER ], setpointsList[ SHM_CONTROL_MAX_FLOATS_NUMBER ];
  
  double positionValues[ NUM_POINTS ], velocityValues[ NUM_POINTS ], torqueIDValues[ NUM_POINTS ], torqueEMGValues[ NUM_POINTS ];
  size_t axisMeasureIndex = 0, jointMeasureIndex = 0, setpointIndex = 0;
  
  double initialTime = Timing_GetExecTimeSeconds();
  double elapsedTime = 0.0, deltaTime = 0.0, setpointTime = 0.0, absoluteTime = initialTime; 
  
  isDataUpdateRunning = true;
  
  while( isDataUpdateRunning )
  {
    deltaTime = ( Timing_GetExecTimeSeconds() - absoluteTime );
    setpointTime += deltaTime;
    absoluteTime = Timing_GetExecTimeSeconds();
    elapsedTime = absoluteTime - initialTime;
    
    if( deltaTime > CONTROL_SAMPLING_INTERVAL )
    {
      axisMeasureIndex++;
      jointMeasureIndex++;
      
      //fprintf( stderr, "measure %u of %u\r", axisMeasureIndex, NUM_POINTS );
    }
    
    uint8_t axisDataMask = SHMControl.GetNumericValuesList( axisMotorController, measuresList, SHM_CONTROL_REMOVE );
    if( axisDataMask )
    {
      if( axisMeasureIndex >= NUM_POINTS )
      {
        DeleteGraphPlot( panel, PANEL_GRAPH_1, -1, VAL_DELAYED_DRAW );
        PlotY( panel, PANEL_GRAPH_1, positionValues, NUM_POINTS, VAL_DOUBLE, VAL_FAT_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_YELLOW );
        PlotY( panel, PANEL_GRAPH_1, velocityValues, NUM_POINTS, VAL_DOUBLE, VAL_THIN_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_GREEN );

        axisMeasureIndex = 0;
      }
      
      positionValues[ axisMeasureIndex ] = measuresList[ SHM_AXIS_POSITION ] * 6.28;
      velocityValues[ axisMeasureIndex ] = measuresList[ SHM_AXIS_VELOCITY ] * 6.28;
      
      SetCtrlVal( panel, PANEL_MEASURE_SLIDER, positionValues[ axisMeasureIndex ] * 180.0 / 3.14 );
    }
    
    uint8_t jointDataMask = SHMControl.GetNumericValuesList( jointEMGController, measuresList, SHM_CONTROL_REMOVE );
    //{
      if( jointMeasureIndex >= NUM_POINTS )
      {
        DeleteGraphPlot( panel, PANEL_GRAPH_2, -1, VAL_DELAYED_DRAW );
        PlotY( panel, PANEL_GRAPH_2, torqueIDValues, NUM_POINTS, VAL_DOUBLE, VAL_THIN_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_RED );
        PlotY( panel, PANEL_GRAPH_2, torqueEMGValues, NUM_POINTS, VAL_DOUBLE, VAL_THIN_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_GREEN );

        jointMeasureIndex = 0;
      }
      
      torqueEMGValues[ jointMeasureIndex ] = measuresList[ SHM_JOINT_EMG_TORQUE ];
      torqueIDValues[ jointMeasureIndex ] = measuresList[ SHM_JOINT_ID_TORQUE ];
    //}
    
    if( setpointTime >= SETPOINT_UPDATE_INTERVAL )
    {
      setpointsList[ SHM_AXIS_POSITION ] = sin( absoluteTime / 2.0 ) / 20.0 - 0.125;
      setpointsList[ SHM_AXIS_VELOCITY ] = cos( absoluteTime / 2.0 ) / 20.0;
      setpointsList[ SHM_AXIS_STIFFNESS ] = maxStiffness;
      setpointsList[ SHM_AXIS_TIME ] = SETPOINT_UPDATE_INTERVAL;
      
      SetCtrlVal( panel, PANEL_SETPOINT_SLIDER, setpointsList[ SHM_AXIS_POSITION ] * 360.0 );
      
      for( size_t i = 0; i < WAIT_SAMPLES; i++ )
        referenceValues[ ( setpointIndex + i ) % NUM_POINTS ] = setpointsList[ SHM_AXIS_POSITION ];
      setpointIndex += WAIT_SAMPLES;
    
      setpointTime = 0.0;
      
      SHMControl.SetNumericValuesList( axisMotorController, setpointsList, 0xFF );
    }
  }
  
  return NULL;
}

static void InitUserInterface( void )
{
	// Manually call the control callbacks to write the initial values for
	// amplitude and frequency to the network variables.
  SetCtrlVal( panel, PANEL_MOTOR_TOGGLE, 0 );
  ChangeStateCallback( panel, PANEL_MOTOR_TOGGLE, EVENT_COMMIT, NULL, 0, 0 );
  
	ChangeValueCallback( panel, PANEL_STIFFNESS_SLIDER, EVENT_COMMIT, NULL, 0, 0 );
}

int CVICALLBACK ConnectCallback( int panel, int control, int event, void* callbackData, int eventData1, int eventData2 )
{
  char sharedVarName[ SHARED_VARIABLE_NAME_MAX_LENGTH ] = "192.168.0.181:";
  
	if( event == EVENT_COMMIT )
	{
    // Write the new value to the appropriate network variable.
    if( control == PANEL_CONNECT_BUTTON )
    {
      // Connect to shared variables
      
      GetCtrlVal( panel, PANEL_AXIS_STRING, sharedVarName + strlen( "192.168.0.181:" ) );
      fprintf( stderr, "connecting to axis %s\n", sharedVarName );
      axisMotorController = SHMControl.InitData( sharedVarName, SHM_CONTROL_OUT );
      if( axisMotorController != NULL ) fprintf( stderr, "connected to axis %s\n\n", sharedVarName );
      
      GetCtrlVal( panel, PANEL_JOINT_STRING, sharedVarName + strlen( "192.168.0.181:" ) );
      fprintf( stderr, "connecting to joint %s\n", sharedVarName );
      jointEMGController = SHMControl.InitData( sharedVarName, SHM_CONTROL_OUT );
      if( jointEMGController != NULL ) fprintf( stderr, "connected to joint %s\n\n", sharedVarName );
      
      if( dataConnectionThreadID == INVALID_THREAD_HANDLE )
        dataConnectionThreadID = Threading.StartThread( UpdateData, NULL, THREAD_JOINABLE );
    }
	}
  
	return 0;
}

int CVICALLBACK ChangeStateCallback( int panel, int control, int event, void* callbackData, int eventData1, int eventData2 )
{
	if( event == EVENT_COMMIT )
	{
    // Write the new value to the appropriate network variable.
    if( control == PANEL_MOTOR_TOGGLE )
    {
      // Get the new value.
      int enabled;
      GetCtrlVal( panel, control, &enabled );

      if( enabled == 1 ) SHMControl.SetByteValue( axisMotorController, SHM_COMMAND_ENABLE );
      else SHMControl.SetByteValue( axisMotorController, SHM_COMMAND_DISABLE );
    }
    else if( control == PANEL_MOTOR_OFFSET_TOGGLE ) SHMControl.SetByteValue( axisMotorController, SHM_COMMAND_OFFSET );
    else if( control == PANEL_MOTOR_CAL_TOGGLE ) SHMControl.SetByteValue( axisMotorController, SHM_COMMAND_CALIBRATE );
    else if( control == PANEL_EMG_OFFSET_TOGGLE ) SHMControl.SetByteValue( jointEMGController, SHM_EMG_OFFSET );
    else if( control == PANEL_EMG_CAL_TOGGLE ) SHMControl.SetByteValue( jointEMGController, SHM_EMG_CALIBRATION );
    else if( control == PANEL_EMG_SAMPLE_TOGGLE ) SHMControl.SetByteValue( jointEMGController, SHM_EMG_SAMPLING );
	}
  
	return 0;
}

int CVICALLBACK ChangeValueCallback( int panel, int control, int event, void* callbackData, int eventData1, int eventData2 )
{
	if( event == EVENT_COMMIT )
	{
    if( control == PANEL_STIFFNESS_SLIDER )
    {
      // Get the new value.
      //double maxStiffness;
      GetCtrlVal( panel, control, &maxStiffness );
      SHMControl.SetNumericValue( axisMotorController, SHM_AXIS_STIFFNESS, maxStiffness );
    }
	}
	return 0;
}

int CVICALLBACK QuitCallback( int panel, int event, void *callbackData, int eventData1, int eventData2 )
{
  if( event == EVENT_CLOSE )
	{
		QuitUserInterface( 0 );
	}
  
	return 0;
}

