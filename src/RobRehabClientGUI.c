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

#include "threads/threading.h"
#include "time/timing.h"

#include "ip_network/async_ip_network.h"

//#include "shm_control.h"
#include "shm_axis_control.h"
#include "shm_joint_control.h"

#include "control_definitions.h"

#include "RobRehabClientGUI.h"

#define NUM_POINTS			20

/* Global variables */
static int panel;

/* Function prototypes */
static void* UpdateData( void* );

static int eventServerConnectionID = IP_CONNECTION_INVALID_ID;
static int axisServerConnectionID = IP_CONNECTION_INVALID_ID;
static int jointServerConnectionID = IP_CONNECTION_INVALID_ID;

//SHMController sharedRobotAxesInfo;
//SHMController sharedRobotJointsInfo;
//SHMController sharedRobotAxesData;
//SHMController sharedRobotJointsData;

char robotAxesInfo[ IP_MAX_MESSAGE_LENGTH ] = "";
char robotJointsInfo[ IP_MAX_MESSAGE_LENGTH ] = "";

Thread dataConnectionThreadID = THREAD_INVALID_HANDLE;
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
  
  AsyncIPNetwork.CloseConnection( eventServerConnectionID );
  AsyncIPNetwork.CloseConnection( axisServerConnectionID );
  AsyncIPNetwork.CloseConnection( jointServerConnectionID );
  
  //SHMControl.EndData( sharedRobotAxesInfo );
  //SHMControl.EndData( sharedRobotJointsInfo );
  //SHMControl.EndData( sharedRobotAxesData );
  //SHMControl.EndData( sharedRobotJointsData );
  
	DiscardPanel( panel );
	
  CloseCVIRTE();
  
	return 0;
}

static void* UpdateData( void* callbackData )
{
  const size_t WAIT_SAMPLES = 2;
  const double SETPOINT_UPDATE_INTERVAL = WAIT_SAMPLES * CONTROL_SAMPLING_INTERVAL;
  
  uint8_t controlData[ IP_MAX_MESSAGE_LENGTH ];
  
  double positionValues[ NUM_POINTS ], velocityValues[ NUM_POINTS ], torqueValues[ NUM_POINTS ], angleValues[ NUM_POINTS ];
  size_t axisMeasureIndex = 0, jointMeasureIndex = 0, setpointIndex = 0;
  
  double initialTime = Timing.GetExecTimeSeconds();
  double elapsedTime = 0.0, deltaTime = 0.0, measureTime = 0.0, setpointTime = 0.0, absoluteTime = initialTime; 
  
  isDataUpdateRunning = true;
  
  while( isDataUpdateRunning )
  {
    deltaTime = ( Timing.GetExecTimeSeconds() - absoluteTime );
    measureTime += deltaTime;
    setpointTime += deltaTime;
    absoluteTime = Timing.GetExecTimeSeconds();
    elapsedTime = absoluteTime - initialTime;
    
    if( measureTime > CONTROL_SAMPLING_INTERVAL )
    {
      axisMeasureIndex++;
      jointMeasureIndex++;
      
      //fprintf( stderr, "new axis/joint value indexes: %lu %lu", axisMeasureIndex, jointMeasureIndex );
      
      measureTime = 0.0;
    }

    float* measuresList = (float*) controlData;
    
    //SHMControl.GetData( sharedRobotAxesData, (void*) controlData, 0, SHM_CONTROL_MAX_DATA_SIZE );
    
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
    
    //SHMControl.GetData( sharedRobotJointsData, (void*) controlData, 0, SHM_CONTROL_MAX_DATA_SIZE );

    if( jointMeasureIndex >= NUM_POINTS )
    {
      DeleteGraphPlot( panel, PANEL_GRAPH_2, -1, VAL_DELAYED_DRAW );
      PlotY( panel, PANEL_GRAPH_2, torqueValues, NUM_POINTS, VAL_DOUBLE, VAL_THIN_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_RED );
      PlotY( panel, PANEL_GRAPH_2, angleValues, NUM_POINTS, VAL_DOUBLE, VAL_THIN_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_GREEN );

      jointMeasureIndex = 0;
    }

    angleValues[ jointMeasureIndex ] = measuresList[ SHM_JOINT_POSITION ];
    torqueValues[ jointMeasureIndex ] = measuresList[ SHM_JOINT_FORCE ];
    
    /*if( setpointTime >= SETPOINT_UPDATE_INTERVAL )
    {
      setpointsList[ SHM_AXIS_POSITION ] = sin( absoluteTime ) / 10.0 - 0.125;
      setpointsList[ SHM_AXIS_VELOCITY ] = cos( absoluteTime ) / 10.0;
      setpointsList[ SHM_AXIS_STIFFNESS ] = maxStiffness;
      setpointsList[ SHM_AXIS_TIME ] = SETPOINT_UPDATE_INTERVAL;
      
      SetCtrlVal( panel, PANEL_SETPOINT_SLIDER, setpointsList[ SHM_AXIS_POSITION ] * 360.0 );
      
      for( size_t i = 0; i < WAIT_SAMPLES; i++ )
        referenceValues[ ( setpointIndex + i ) % NUM_POINTS ] = setpointsList[ SHM_AXIS_POSITION ];
      setpointIndex += WAIT_SAMPLES;
    
      setpointTime = 0.0;
      
      SHMControl.SetData( axisMotorController, setpointsList, 0xFF );
    }*/
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
  const char listRequest[ 2 ] = "\xff";
  
	if( event == EVENT_COMMIT )
	{
    // Write the new value to the appropriate network variable.
    if( control == PANEL_CONNECT_BUTTON )
    {
      // Connect to robot server
      
      eventServerConnectionID = AsyncIPNetwork.OpenConnection( IP_CLIENT | IP_TCP, "192.168.0.181", 50000 );
      axisServerConnectionID = AsyncIPNetwork.OpenConnection( IP_CLIENT | IP_UDP, "192.168.0.181", 50001 );
      jointServerConnectionID = AsyncIPNetwork.OpenConnection( IP_CLIENT | IP_UDP, "192.168.0.181", 50002 );
      
      //sharedRobotAxesInfo = SHMControl.InitData( "192.168.0.181:robot_axes_info", SHM_CONTROL_OUT );
      //sharedRobotJointsInfo = SHMControl.InitData( "192.168.0.181:robot_joints_info", SHM_CONTROL_OUT );
      //sharedRobotAxesData = SHMControl.InitData( "192.168.0.181:robot_axes_data", SHM_CONTROL_OUT );
      //sharedRobotJointsData = SHMControl.InitData( "192.168.0.181:robot_joints_data", SHM_CONTROL_OUT );
      
      AsyncIPNetwork.WriteMessage( eventServerConnectionID, listRequest );
      //SHMControl.SetControlByte( sharedRobotAxesInfo, 0, ++listRequestCount );
      
      //SHMControl.GetData( sharedRobotAxesInfo, (void*) robotAxesInfo, 0, SHM_CONTROL_MAX_DATA_SIZE );
      //fprintf( stderr, "Read shared axes info: %s\n", robotAxesInfo );
      //SHMControl.GetData( sharedRobotJointsInfo, (void*) robotJointsInfo, 0, SHM_CONTROL_MAX_DATA_SIZE );
      //fprintf( stderr, "Read shared joints info: %s\n", robotJointsInfo );
      
      if( dataConnectionThreadID == THREAD_INVALID_HANDLE )
        dataConnectionThreadID = Threading.StartThread( UpdateData, NULL, THREAD_JOINABLE );
    }
	}
  
	return 0;
}

int CVICALLBACK ChangeStateCallback( int panel, int control, int event, void* callbackData, int eventData1, int eventData2 )
{
	if( event == EVENT_COMMIT )
	{
    // Get the new value.
    int enabled;
    GetCtrlVal( panel, control, &enabled );
    
    // Write the new value to the appropriate network variable.
    //if( control == PANEL_MOTOR_TOGGLE ) SHMControl.SetControlByte( sharedRobotJointsInfo, 0, enabled == 1 ? SHM_ROBOT_ENABLE : SHM_ROBOT_DISABLE );
    //else if( control == PANEL_OFFSET_TOGGLE ) SHMControl.SetControlByte( sharedRobotJointsInfo, 1, enabled == 1 ? SHM_JOINT_OFFSET : SHM_JOINT_MEASURE );
    //else if( control == PANEL_CAL_TOGGLE ) SHMControl.SetControlByte( sharedRobotJointsInfo, 1, enabled == 1 ? SHM_JOINT_CALIBRATE : SHM_JOINT_MEASURE );
    //else if( control == PANEL_SAMPLE_TOGGLE ) SHMControl.SetControlByte( sharedRobotJointsInfo, 1, enabled == 1 ? SHM_JOINT_SAMPLE : SHM_JOINT_MEASURE );
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
      GetCtrlVal( panel, control, &maxStiffness );
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

