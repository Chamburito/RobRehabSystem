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

//#include <ws2tcpip.h>
//#include <stdint.h>
//#include <time.h>
  
//#pragma comment(lib, "Ws2_32.lib")

#include "threads/threading.h"
#include "time/timing.h"

//#include "ip_network/async_ip_network.h"

#include "shm_control.h"
#include "shm_robot_control.h"
#include "shm_axis_control.h"
#include "shm_joint_control.h"

#include "control_definitions.h"

#include "RobRehabClientGUI.h"


const double PI = 3.1416;

const double SIGNAL_AMPLITUDE = 0.1;

const double UPDATE_PERIOD = 8.0;
const double UPDATE_FREQUENCY = 1 / UPDATE_PERIOD;

const size_t DISPLAY_POINTS_NUMBER = (size_t) ( UPDATE_PERIOD / CONTROL_PASS_INTERVAL );

const int PHASE_CYCLES_NUMBER = 5;

/* Global variables */
static int panel;

/* Function prototypes */
static void* UpdateData( void* );

SHMController sharedRobotAxesInfo;
SHMController sharedRobotJointsInfo;
SHMController sharedRobotAxesData;
SHMController sharedRobotJointsData;

char robotAxesInfo[ SHM_CONTROL_MAX_DATA_SIZE ] = "";
char robotJointsInfo[ SHM_CONTROL_MAX_DATA_SIZE ] = "";

Thread dataConnectionThreadID = THREAD_INVALID_HANDLE;
bool isDataUpdateRunning = false;

double maxStiffness = 0.0;
double calibrationTime = 0.0;
double setpointTime = 0.0;
double targetStiffness = 0.0;
int stiffnessPhase = 0, stiffnessPhaseIncrement = 1;
int setpointPhase = 0, setpointDirection = 1;

enum ControlState currentControlState = CONTROL_OPERATION;

void InitUserInterface( void );

/* Program entry-point */
int main( int argc, char *argv[] )
{
	if( InitCVIRTE( 0, argv, 0 ) == 0 )
		return -1;
  
	if( (panel = LoadPanel( 0, "RobRehabClientGUI.uir", PANEL )) < 0 )
		return -1;

  SetCtrlVal( panel, PANEL_SAMPLE_TOGGLE, 0 );
  ChangeValueCallback( panel, PANEL_SAMPLE_TOGGLE, EVENT_COMMIT, NULL, 0, 0 );
  
	InitUserInterface();
	DisplayPanel( panel );
	RunUserInterface();
	
  isDataUpdateRunning = false;
  Threading.WaitExit( dataConnectionThreadID, 5000 );
  
  SHMControl.EndData( sharedRobotAxesInfo );
  SHMControl.EndData( sharedRobotJointsInfo );
  SHMControl.EndData( sharedRobotAxesData );
  SHMControl.EndData( sharedRobotJointsData );
  
	DiscardPanel( panel );
	
  CloseCVIRTE();
  
	return 0;
}

static void* UpdateData( void* callbackData )
{
  const size_t WAIT_SAMPLES = 2;
  const double SETPOINT_UPDATE_INTERVAL = WAIT_SAMPLES * CONTROL_PASS_INTERVAL;
  
  uint8_t measureData[ SHM_CONTROL_MAX_DATA_SIZE ], setpointData[ SHM_CONTROL_MAX_DATA_SIZE ];
  
  double positionValues[ DISPLAY_POINTS_NUMBER ], velocityValues[ DISPLAY_POINTS_NUMBER ];
  double torqueValues[ DISPLAY_POINTS_NUMBER ], angleValues[ DISPLAY_POINTS_NUMBER ];
  double referenceValues[ DISPLAY_POINTS_NUMBER ];
  size_t displayPointIndex = 0;
  
  double initialTime = Timing.GetExecTimeSeconds();
  double elapsedTime = 0.0, deltaTime = 0.0, measureTime = 0.0, absoluteTime = initialTime; 
  
  isDataUpdateRunning = true;
  
  while( isDataUpdateRunning )
  {
    deltaTime = ( Timing.GetExecTimeSeconds() - absoluteTime );
    absoluteTime = Timing.GetExecTimeSeconds();
    elapsedTime = absoluteTime - initialTime;
    
    measureTime += deltaTime;
    if( measureTime > CONTROL_PASS_INTERVAL )
    {
      displayPointIndex++;
      //fprintf( stderr, "new axis/joint value indexes: %lu %lu", axisMeasureIndex, jointMeasureIndex );
      measureTime = 0.0;
    }

    float* measuresList = (float*) measureData;
    
    SHMControl.GetData( sharedRobotAxesData, (void*) measureData, 0, SHM_CONTROL_MAX_DATA_SIZE );
    
    if( displayPointIndex >= DISPLAY_POINTS_NUMBER )
    {
      DeleteGraphPlot( panel, PANEL_GRAPH_1, -1, VAL_DELAYED_DRAW );
      
      PlotY( panel, PANEL_GRAPH_1, referenceValues, DISPLAY_POINTS_NUMBER, VAL_DOUBLE, VAL_FAT_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_RED );
      
      PlotY( panel, PANEL_GRAPH_1, positionValues, DISPLAY_POINTS_NUMBER, VAL_DOUBLE, VAL_FAT_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_YELLOW );
      PlotY( panel, PANEL_GRAPH_1, velocityValues, DISPLAY_POINTS_NUMBER, VAL_DOUBLE, VAL_THIN_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_GREEN );

      DeleteGraphPlot( panel, PANEL_GRAPH_2, -1, VAL_DELAYED_DRAW );
      PlotY( panel, PANEL_GRAPH_2, torqueValues, DISPLAY_POINTS_NUMBER, VAL_DOUBLE, VAL_THIN_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_RED );
      PlotY( panel, PANEL_GRAPH_2, angleValues, DISPLAY_POINTS_NUMBER, VAL_DOUBLE, VAL_THIN_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_GREEN );
      
      displayPointIndex = 0;
      
      if( currentControlState == CONTROL_OPTIMIZATION )
      {
        stiffnessPhase += stiffnessPhaseIncrement;
        if( stiffnessPhase == 3 * PHASE_CYCLES_NUMBER ) 
        {
          stiffnessPhaseIncrement = -1;
          stiffnessPhase = 3 * PHASE_CYCLES_NUMBER - 1;
        }
        else if( stiffnessPhase == -1 ) 
        {
          stiffnessPhaseIncrement = 1;
          stiffnessPhase = 0;
        }
      
        setpointPhase++;
        if( setpointPhase == 3 * PHASE_CYCLES_NUMBER ) SetCtrlVal( panel, PANEL_CALIBRATION_LED, 1 );
        else if( setpointPhase == 6 * PHASE_CYCLES_NUMBER ) setpointDirection = -1;
        else if( setpointPhase == 9 * PHASE_CYCLES_NUMBER )
        {
          SetCtrlVal( panel, PANEL_SAMPLE_TOGGLE, 0 );
          SetCtrlVal( panel, PANEL_STIFFNESS_SLIDER, 0.0 );
          SetCtrlVal( panel, PANEL_SETPOINT_SLIDER, 0.0 );
          ChangeStateCallback( panel, PANEL_SAMPLE_TOGGLE, EVENT_COMMIT, NULL, 0, 0 );
        }
      
        targetStiffness = 30.0 * ( stiffnessPhase / PHASE_CYCLES_NUMBER );
        if( targetStiffness > 100.0 ) targetStiffness = 100.0;
      }
    }

    positionValues[ displayPointIndex ] = measuresList[ SHM_AXIS_POSITION ] * 6.28;
    velocityValues[ displayPointIndex ] = measuresList[ SHM_AXIS_VELOCITY ] * 6.28;

    SetCtrlVal( panel, PANEL_MEASURE_SLIDER, measuresList[ SHM_AXIS_POSITION ] * 360.0 );
    
    SHMControl.GetData( sharedRobotJointsData, (void*) measureData, 0, SHM_CONTROL_MAX_DATA_SIZE );

    angleValues[ displayPointIndex ] = measuresList[ SHM_JOINT_POSITION ];
    torqueValues[ displayPointIndex ] = measuresList[ SHM_JOINT_FORCE ];

    float* setpointsList = (float*) setpointData;

    double referencePosition = sin( 2 * PI * UPDATE_FREQUENCY * absoluteTime ) * SIGNAL_AMPLITUDE;
    double referenceVelocity = cos( 2 * PI * UPDATE_FREQUENCY * absoluteTime ) * SIGNAL_AMPLITUDE;

    setpointTime += deltaTime;
    if( setpointTime >= SETPOINT_UPDATE_INTERVAL )
    {
      uint8_t setpointMask = 0;

      if( currentControlState == CONTROL_OPTIMIZATION )
      {
        SetCtrlVal( panel, PANEL_STIFFNESS_SLIDER, maxStiffness * 0.99 + targetStiffness * 0.01 );
        ChangeValueCallback( panel, PANEL_STIFFNESS_SLIDER, EVENT_COMMIT, NULL, 0, 0 );

        setpointsList[ SHM_AXIS_POSITION ] = setpointDirection * (float) referencePosition - 0.125;
        SHM_CONTROL_SET_BIT( setpointMask, SHM_AXIS_POSITION );
        setpointsList[ SHM_AXIS_VELOCITY ] = setpointDirection * (float) referenceVelocity;
        SHM_CONTROL_SET_BIT( setpointMask, SHM_AXIS_VELOCITY );
      }

      setpointsList[ SHM_AXIS_STIFFNESS ] = maxStiffness;
      SHM_CONTROL_SET_BIT( setpointMask, SHM_AXIS_STIFFNESS );
      //setpointsList[ SHM_AXIS_TIME ] = SETPOINT_UPDATE_INTERVAL;

      SetCtrlVal( panel, PANEL_SETPOINT_SLIDER, ( referencePosition - 0.125 ) * 360.0 );

      setpointTime = 0.0;

      SHMControl.SetData( sharedRobotAxesData, (void*) setpointData, 0, SHM_CONTROL_MAX_DATA_SIZE );
      SHMControl.SetControlByte( sharedRobotAxesData, 0, setpointMask );
    }

    referenceValues[ displayPointIndex ] = ( referencePosition - 0.125 ) * 6.28;
    
    if( currentControlState == CONTROL_CALIBRATION )
    {
      calibrationTime += deltaTime;
      
      if( fmod( calibrationTime, 15.0 ) > 10.0 ) SetCtrlVal( panel, PANEL_CALIBRATION_LED, 1 );
      else SetCtrlVal( panel, PANEL_CALIBRATION_LED, 0 );
    }
    
    //fprintf( stderr, "calibration timer: %g\r", calibrationTime );
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
  static uint8_t listRequestsCount;
  
	if( event == EVENT_COMMIT )
	{
    // Write the new value to the appropriate network variable.
    if( control == PANEL_CONNECT_BUTTON )
    {
      // Connect to robot server
      
      SetCtrlVal( panel, PANEL_MOTOR_TOGGLE, 1 );
      
      sharedRobotAxesInfo = SHMControl.InitData( "192.168.0.181:robot_axes_info", SHM_CONTROL_OUT );
      sharedRobotJointsInfo = SHMControl.InitData( "192.168.0.181:robot_joints_info", SHM_CONTROL_OUT );
      sharedRobotAxesData = SHMControl.InitData( "192.168.0.181:robot_axes_data", SHM_CONTROL_OUT );
      sharedRobotJointsData = SHMControl.InitData( "192.168.0.181:robot_joints_data", SHM_CONTROL_OUT );
      
      SHMControl.SetControlByte( sharedRobotAxesInfo, 0, ++listRequestsCount );
      
      SHMControl.GetData( sharedRobotAxesInfo, (void*) robotAxesInfo, 0, SHM_CONTROL_MAX_DATA_SIZE );
      fprintf( stderr, "Read shared axes info: %s\n", robotAxesInfo );
      SHMControl.GetData( sharedRobotJointsInfo, (void*) robotJointsInfo, 0, SHM_CONTROL_MAX_DATA_SIZE );
      fprintf( stderr, "Read shared joints info: %s\n", robotJointsInfo );
      
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
    maxStiffness = 0.0;
    targetStiffness = 0.0;
    stiffnessPhase = 0;
    stiffnessPhaseIncrement = 1;
    setpointTime = 0.0;
    setpointPhase = 0;
    setpointDirection = 1;
    calibrationTime = 0.0;
    SetCtrlVal( panel, PANEL_CALIBRATION_LED, 0 );
    
    // Get the new value.
    int enabled;
    GetCtrlVal( panel, control, &enabled );
    
    // Reset command values
    SHMControl.SetControlByte( sharedRobotJointsInfo, 0, 0x00 );
    SHMControl.SetControlByte( sharedRobotJointsInfo, 0, 0x00 );
    SHMControl.SetControlByte( sharedRobotJointsInfo, 0, 0x00 );
    SHMControl.SetControlByte( sharedRobotJointsInfo, 0, 0x00 );
    
    // Write the new value to the appropriate network variable.
    if( control == PANEL_MOTOR_TOGGLE ) SHMControl.SetControlByte( sharedRobotJointsInfo, 0, ( enabled == 1 ) ? SHM_ROBOT_ENABLE : SHM_ROBOT_DISABLE );
    else if( control == PANEL_OFFSET_TOGGLE ) 
    {
      currentControlState = ( enabled == 1 ) ? CONTROL_OFFSET : CONTROL_OPERATION;
      SHMControl.SetControlByte( sharedRobotJointsInfo, 0, ( enabled == 1 ) ? SHM_ROBOT_OFFSET : SHM_ROBOT_OPERATE );
    }
    else if( control == PANEL_CAL_TOGGLE ) 
    {
      currentControlState = ( enabled == 1 ) ? CONTROL_CALIBRATION : CONTROL_OPERATION;
      SHMControl.SetControlByte( sharedRobotJointsInfo, 0, ( enabled == 1 ) ? SHM_ROBOT_CALIBRATE : SHM_ROBOT_OPERATE );
    }
    else if( control == PANEL_SAMPLE_TOGGLE ) 
    {
      currentControlState = ( enabled == 1 ) ? CONTROL_OPTIMIZATION : CONTROL_OPERATION;
      SHMControl.SetControlByte( sharedRobotJointsInfo, 0, ( enabled == 1 ) ? SHM_ROBOT_OPTIMIZE : SHM_ROBOT_OPERATE );
    }
	}
  
	return 0;
}

int CVICALLBACK InsertUserNameCallback( int panel, int control, int event, void *callbackData, int eventData1, int eventData2 )
{
  if( event == EVENT_COMMIT )
	{
    if( control == PANEL_USER_NAME_INPUT )
    {
      // Get the new value
      char userName[ SHM_CONTROL_MAX_DATA_SIZE ];
      GetCtrlVal( panel, control, userName );
      SHMControl.SetControlByte( sharedRobotJointsInfo, 0, SHM_ROBOT_SET_USER );
      SHMControl.SetData( sharedRobotJointsInfo, (void*) userName, 0, SHM_CONTROL_MAX_DATA_SIZE );
    }
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
      //fprintf( stderr, "set stiffness to %g\r", maxStiffness );
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

