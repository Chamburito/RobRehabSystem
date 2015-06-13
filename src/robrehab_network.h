#ifndef ROBREHAB_NETWORK_H
#define ROBREHAB_NETWORK_H

#ifdef _CVI_
  #include "cvirte_ip_connection.h"
#else
  #include "async_ip_connection.h"
#endif

#include <cvinetv.h>
#include <utility.h>
#include <toolbox.h>
#include "Common.h"

#include "network_axis.h"

#include "emg_axis_control.h"

#include "async_debug.h"

static int infoServerConnectionID;
static int dataServerConnectionID;

static ListType infoClientsList;
static ListType dataClientsList;

typedef struct _NetworkAxis
{
  int infoClient;
  int dataClient;
  TrajectoryPlanner* trajectoryPlanner;
  double lastReceivedTime, lastLocalTime, latency;
} 
NetworkAxis;

static NetworkAxis* networkAxesList;

static char axesInfoString[ IP_CONNECTION_MSG_LEN ] = ""; // String containing used axes names

enum { ROBOT_POSITION, ROBOT_VELOCITY, ROBOT_SETPOINT, ROBOT_ERROR, ROBOT_STIFFNESS, ROBOT_TORQUE, MAX_STIFFNESS, PATIENT_STIFFNESS, PATIENT_TORQUE, EMG_AGONIST, EMG_ANTAGONIST, DISPLAY_VALUES_NUMBER };

static CNVData data = 0;

static double maxStiffness;
static CNVSubscriber gMaxToggleSubscriber, gMinToggleSubscriber, gMotorToggleSubscriber, gMaxStiffnessSubscriber;
void CVICALLBACK ChangeStateDataCallback( void*, CNVData, void* );
void CVICALLBACK ChangeValueDataCallback( void*, CNVData, void* );

static CNVBufferedWriter gWavePublisher;

int RobRehabNetwork_Init()
{
  DEBUG_EVENT( 0, "Initializing RobRehab Network on thread %x", CmtGetCurrentThreadID() );
  if( (infoServerConnectionID = AsyncIPConnection_Open( NULL, "50000", TCP )) == -1 )
    return -1;
  if( (dataServerConnectionID = AsyncIPConnection_Open( NULL, "50001", UDP )) == -1 )
  {
    AsyncIPConnection_Close( infoServerConnectionID );
    return -1;
  }
  
  DEBUG_EVENT( 1, "Received server connection IDs: %d (State), %d (Data)", infoServerConnectionID, dataServerConnectionID );
  
  infoClientsList = ListCreate( sizeof(int) );
  dataClientsList = ListCreate( sizeof(int) );
  
  EMGAxisControl_Init();
  
  networkAxesList = (NetworkAxis*) calloc( AxisControl_GetActiveAxesNumber(), sizeof(NetworkAxis) );
  
  for( size_t axisID = 0; axisID < AxisControl_GetActiveAxesNumber(); axisID++ )
  {
    if( AxisControl_GetAxisName( axisID ) != NULL )
    {
      if( strlen( axesInfoString ) > 0 ) strcat( axesInfoString, ":" );
      snprintf( &axesInfoString[ strlen( axesInfoString ) ], IP_CONNECTION_MSG_LEN, "%u %s", axisID, AxisControl_GetAxisName( axisID ) );
    }
    
    networkAxesList[ axisID ].infoClient = networkAxesList[ axisID ].dataClient = -1;
    networkAxesList[ axisID ].trajectoryPlanner = TrajectoryPlanner_Init();
    networkAxesList[ axisID ].lastReceivedTime = networkAxesList[ axisID ].lastLocalTime = ( (double) Timing_GetExecTimeMilliseconds() ) / 1000.0;
    networkAxesList[ axisID ].latency = 0.0;
  }

  int status = 0;
  
  // Create network variable connections.
	status = CNVCreateSubscriber( "\\\\localhost\\" PROCESS "\\" MAX_TOGGLE_VARIABLE, ChangeStateDataCallback, 0, 0, 10000, 0, &gMaxToggleSubscriber );
	if( status != 0 ) DEBUG_PRINT( "%s", CNVGetErrorDescription( status ) );
	status = CNVCreateSubscriber( "\\\\localhost\\" PROCESS "\\" MIN_TOGGLE_VARIABLE, ChangeStateDataCallback, 0, 0, 10000, 0, &gMinToggleSubscriber );
	if( status != 0 ) DEBUG_PRINT( "%s", CNVGetErrorDescription( status ) );
  
  status = CNVCreateBufferedWriter( "\\\\localhost\\" PROCESS "\\" WAVE_VARIABLE, 0, 0, 0, 10, 10000, 0, &gWavePublisher );
	if( status != 0 ) DEBUG_PRINT( "%s", CNVGetErrorDescription( status ) );
  
  status = CNVCreateSubscriber( "\\\\localhost\\" PROCESS "\\" ENABLE_VARIABLE, ChangeStateDataCallback, 0, 0, 10000, 0, &gMotorToggleSubscriber );
	if( status != 0 ) printf( "%s\n\n", CNVGetErrorDescription( status ) );
  
  status = CNVCreateSubscriber( "\\\\localhost\\" PROCESS "\\" STIFFNESS_VARIABLE, ChangeValueDataCallback, 0, 0, 10000, 0, &gMaxStiffnessSubscriber );
	if( status != 0 ) DEBUG_PRINT( "%s", CNVGetErrorDescription( status ) );
  
  DEBUG_EVENT( 0, "RobRehab Network initialized on thread %x", CmtGetCurrentThreadID() );
  
  return 0;
}

void RobRehabNetwork_End()
{
  DEBUG_EVENT( 0, "Ending RobRehab Network on thread %x", CmtGetCurrentThreadID() );
  
  AsyncIPConnection_Close( infoServerConnectionID );
  AsyncIPConnection_Close( dataServerConnectionID );
  
  ListDispose( infoClientsList );
  ListDispose( dataClientsList );
  
  for( size_t axisID = 0; axisID < AxisControl_GetActiveAxesNumber(); axisID++ )
    TrajectoryPlanner_End( networkAxesList[ axisID ].trajectoryPlanner );
  free( networkAxesList );
  
  if( data ) CNVDisposeData( data );
  if( gMaxToggleSubscriber ) CNVDispose( gMaxToggleSubscriber );
	if( gMinToggleSubscriber ) CNVDispose( gMinToggleSubscriber );
  if( gWavePublisher ) CNVDispose( gWavePublisher );
  if( gMotorToggleSubscriber ) CNVDispose( gMotorToggleSubscriber );
  if( gMaxStiffnessSubscriber ) CNVDispose( gMaxStiffnessSubscriber );
	CNVFinish();
  
  EMGAxisControl_End();
  
  DEBUG_EVENT( 0, "RobRehab Network ended on thread %x", CmtGetCurrentThreadID() );
}

static int CVICALLBACK UpdateAxisControlState( int, void*, void* );
static int CVICALLBACK UpdateAxisControlData( int, void*, void* );


void RobRehabNetwork_Update()
{
  static int newInfoClient;
  static int newDataClient;
  
  if( (newInfoClient = AsyncIPConnection_GetClient( infoServerConnectionID )) != -1 )
  {
    DEBUG_PRINT( "New TCP client %d from %s", newInfoClient, AsyncIPConnection_GetAddress( newInfoClient ) );
    AsyncIPConnection_WriteMessage( newInfoClient, axesInfoString );
    ListInsertItem( infoClientsList, &newInfoClient, END_OF_LIST );
  }
  
  if( (newDataClient = AsyncIPConnection_GetClient( dataServerConnectionID )) != -1 )
    ListInsertItem( dataClientsList, &newDataClient, END_OF_LIST );
  
  ListApplyToEach( infoClientsList, 1, UpdateAxisControlState, NULL );
  ListApplyToEach( dataClientsList, 1, UpdateAxisControlData, NULL );
  
  for( size_t axisID = 0; axisID < AxisControl_GetActiveAxesNumber(); axisID++ )
  {
    //networkAxesList[ axisID ].infoClient = networkAxesList[ axisID ].dataClient = 0;
    
    double* controlMeasuresList = AxisControl_GetMeasuresList( axisID );

    double* targetList = TrajectoryPlanner_GetTargetList( networkAxesList[ axisID ].trajectoryPlanner );
    
    //DEBUG_PRINT( "next setpoint: %lf (time: %lf)", targetList[ TRAJECTORY_POSITION ], targetList[ TRAJECTORY_TIME ] );
    
    AxisControl_SetSetpoint( axisID, targetList[ TRAJECTORY_POSITION ] );

    //Gamb
    static size_t valuesCount;
    static double dataArray[ DISPLAY_VALUES_NUMBER * NUM_POINTS ];
    static size_t arrayDims = DISPLAY_VALUES_NUMBER * NUM_POINTS;

    dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + ROBOT_POSITION ] = controlMeasuresList[ CONTROL_POSITION ];
    dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + ROBOT_VELOCITY ] = controlMeasuresList[ CONTROL_VELOCITY ];
    dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + ROBOT_SETPOINT ] = targetList[ TRAJECTORY_POSITION ];
    dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + ROBOT_ERROR ] = AxisControl_GetError( axisID );
    dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + ROBOT_STIFFNESS ] = controlMeasuresList[ CONTROL_STIFFNESS ];
    dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + ROBOT_TORQUE ] = controlMeasuresList[ CONTROL_TORQUE ];
    dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + MAX_STIFFNESS ] = maxStiffness;
    dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + PATIENT_TORQUE ] = EMGAxisControl_GetTorque( axisID );
    dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + PATIENT_STIFFNESS ] = EMGAxisControl_GetStiffness( axisID );
    dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + EMG_AGONIST ] = EMGProcessing_GetMuscleActivation( 0 );
    dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + EMG_ANTAGONIST ] = EMGProcessing_GetMuscleActivation( 1 );

    //DEBUG_PRINT( "EMG activation 1: %.3f - activation 2: %.3f - stiffness: %.3f", dataArray[ valuesCount * CONTROL_VALUES_NUMBER + AXIS_CURRENT ],
    //                                                                              dataArray[ valuesCount * CONTROL_VALUES_NUMBER + AXIS_TENSION ], motorParametersList[ CONTROL_STIFFNESS ] );

    valuesCount++;

    if( valuesCount >= NUM_POINTS )
    {
      CNVCreateArrayDataValue( (CNVData*) &data, CNVDouble, dataArray, 1, &arrayDims );
      CNVPutDataInBuffer( gWavePublisher, data, 1000 );
      valuesCount = 0;
    }
  }
}


static int CVICALLBACK UpdateAxisControlState( int index, void* ref_clientID, void* ref_callback )
{
  int clientID = *((int*) ref_clientID);
  
  static char messageOut[ IP_CONNECTION_MSG_LEN ];
  
  char* messageIn = AsyncIPConnection_ReadMessage( clientID );
  
  if( messageIn != NULL ) 
  {
    DEBUG_UPDATE( "received input message: %s", messageIn );
  
    for( char* axisCommand = strtok( messageIn, ":" ); axisCommand != NULL; axisCommand = strtok( NULL, ":" ) )
    {
      unsigned int axisID = (unsigned int) strtoul( axisCommand, &axisCommand, 0 );
    
      if( axisID < 0 || axisID >= AxisControl_GetActiveAxesNumber() ) continue;
      
      networkAxesList[ axisID ].infoClient = clientID;
    
      DEBUG_UPDATE( "parsing axis %u command \"%s\"", axisID, axisCommand );
      
      bool motorEnabled = (bool) strtoul( axisCommand, &axisCommand, 0 );

      if( motorEnabled ) AxisControl_EnableMotor( axisID );
      else AxisControl_DisableMotor( axisID );

      bool reset = (bool) strtoul( axisCommand, NULL, 0 );

      if( reset ) AxisControl_Reset( axisID );
    }
  }
  
  strcpy( messageOut, "" );
  for( size_t axisID = 0; axisID < AxisControl_GetActiveAxesNumber(); axisID++ )
  {
    if( networkAxesList[ axisID ].infoClient == clientID )
    {
      bool* motorStatesList = AxisControl_GetMotorStatus( axisID );
      if( motorStatesList != NULL )
      {
        if( strlen( messageOut ) > 0 ) strcat( messageOut, ":" );
        for( size_t stateIndex = 0; stateIndex < AXIS_STATES_NUMBER; stateIndex++ )
          strcat( messageOut, ( motorStatesList[ stateIndex ] == true ) ? "1 " : "0 " );
        messageOut[ strlen( messageOut ) - 1 ] = '\0';
      }
    }
  }
  
  if( strlen( messageOut ) > 0 ) 
  {
    DEBUG_UPDATE( "outgoing message: %s", messageOut );
    AsyncIPConnection_WriteMessage( clientID, messageOut );
  }
  
  return 0;
}

static int CVICALLBACK UpdateAxisControlData( int index, void* ref_clientID, void* ref_callback )
{
  int clientID = *((int*) ref_clientID);
  
  static char messageOut[ IP_CONNECTION_MSG_LEN ];
  
  static double serverDispatchTime, clientReceiveTime, clientDispatchTime, serverReceiveTime, latency;
  static double setpoint, setpointDerivative, setpointsInterval;
  
  char* messageIn = AsyncIPConnection_ReadMessage( clientID );
  
  if( messageIn != NULL ) 
  {
    DEBUG_UPDATE( "received input message: %s", messageIn );
  
    for( char* axisCommand = strtok( messageIn, ":" ); axisCommand != NULL; axisCommand = strtok( NULL, ":" ) )
    {
      unsigned int axisID = (unsigned int) strtoul( axisCommand, &axisCommand, 0 );
    
      if( axisID < 0 || axisID >= AxisControl_GetActiveAxesNumber() ) continue;
      
      networkAxesList[ axisID ].dataClient = clientID;
      
      DEBUG_UPDATE( "parsing axis %u command \"%s\"", axisID, axisCommand );
      
      serverDispatchTime = strtod( axisCommand, &axisCommand );
      clientReceiveTime = strtod( axisCommand, &axisCommand );
      clientDispatchTime = strtod( axisCommand, &axisCommand );
      serverReceiveTime = ( (double) Timing_GetExecTimeMilliseconds() ) / 1000.0;
      
      if( serverDispatchTime > 0.0 && clientReceiveTime > 0.0 )
      {
        latency = ( ( serverReceiveTime - serverDispatchTime ) - ( clientDispatchTime - clientReceiveTime ) ) / 2;
        if( latency < 0.0 ) latency = 0.0;
      }
      else
        latency = 0.0;
      
      //DEBUG_PRINT( "lag: ( ( %.3f - %.3f ) - ( %.3f - %.3f ) ) / 2 = %g", serverReceiveTime, serverDispatchTime, clientDispatchTime, clientReceiveTime, latency );
      
      networkAxesList[ axisID ].lastReceivedTime = clientDispatchTime;
      networkAxesList[ axisID ].lastLocalTime = serverReceiveTime;
      
      setpoint = strtod( axisCommand, &axisCommand );
      setpointDerivative = strtod( axisCommand, &axisCommand );
      setpointsInterval = strtod( axisCommand, &axisCommand );
      
      if( setpointsInterval > latency )
        TrajectoryPlanner_SetCurve( networkAxesList[ axisID ].trajectoryPlanner, setpoint, setpointDerivative, setpointsInterval - latency );
    }
  }
  
  strcpy( messageOut, "" );
  for( size_t axisID = 0; axisID < AxisControl_GetActiveAxesNumber(); axisID++ )
  {
    if( networkAxesList[ axisID ].dataClient == clientID )
    {
      double* controlMeasuresList = AxisControl_GetMeasuresList( axisID );
      if( controlMeasuresList != NULL )
      {
        //double* jointMeasuresList = EMGAxisControl_ApplyGains( axisID, maxStiffness );
    
        if( strlen( messageOut ) > 0 ) strcat( messageOut, ":" );
      
        clientDispatchTime = networkAxesList[ axisID ].lastReceivedTime;
        serverReceiveTime = networkAxesList[ axisID ].lastLocalTime;
        serverDispatchTime = ( (double) Timing_GetExecTimeMilliseconds() ) / 1000.0;
      
        sprintf( &messageOut[ strlen( messageOut ) ], "%u %f %f %f", axisID, clientDispatchTime, serverReceiveTime, serverDispatchTime );
        
        for( size_t dimensionIndex = 0; dimensionIndex < CONTROL_DIMS_NUMBER; dimensionIndex++ )
          sprintf( &messageOut[ strlen( messageOut ) ], " %f", controlMeasuresList[ dimensionIndex ] );
      }
    }
  }
  
  if( strlen( messageOut ) > 0 ) 
  {
    DEBUG_UPDATE( "outgoing message: %s", messageOut );
    AsyncIPConnection_WriteMessage( clientID, messageOut );
  } 
  
  return 0;
}

/*static void WriteAxisControlState( unsigned int axisID, const char* command )
{
  bool motorEnabled = (bool) strtoul( command, &command, 0 );

  if( motorEnabled ) AxisControl_EnableMotor( axisID );
  else AxisControl_DisableMotor( axisID );

  bool reset = (bool) strtoul( command, NULL, 0 );

  if( reset ) AxisControl_Reset( axisID );
}

static char* ReadAxisControlState( unsigned int axisID )
{
  const size_t STATE_MAX_LEN = 2;
  static char readout[ STATE_MAX_LEN * AXIS_STATES_NUMBER + 1 ];
  
  bool* motorStatesList = AxisControl_GetMotorStatus( axisID );
  if( motorStatesList != NULL )
  {
    strcpy( readout, "" );
    for( size_t stateIndex = 0; stateIndex < AXIS_STATES_NUMBER; stateIndex++ )
      strcat( readout, ( motorStatesList[ stateIndex ] == true ) ? "1 " : "0 " );
    readout[ strlen( readout ) - 1 ] = '\0';
  }
  
  DEBUG_UPDATE( "measure readout: %s", readout );
  
  return readout;
}

static void WriteAxisControlData( unsigned int axisID, const char* command )
{
  static double setpointsList[ TRAJECTORY_VALUES_NUMBER ];
  static size_t setpointsCount;
  
  if( clientID != -1 ) return;
  
  networkAxesList[ axisID ].dataClient = clientID;

  setpointsCount = 0;
  while( *command != '\0' && setpointsCount < TRAJECTORY_VALUES_NUMBER )
  {
    setpointsList[ setpointsCount ] = strtod( command, &command );
    setpointsCount++;
  }

  TrajectoryPlanner_SetCurve( networkAxesList[ axisID ].trajectoryPlanner, setpointsList );
}

static char* ReadAxisControlData( unsigned int axisID )
{
  const size_t VALUE_MAX_LEN = 10;
  
  static char readout[ VALUE_MAX_LEN * CONTROL_DIMS_NUMBER + 1 ];
  
  double* controlMeasuresList = AxisControl_GetMeasuresList( axisID );
  if( controlMeasuresList != NULL )
  {
    //double* jointMeasuresList = EMGAxisControl_ApplyGains( axisID, maxStiffness );
      
    strcpy( readout, "" );
    for( size_t dimensionIndex = 0; dimensionIndex < CONTROL_DIMS_NUMBER; dimensionIndex++ )
      sprintf( &readout[ strlen( readout ) ], "%.3f ", controlMeasuresList[ dimensionIndex ] );
    readout[ strlen( readout ) - 1 ] = '\0';
  }
  
  DEBUG_UPDATE( "measure readout: %s", readout );
  
  return readout;
}*/


void CVICALLBACK ChangeStateDataCallback( void* handle, CNVData data, void* callbackData )
{
	int messageCode;
  CNVGetScalarDataValue( data, CNVInt32, &messageCode );
  
  unsigned int axisID = (unsigned int) ( messageCode & 0x000000ff );
  bool enabled = (bool) ( ( messageCode & 0x0000ff00 ) / 0x100 );
  unsigned int muscleGroup = (unsigned int) ( ( messageCode & 0x00ff0000 ) / 0x10000 ); 

	if( handle == gMaxToggleSubscriber )
	{
    EMGAxisControl_ChangeState( axisID, muscleGroup, enabled ? EMG_CONTRACTION_PHASE : EMG_ACTIVATION_PHASE ); 
    
    DEBUG_PRINT( "axis %u %s %s EMG contraction phase", axisID, enabled ? "starting" : "ending", ( muscleGroup == MUSCLE_AGONIST ) ? "agonist" : "antagonist" );
	}
	else if( handle == gMinToggleSubscriber )
	{
    EMGAxisControl_ChangeState( axisID, muscleGroup, enabled ? EMG_RELAXATION_PHASE : EMG_ACTIVATION_PHASE );
    
    DEBUG_PRINT( "axis %u %s %s EMG relaxation phase", axisID, enabled ? "starting" : "ending", ( muscleGroup == MUSCLE_AGONIST ) ? "agonist" : "antagonist" );
	}
  else if( handle == gMotorToggleSubscriber )
  {
    if( enabled )
    {
      AxisControl_Reset( axisID );
  		AxisControl_EnableMotor( axisID );
    }
    else
      AxisControl_DisableMotor( axisID );
  
    DEBUG_PRINT( "motor %u %s", axisID, enabled ? "enabled" : "disabled" );
  }
}

void CVICALLBACK ChangeValueDataCallback( void* handle, CNVData data, void* callbackData )
{
  double value;
  CNVGetScalarDataValue( data, CNVDouble, &value );
  
  if( handle == gMaxStiffnessSubscriber )
	{
    maxStiffness = value;
    AxisControl_SetImpedance( 0, maxStiffness, 0.0 );
    
    DEBUG_PRINT( "new maximum stiffness: %g", maxStiffness );
	}
}


#endif //ROBREHAB_NETWORK_H
