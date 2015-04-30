#ifndef ROBREHAB_NETWORK_H
#define ROBREHAB_NETWORK_H

#include <cvinetv.h>
#include <utility.h>
#include "Common.h"

#ifdef _CVI_
  #include "cvirte_ip_connection.h"
#else
  #include "async_ip_connection.h"
#endif

#include "emg_axis_control.h"

#include "async_debug.h"

static int infoServerConnectionID;
static int dataServerConnectionID;

static ListType infoClientsList;
static ListType dataClientsList;

typedef struct _NetworkAxis
{
  unsigned int axisID;
  int infoClient;
  int dataClient;
} 
NetworkAxis;

static NetworkAxis* networkAxesList;

static char axesInfoString[ IP_CONNECTION_MSG_LEN ] = ""; // String containing used axes names

enum { ROBOT_POSITION, ROBOT_VELOCITY, ROBOT_SETPOINT, ROBOT_STIFFNESS, MAX_STIFFNESS, PATIENT_STIFFNESS, PATIENT_TORQUE, EMG_AGONIST, EMG_ANTAGONIST, DISPLAY_VALUES_NUMBER };

static CNVData data = 0;

static double maxStiffness, networkSetpoint;
static CNVSubscriber gMaxToggleSubscriber, gMinToggleSubscriber, gMotorToggleSubscriber, gMaxStiffnessSubscriber, gSetpointSubscriber;
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
      snprintf( axesInfoString, IP_CONNECTION_MSG_LEN, "%s%u:%s:", axesInfoString, axisID, AxisControl_GetAxisName( axisID ) );
    
    networkAxesList[ axisID ].infoClient = networkAxesList[ axisID ].dataClient = 0;
  }
  
  if( strlen( axesInfoString ) > 0 )
    axesInfoString[ strlen( axesInfoString ) - 1 ] = '\0';
  
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
	status = CNVCreateSubscriber( "\\\\localhost\\" PROCESS "\\" SETPOINT_VARIABLE, ChangeValueDataCallback, 0, 0, 10000, 0, &gSetpointSubscriber );
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
  
  if( data ) CNVDisposeData( data );
  if( gMaxToggleSubscriber ) CNVDispose( gMaxToggleSubscriber );
	if( gMinToggleSubscriber ) CNVDispose( gMinToggleSubscriber );
  if( gWavePublisher ) CNVDispose( gWavePublisher );
  if( gMotorToggleSubscriber ) CNVDispose( gMotorToggleSubscriber );
	CNVFinish();
  
  EMGAxisControl_End();
  
  DEBUG_EVENT( 0, "RobRehab Network ended on thread %x", CmtGetCurrentThreadID() );
}

static int CVICALLBACK UpdateMotorState( int, void*, void* );
static int CVICALLBACK UpdateMotorData( int, void*, void* );

static int CVICALLBACK UpdateAxisControl( int, void*, void* );

typedef char* (*ProcessAxisCommandFunction)( int, unsigned int, const char* );
static char* ProcessAxisControlState( int, unsigned int, const char* );
static char* ProcessAxisControlData( int, unsigned int, const char* );


void RobRehabNetwork_Update()
{
  static int newInfoClient;
  static int newDataClient;
  
  if( (newInfoClient = AsyncIPConnection_GetClient( infoServerConnectionID )) != -1 )
  {
    AsyncIPConnection_WriteMessage( newInfoClient, axesInfoString );
    ListInsertItem( infoClientsList, &newInfoClient, END_OF_LIST );
  }
  
  if( (newDataClient = AsyncIPConnection_GetClient( dataServerConnectionID )) != -1 )
    ListInsertItem( dataClientsList, &newDataClient, END_OF_LIST );
  
  for( size_t axisID = 0; axisID < AxisControl_GetActiveAxesNumber(); axisID++ )
    networkAxesList[ axisID ].infoClient = networkAxesList[ axisID ].dataClient = 0;
  
  ProcessAxisCommandFunction ref_ProcessAxisCommand;
  ref_ProcessAxisCommand = ProcessAxisControlState;
  ListApplyToEach( infoClientsList, 1, UpdateAxisControl, &ref_ProcessAxisCommand );
  ref_ProcessAxisCommand = ProcessAxisControlData;
  ListApplyToEach( dataClientsList, 1, UpdateAxisControl, &ref_ProcessAxisCommand );
}


static int CVICALLBACK UpdateAxisControl( int index, void* ref_clientID, void* ref_callback )
{
  int clientID = *((int*) ref_clientID);
  ProcessAxisCommandFunction ref_ProcessAxisCommand = *((ProcessAxisCommandFunction*) ref_callback);
  
  static char messageOut[ IP_CONNECTION_MSG_LEN ];
  
  char* messageIn = AsyncIPConnection_ReadMessage( clientID );
  
  if( messageIn == NULL ) return 0;
  
  DEBUG_PRINT( "received input message: %s", messageIn );
  
  strcpy( messageOut, "" );
  for( char* axisCommand = strtok( messageIn, ":" ); axisCommand != NULL; axisCommand = strtok( NULL, ":" ) )
  {
    unsigned int axisID = (unsigned int) strtoul( axisCommand, &axisCommand, 0 );
    
    if( axisID < 0 || axisID >= AxisControl_GetActiveAxesNumber() ) continue;
    
    DEBUG_UPDATE( "parsing axis %u command \"%s\"", axisID, axisCommand );
    char* readout = ref_ProcessAxisCommand( clientID, axisID, axisCommand );
    if( strcmp( readout, "" ) != 0 )
    {
      if( strlen( messageOut ) > 0 ) strcat( messageOut, ":" );
      
      snprintf( messageOut, IP_CONNECTION_MSG_LEN, "%s%u %s", messageOut, axisID, readout );
    }
  }
  
  DEBUG_UPDATE( "generated output message: %s", messageOut );
  //AsyncIPConnection_WriteMessage( clientID, messageOut );
  
  return 0;
}

static char* ProcessAxisControlState( int clientID, unsigned int axisID, const char* command )
{
  const size_t STATE_MAX_LEN = 2;
  static char readout[ STATE_MAX_LEN * AXIS_STATES_NUMBER + 1 ];
  
  if( networkAxesList[ axisID ].infoClient == 0 )
  {
    networkAxesList[ axisID ].infoClient = clientID;

    bool motorEnabled = (bool) strtoul( command, &command, 0 );

    if( motorEnabled ) AxisControl_EnableMotor( axisID );
    else AxisControl_DisableMotor( axisID );

    bool reset = (bool) strtoul( command, NULL, 0 );

    if( reset ) AxisControl_Reset( axisID );

    bool* motorStatesList = AxisControl_GetMotorStatus( axisID );
    if( motorStatesList != NULL )
    {
      strcpy( readout, "" );
      for( size_t stateIndex = 0; stateIndex < AXIS_STATES_NUMBER; stateIndex++ )
        strcat( readout, ( motorStatesList[ stateIndex ] == true ) ? "1 " : "0 " );
      readout[ strlen( readout ) - 1 ] = '\0';
    }
  }
  
  return readout;
}

static char* ProcessAxisControlData( int clientID, unsigned int axisID, const char* command )
{
  const size_t VALUE_MAX_LEN = 10;
  const size_t SETPOINTS_MAX_NUMBER = IP_CONNECTION_MSG_LEN / VALUE_MAX_LEN;
  const size_t CONTROL_VALUES_NUMBER = AXIS_DIMS_NUMBER + CONTROL_PARAMS_NUMBER;
  
  static char readout[ VALUE_MAX_LEN * CONTROL_VALUES_NUMBER + 1 ];
  
  static double setpointsList[ SETPOINTS_MAX_NUMBER ];
  static size_t setpointsCount;
  
  if( networkAxesList[ axisID ].dataClient == 0 )
  {
    networkAxesList[ axisID ].dataClient = clientID;
    
    setpointsCount = 0;
    while( *command != '\0' )
    {
      setpointsList[ setpointsCount ] = strtod( command, &command );
      //setpointsList[ setpointsCount ] = networkSetpoint; // Gamb
      setpointsCount++;
    }
    DEBUG_PRINT( "loading %u setpoints to axis %u control", setpointsCount, axisID );
    AxisControl_EnqueueSetpoints( axisID, setpointsList, setpointsCount );
    
    double* motorMeasuresList = AxisControl_GetSensorMeasures( axisID );
    if( motorMeasuresList != NULL )
    {
      double* motorParametersList = AxisControl_GetParameters( axisID );
      double* jointMeasuresList = EMGAxisControl_ApplyGains( axisID, maxStiffness );
      
      strcpy( readout, "" );
      for( size_t dimensionIndex = 0; dimensionIndex < AXIS_DIMS_NUMBER; dimensionIndex++ )
        sprintf( &readout[ strlen( readout ) ], "%.3f ", motorMeasuresList[ dimensionIndex ] );
      for( size_t parameterIndex = 0; parameterIndex < CONTROL_PARAMS_NUMBER; parameterIndex++ )
        sprintf( &readout[ strlen( readout ) ], "%.3f ", motorParametersList[ parameterIndex ] );
      readout[ strlen( readout ) - 1 ] = '\0';
      
      //Gamb 
      static size_t valuesCount;
      static double dataArray[ DISPLAY_VALUES_NUMBER * NUM_POINTS ];
      static size_t arrayDims = DISPLAY_VALUES_NUMBER * NUM_POINTS;

      dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + ROBOT_POSITION ] = motorMeasuresList[ AXIS_POSITION ];
      dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + ROBOT_VELOCITY ] = motorMeasuresList[ AXIS_VELOCITY ];
      dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + ROBOT_SETPOINT ] = motorParametersList[ CONTROL_SETPOINT ];
      dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + ROBOT_STIFFNESS ] = motorParametersList[ PROPORTIONAL_GAIN ];
      dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + MAX_STIFFNESS ] = maxStiffness;
      dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + PATIENT_TORQUE ] = jointMeasuresList[ JOINT_TORQUE ];
      dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + PATIENT_STIFFNESS ] = jointMeasuresList[ JOINT_STIFFNESS ];
      dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + EMG_AGONIST ] = EMGProcessing_GetMuscleActivation( 0 );
      dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + EMG_ANTAGONIST ] = EMGProcessing_GetMuscleActivation( 1 );
      
      //DEBUG_PRINT( "EMG activation 1: %.3f - activation 2: %.3f - stiffness: %.3f", dataArray[ valuesCount * CONTROL_VALUES_NUMBER + AXIS_TORQUE ],
      //                                                                              dataArray[ valuesCount * CONTROL_VALUES_NUMBER + AXIS_TENSION ], motorParametersList[ PROPORTIONAL_GAIN ] );
      
      valuesCount++;
      
      if( valuesCount >= NUM_POINTS )
      {
        CNVCreateArrayDataValue( (CNVData*) &data, CNVDouble, dataArray, 1, &arrayDims );
    	  CNVPutDataInBuffer( gWavePublisher, data, 1000 );
        valuesCount = 0;
      }
    }
  }
  
  DEBUG_UPDATE( "measure readout: %s", readout );
  
  return readout;
}


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
    
    DEBUG_PRINT( "new maximum stiffness: %g", maxStiffness );
	}
	else if( handle == gSetpointSubscriber )
	{
    networkSetpoint = value;
    
    DEBUG_PRINT( "new position setpoint: %g", networkSetpoint );
	}
}


#endif //ROBREHAB_NETWORK_H
