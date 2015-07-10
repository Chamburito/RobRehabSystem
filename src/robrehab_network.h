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

//#include "emg_axis_control.h"
#include "aes_control.h"
#include "emg_processing.h"

#include "async_debug.h"

static int infoServerConnectionID;
static int dataServerConnectionID;

static ListType infoClientsList;
static ListType dataClientsList;

typedef struct _NetworkAxis
{
  int dataClientID;
  TrajectoryPlanner* trajectoryPlanner;
} 
NetworkAxis;

static NetworkAxis* networkAxesList;

typedef struct _DataClient
{
  int clientID;
  double lastReceivedTime, lastLocalTime;
}
DataClient;

static double latency = 0.0; // hack

static char axesInfoString[ IP_CONNECTION_MSG_LEN ] = ""; // String containing used axes names

enum { ROBOT_POSITION, ROBOT_VELOCITY, ROBOT_SETPOINT, ROBOT_ERROR, ROBOT_STIFFNESS, ROBOT_TORQUE, MAX_STIFFNESS, PATIENT_STIFFNESS, PATIENT_TORQUE, EMG_AGONIST, EMG_ANTAGONIST, DISPLAY_VALUES_NUMBER };

static CNVData data = 0;

static double maxStiffness;
static CNVSubscriber gMaxToggleSubscriber, gMinToggleSubscriber;
void CVICALLBACK ChangeStateDataCallback( void*, CNVData, void* );

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
  dataClientsList = ListCreate( sizeof(DataClient) );
  
  //EMGAESControl_Init();
  AESControl_Init();
  EMGProcessing_Init();
  
  networkAxesList = (NetworkAxis*) calloc( AESControl_GetDevicesNumber(), sizeof(NetworkAxis) );
  
  for( size_t deviceID = 0; deviceID < AESControl_GetDevicesNumber(); deviceID++ )
  {
    char* deviceName = AESControl_GetDeviceName( deviceID );
    if( deviceName != NULL )
    {
      if( strlen( axesInfoString ) > 0 ) strcat( axesInfoString, "|" );
      snprintf( &axesInfoString[ strlen( axesInfoString ) ], IP_CONNECTION_MSG_LEN, "%u:%s", deviceID, deviceName );
    }
    
    networkAxesList[ deviceID ].dataClientID = -1;
    networkAxesList[ deviceID ].trajectoryPlanner = TrajectoryPlanner_Init();
  }

  int status = 0;
  
  // Create network variable connections.
	status = CNVCreateSubscriber( "\\\\localhost\\" PROCESS "\\" MAX_TOGGLE_VARIABLE, ChangeStateDataCallback, 0, 0, 10000, 0, &gMaxToggleSubscriber );
	if( status != 0 ) DEBUG_PRINT( "%s", CNVGetErrorDescription( status ) );
	status = CNVCreateSubscriber( "\\\\localhost\\" PROCESS "\\" MIN_TOGGLE_VARIABLE, ChangeStateDataCallback, 0, 0, 10000, 0, &gMinToggleSubscriber );
	if( status != 0 ) DEBUG_PRINT( "%s", CNVGetErrorDescription( status ) );
  
  status = CNVCreateBufferedWriter( "\\\\localhost\\" PROCESS "\\" WAVE_VARIABLE, 0, 0, 0, 10, 10000, 0, &gWavePublisher );
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
  
  for( size_t deviceID = 0; deviceID < AESControl_GetDevicesNumber(); deviceID++ )
    TrajectoryPlanner_End( networkAxesList[ deviceID ].trajectoryPlanner );
  free( networkAxesList );
  
  if( data ) CNVDisposeData( data );
  if( gMaxToggleSubscriber ) CNVDispose( gMaxToggleSubscriber );
	if( gMinToggleSubscriber ) CNVDispose( gMinToggleSubscriber );
  if( gWavePublisher ) CNVDispose( gWavePublisher );
	CNVFinish();
  
  //EMGAESControl_End();
  EMGProcessing_End();
  AESControl_End();
  
  DEBUG_EVENT( 0, "RobRehab Network ended on thread %x", CmtGetCurrentThreadID() );
}

static int CVICALLBACK UpdateControlState( int, void*, void* );
static int CVICALLBACK UpdateControlData( int, void*, void* );


void RobRehabNetwork_Update()
{
  static int newInfoClientID, newDataClientID;
  
  if( (newInfoClientID = AsyncIPConnection_GetClient( infoServerConnectionID )) != -1 )
  {
    AsyncIPConnection_WriteMessage( newInfoClientID, axesInfoString );
    ListInsertItem( infoClientsList, &newInfoClientID, END_OF_LIST );
  }
  
  if( (newDataClientID = AsyncIPConnection_GetClient( dataServerConnectionID )) != -1 )
  {
    DataClient newDataClient = { .clientID = newDataClientID };
    newDataClient.lastReceivedTime = newDataClient.lastReceivedTime = Timing_GetExecTimeSeconds();
    ListInsertItem( dataClientsList, &newDataClient, END_OF_LIST );
  }
  
  ListApplyToEach( infoClientsList, 1, UpdateControlState, NULL );
  ListApplyToEach( dataClientsList, 1, UpdateControlData, NULL );
  
  for( size_t deviceID = 0; deviceID < AESControl_GetDevicesNumber(); deviceID++ )
  {
    double* controlMeasuresList = AESControl_GetMeasuresList( deviceID );
    double* controlParametersList = AESControl_GetParametersList( deviceID );

    double* targetList = TrajectoryPlanner_GetTargetList( networkAxesList[ deviceID ].trajectoryPlanner );
    
    DEBUG_PRINT( "\tnext setpoint: %lf (time: %lf)", targetList[ TRAJECTORY_POSITION ], targetList[ TRAJECTORY_TIME ] );
    
    AESControl_SetSetpoint( deviceID, targetList[ TRAJECTORY_POSITION ] );

    //Gamb
    static size_t valuesCount;
    static double dataArray[ DISPLAY_VALUES_NUMBER * NUM_POINTS ];
    static size_t arrayDims = DISPLAY_VALUES_NUMBER * NUM_POINTS;

    dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + ROBOT_POSITION ] = controlMeasuresList[ CONTROL_POSITION ];
    dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + ROBOT_VELOCITY ] = controlMeasuresList[ CONTROL_VELOCITY ];
    dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + ROBOT_SETPOINT ] = controlParametersList[ CONTROL_SETPOINT ];
    dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + ROBOT_ERROR ] = controlMeasuresList[ CONTROL_ERROR ]; //Timing_GetExecTimeSeconds();
    dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + ROBOT_STIFFNESS ] = controlParametersList[ CONTROL_STIFFNESS ];
    dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + ROBOT_TORQUE ] = controlMeasuresList[ CONTROL_FORCE ];
    dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + MAX_STIFFNESS ] = maxStiffness;
    dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + PATIENT_TORQUE ] = 0.0;//EMGAESControl_GetTorque( deviceID );
    dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + PATIENT_STIFFNESS ] = 0.0;//EMGAESControl_GetStiffness( deviceID );
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


static int CVICALLBACK UpdateControlState( int index, void* ref_clientID, void* ref_callback )
{
  int clientID = *((int*) ref_clientID);
  
  static char messageOut[ IP_CONNECTION_MSG_LEN ];
  
  char* messageIn = AsyncIPConnection_ReadMessage( clientID );
  
  if( messageIn != NULL ) 
  {
    DEBUG_PRINT( "received input message: %s", messageIn );
  
    for( char* axisCommand = strtok( messageIn, ":" ); axisCommand != NULL; axisCommand = strtok( NULL, ":" ) )
    {
      unsigned int deviceID = (unsigned int) strtoul( axisCommand, &axisCommand, 0 );
    
      if( deviceID < 0 || deviceID >= AESControl_GetDevicesNumber() ) continue;
    
      DEBUG_PRINT( "parsing axis %u command \"%s\"", deviceID, axisCommand );
      
      bool motorEnabled = (bool) strtoul( axisCommand, &axisCommand, 0 );

      if( motorEnabled ) AESControl_EnableMotor( deviceID );
      else AESControl_DisableMotor( deviceID );

      bool reset = (bool) strtoul( axisCommand, &axisCommand, 0 );

      if( reset ) 
      {
        AESControl_Reset( deviceID );
        networkAxesList[ deviceID ].dataClientID = -1;
      }
      
      double impedanceStiffness = strtod( axisCommand, &axisCommand );
      double impedanceDamping = strtod( axisCommand, NULL );
      
      AESControl_SetImpedance( deviceID, impedanceStiffness, impedanceDamping );
      
      maxStiffness = impedanceStiffness;
    }
    
    strcpy( messageOut, "" );
    for( size_t deviceID = 0; deviceID < AESControl_GetDevicesNumber(); deviceID++ )
    {
      bool* motorStatesList = AESControl_GetMotorStatus( deviceID );
      if( motorStatesList != NULL )
      {
        if( strlen( messageOut ) > 0 ) strcat( messageOut, ":" );
      
        sprintf( &messageOut[ strlen( messageOut ) ], "%u", deviceID );
      
        for( size_t stateIndex = 0; stateIndex < AXIS_STATES_NUMBER; stateIndex++ )
          strcat( messageOut, ( motorStatesList[ stateIndex ] == true ) ? " 1" : " 0" );
      }
    }
  
    if( strlen( messageOut ) > 0 ) 
    {
      DEBUG_UPDATE( "sending message %s to client %d", messageOut, clientID );
      AsyncIPConnection_WriteMessage( clientID, messageOut );
    }
  }
  
  return 0;
}

static int CVICALLBACK UpdateControlData( int index, void* ref_dataClient, void* ref_callback )
{
  DataClient* dataClient = (DataClient*) ref_dataClient;
  
  static char messageOut[ IP_CONNECTION_MSG_LEN ];
  
  static double serverDispatchTime, clientReceiveTime, clientDispatchTime, serverReceiveTime;
  static double setpoint, setpointDerivative, setpointsInterval;
  
  char* messageIn = AsyncIPConnection_ReadMessage( dataClient->clientID );
  
  if( messageIn != NULL ) 
  {
    DEBUG_UPDATE( "received input message: %s", messageIn );
    
    char* timeData = strtok( messageIn, "|" );
    
    serverDispatchTime = strtod( timeData, &timeData );
    clientReceiveTime = strtod( timeData, &timeData );
    clientDispatchTime = strtod( timeData, &timeData );
    serverReceiveTime = Timing_GetExecTimeSeconds();
    
    dataClient->lastReceivedTime = clientDispatchTime;
    dataClient->lastLocalTime = serverReceiveTime;
    
    char* axesData = strtok( NULL, "|" );
  
    for( char* axisCommand = strtok( axesData, ":" ); axisCommand != NULL; axisCommand = strtok( NULL, ":" ) )
    {
      unsigned int deviceID = (unsigned int) strtoul( axisCommand, &axisCommand, 0 );
    
      if( deviceID < 0 || deviceID >= AESControl_GetDevicesNumber() ) continue;
      
      NetworkAxis* networkAxis = &(networkAxesList[ deviceID ]);
      
      if( networkAxis->dataClientID == -1 )
      {
        DEBUG_PRINT( "new client for axis %u: %d", deviceID, dataClient->clientID );
        networkAxis->dataClientID = dataClient->clientID;
      }
      else if( networkAxis->dataClientID != dataClient->clientID ) continue;
      
      DEBUG_UPDATE( "parsing axis %u command \"%s\"", deviceID, axisCommand );
      
      if( serverDispatchTime > 0.0 && clientReceiveTime > 0.0 )
      {
        latency = ( ( serverReceiveTime - serverDispatchTime ) - ( clientDispatchTime - clientReceiveTime ) ) / 2;
        if( latency < 0.0 ) latency = 0.0;
      }
      else
        latency = 0.0;
      
      DEBUG_UPDATE( "lag: ( ( %.3f - %.3f ) - ( %.3f - %.3f ) ) / 2 = %g", serverReceiveTime, serverDispatchTime, clientDispatchTime, clientReceiveTime, latency );
      
      setpoint = strtod( axisCommand, &axisCommand );
      setpointDerivative = strtod( axisCommand, &axisCommand );
      setpointsInterval = strtod( axisCommand, &axisCommand );
      
      DEBUG_PRINT( "received setpoint: %f", setpoint );
      
      if( setpointsInterval > latency )
        TrajectoryPlanner_SetCurve( networkAxis->trajectoryPlanner, setpoint, setpointDerivative, setpointsInterval - latency );
    }
  }
  
  strcpy( messageOut, "" );
  for( size_t deviceID = 0; deviceID < AESControl_GetDevicesNumber(); deviceID++ )
  {
    double* controlMeasuresList = AESControl_GetMeasuresList( deviceID );
    if( controlMeasuresList != NULL )
    {
      //double* jointMeasuresList = EMGAESControl_ApplyGains( deviceID, maxStiffness );

      clientDispatchTime = dataClient->lastReceivedTime;
      serverReceiveTime = dataClient->lastLocalTime;
      serverDispatchTime = Timing_GetExecTimeSeconds();
      
      if( strlen( messageOut ) > 0 ) strcat( messageOut, ":" );
      else sprintf( messageOut, "%f %f %f|", clientDispatchTime, serverReceiveTime, serverDispatchTime );

      sprintf( &messageOut[ strlen( messageOut ) ], "%u", deviceID );

      for( size_t dimensionIndex = 0; dimensionIndex < CONTROL_DIMS_NUMBER; dimensionIndex++ )
        sprintf( &messageOut[ strlen( messageOut ) ], " %f", controlMeasuresList[ dimensionIndex ] );
    }
  }
  
  if( strlen( messageOut ) > 0 ) 
  {
    DEBUG_UPDATE( "sending message %s to client %d", messageOut, dataClient->clientID );
    AsyncIPConnection_WriteMessage( dataClient->clientID, messageOut );
  }
  
  return 0;
}

/*static void WriteAxisControlState( unsigned int deviceID, const char* command )
{
  bool motorEnabled = (bool) strtoul( command, &command, 0 );

  if( motorEnabled ) AESControl_EnableMotor( deviceID );
  else AESControl_DisableMotor( deviceID );

  bool reset = (bool) strtoul( command, NULL, 0 );

  if( reset ) AESControl_Reset( deviceID );
}

static char* ReadAxisControlState( unsigned int deviceID )
{
  const size_t STATE_MAX_LEN = 2;
  static char readout[ STATE_MAX_LEN * AXIS_STATES_NUMBER + 1 ];
  
  bool* motorStatesList = AESControl_GetMotorStatus( deviceID );
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

static void WriteAxisControlData( unsigned int deviceID, const char* command )
{
  static double setpointsList[ TRAJECTORY_VALUES_NUMBER ];
  static size_t setpointsCount;
  
  if( clientID != -1 ) return;
  
  networkAxesList[ deviceID ].dataClient = clientID;

  setpointsCount = 0;
  while( *command != '\0' && setpointsCount < TRAJECTORY_VALUES_NUMBER )
  {
    setpointsList[ setpointsCount ] = strtod( command, &command );
    setpointsCount++;
  }

  TrajectoryPlanner_SetCurve( networkAxesList[ deviceID ].trajectoryPlanner, setpointsList );
}

static char* ReadAxisControlData( unsigned int deviceID )
{
  const size_t VALUE_MAX_LEN = 10;
  
  static char readout[ VALUE_MAX_LEN * CONTROL_DIMS_NUMBER + 1 ];
  
  double* controlMeasuresList = AESControl_GetMeasuresList( deviceID );
  if( controlMeasuresList != NULL )
  {
    //double* jointMeasuresList = EMGAESControl_ApplyGains( deviceID, maxStiffness );
      
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
	/*int messageCode;
  CNVGetScalarDataValue( data, CNVInt32, &messageCode );
  
  unsigned int deviceID = (unsigned int) ( messageCode & 0x000000ff );
  bool enabled = (bool) ( ( messageCode & 0x0000ff00 ) / 0x100 );
  unsigned int muscleGroup = (unsigned int) ( ( messageCode & 0x00ff0000 ) / 0x10000 ); 

	if( handle == gMaxToggleSubscriber )
	{
    EMGAESControl_ChangeState( deviceID, muscleGroup, enabled ? EMG_CONTRACTION_PHASE : EMG_ACTIVATION_PHASE ); 
    
    //DEBUG_PRINT( "axis %u %s %s EMG contraction phase", deviceID, enabled ? "starting" : "ending", ( muscleGroup == MUSCLE_AGONIST ) ? "agonist" : "antagonist" );
	}
	else if( handle == gMinToggleSubscriber )
	{
    EMGAESControl_ChangeState( deviceID, muscleGroup, enabled ? EMG_RELAXATION_PHASE : EMG_ACTIVATION_PHASE );
    
    //DEBUG_PRINT( "axis %u %s %s EMG relaxation phase", deviceID, enabled ? "starting" : "ending", ( muscleGroup == MUSCLE_AGONIST ) ? "agonist" : "antagonist" );
	}*/
}


#endif //ROBREHAB_NETWORK_H
