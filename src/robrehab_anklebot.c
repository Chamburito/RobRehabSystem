#include "async_ip_connection.h"

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

static double maxStiffness;

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
  
  DEBUG_EVENT( 0, "RobRehab Network initialized on thread %x", THREAD_ID );
  
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
    
    //DEBUG_PRINT( "\tnext setpoint: %lf (time: %lf)", targetList[ TRAJECTORY_POSITION ], targetList[ TRAJECTORY_TIME ] );
    
    AESControl_SetSetpoint( deviceID, targetList[ TRAJECTORY_POSITION ] );
  }
}


static int CVICALLBACK UpdateControlState( int index, void* ref_clientID, void* ref_callback )
{
  int clientID = *((int*) ref_clientID);
  
  static char messageOut[ IP_CONNECTION_MSG_LEN ];
  
  char* messageIn = AsyncIPConnection_ReadMessage( clientID );
  
  if( messageIn != NULL ) 
  {
    DEBUG_UPDATE( "received input message: %s", messageIn );
  
    for( char* axisCommand = strtok( messageIn, ":" ); axisCommand != NULL; axisCommand = strtok( NULL, ":" ) )
    {
      unsigned int deviceID = (unsigned int) strtoul( axisCommand, &axisCommand, 0 );
    
      if( deviceID < 0 || deviceID >= AESControl_GetDevicesNumber() ) continue;
    
      DEBUG_UPDATE( "parsing axis %u command \"%s\"", deviceID, axisCommand );
      
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
      double impedanceDamping = strtod( axisCommand, &axisCommand );
      
      AESControl_SetImpedance( deviceID, impedanceStiffness, impedanceDamping );
      
      double positionOffset = strtod( axisCommand, NULL );
      
      AESControl_SetOffset( deviceID, positionOffset );
      
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
      
      //DEBUG_PRINT( "received setpoint: %f", setpoint );
      
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
