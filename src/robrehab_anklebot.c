#include "async_ip_connection.h"

#include "network_axis.h"

//#include "emg_axis_control.h"
#include "shm_axis_control.h"
//#include "axis_control.h"
//#include "emg_processing.h"

#include "klib/kvec.h"
#include "klib/khash.h"
#include "klib/kson.h"

#include "async_debug.h"

static int infoServerConnectionID;
static int dataServerConnectionID;
 
static kvec_t( int ) infoClientsList;
static kvec_t( int ) dataClientsList;

typedef struct _NetworkAxis
{
  int dataClientID;
  TrajectoryPlanner* trajectoryPlanner;
}
NetworkAxis;

KHASH_MAP_INIT_INT( ShmNetAxis, NetworkAxis )
static khash_t( ShmNetAxis )* shmNetworkAxesMap = NULL;

static char axesInfoString[ IP_CONNECTION_MSG_LEN ] = ""; // String containing used axes names

enum { ROBOT_POSITION, ROBOT_VELOCITY, ROBOT_SETPOINT, ROBOT_ERROR, ROBOT_STIFFNESS, ROBOT_TORQUE, MAX_STIFFNESS, PATIENT_STIFFNESS, PATIENT_TORQUE, EMG_AGONIST, EMG_ANTAGONIST, DISPLAY_VALUES_NUMBER };

static double maxStiffness;

int RobRehabNetwork_Init()
{
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "Initializing RobRehab Network on thread %x", THREAD_ID );
  if( (infoServerConnectionID = AsyncIPConnection_Open( NULL, "50000", TCP )) == -1 )
    return -1;
  if( (dataServerConnectionID = AsyncIPConnection_Open( NULL, "50001", UDP )) == -1 )
  {
    AsyncIPConnection_Close( infoServerConnectionID );
    return -1;
  }
  
  /*DEBUG_EVENT( 1,*/DEBUG_PRINT( "Received server connection IDs: %d (Info), %d (Data)", infoServerConnectionID, dataServerConnectionID );
  
  kv_init( infoClientsList );
  kv_init( dataClientsList );
  
  shmNetworkAxesMap = kh_init( ShmNetAxis );
  
  //EMGAESControl_Init();
  //AESControl_Init();
  //EMGProcessing_Init();
  
  const char* parserString = "{ \"Ankle\" : 0x494D5431 }";
  kson_t* shmKeysBlock = kson_parse( parserString );
  
  DEBUG_PRINT( "Found %u keys in JSON file", shmKeysBlock->root->n );
  
  for( size_t shmKeyIndex = 0; shmKeyIndex < shmKeysBlock->root->n; shmKeyIndex++ )
  {
    const kson_node_t* shmKeyNode = kson_by_index( shmKeysBlock->root, shmKeyIndex );
    char* deviceName = shmKeyNode->key;
    
    DEBUG_PRINT( "Found device %s with key %s", shmKeyNode->key, shmKeyNode->v.str );
    
    int shmAxisControllerID = ShmAxisControl_Init( (int) strtol( shmKeyNode->v.str, NULL, 0 ) );    
    if( shmAxisControllerID != -1 )
    {
      int insertionStatus;
      khint_t shmNetworkAxisID = kh_put( ShmNetAxis, shmNetworkAxesMap, shmAxisControllerID, &insertionStatus );
      if( insertionStatus > 0 )
      {
        kh_value( shmNetworkAxesMap, shmNetworkAxisID ).dataClientID = -1;
        kh_value( shmNetworkAxesMap, shmNetworkAxisID ).trajectoryPlanner = TrajectoryPlanner_Init();
        
        DEBUG_PRINT( "Inserted value with key %u on position %u", kh_key( shmNetworkAxesMap, shmNetworkAxisID ), shmNetworkAxisID );
        
        if( strlen( axesInfoString ) > 0 ) strcat( axesInfoString, "|" );
        snprintf( &axesInfoString[ strlen( axesInfoString ) ], IP_CONNECTION_MSG_LEN, "%u:%s", shmNetworkAxisID, deviceName );
      }
      else
        ShmAxisControl_End( shmAxisControllerID );
    }
    
    DEBUG_PRINT( "Shm network axis list going from %u to %u", kh_begin( shmNetworkAxesMap ), kh_end( shmNetworkAxesMap ) );
  }
  
  DEBUG_PRINT( "Created axes info string: %s", axesInfoString );
  
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "RobRehab Network initialized on thread %x", THREAD_ID );
  
  return 0;
}

void RobRehabNetwork_End()
{
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "Ending RobRehab Network on thread %x", THREAD_ID );
  
  AsyncIPConnection_Close( infoServerConnectionID );
  AsyncIPConnection_Close( dataServerConnectionID );
  
  kv_destroy( infoClientsList );
  kv_destroy( dataClientsList );
  
  for( khint_t shmNetworkAxisID = (khint_t) 0; shmNetworkAxisID != kh_end( shmNetworkAxesMap ); shmNetworkAxisID++ )
  {
    ShmAxisControl_End( kh_key( shmNetworkAxesMap, shmNetworkAxisID ) );
    TrajectoryPlanner_End( kh_value( shmNetworkAxesMap, shmNetworkAxisID ).trajectoryPlanner );
  }
  kh_destroy( ShmNetAxis, shmNetworkAxesMap );
  
  //EMGAESControl_End();
  //EMGProcessing_End();
  //AESControl_End();
  
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "RobRehab Network ended on thread %x", THREAD_ID );
}

static int UpdateControlState( int );
static int UpdateControlData( int );

void RobRehabNetwork_Update()
{
  static int newInfoClientID, newDataClientID;
  
  if( (newInfoClientID = AsyncIPConnection_GetClient( infoServerConnectionID )) != -1 )
  {
    AsyncIPConnection_WriteMessage( newInfoClientID, axesInfoString );
    kv_push( int, infoClientsList, newInfoClientID );
  }
  
  if( (newDataClientID = AsyncIPConnection_GetClient( dataServerConnectionID )) != -1 )
    kv_push( int, dataClientsList, newDataClientID );
  
  for( size_t infoClientIndex = 0; infoClientIndex < kv_size( infoClientsList ); infoClientIndex++ )
    UpdateControlState( kv_A( infoClientsList, infoClientIndex ) );
  
  for( size_t dataClientIndex = 0; dataClientIndex < kv_size( dataClientsList ); dataClientIndex++ )
    UpdateControlData( kv_A( dataClientsList, dataClientIndex ) );
  
  for( khint_t shmNetworkAxisID = (khint_t) 0; shmNetworkAxisID != kh_end( shmNetworkAxesMap ); shmNetworkAxisID++ )
  {
    double* controlParametersList = ShmAxisControl_GetParametersList( kh_key( shmNetworkAxesMap, shmNetworkAxisID ) );
    double* targetsList = TrajectoryPlanner_GetTargetList( kh_value( shmNetworkAxesMap, shmNetworkAxisID ).trajectoryPlanner );
    controlParametersList[ CONTROL_SETPOINT ] = targetsList[ TRAJECTORY_POSITION ];
    ShmAxisControl_SetParameters( kh_key( shmNetworkAxesMap, shmNetworkAxisID ) );
  }
}

static int UpdateControlState( int clientID )
{
  static char messageOut[ IP_CONNECTION_MSG_LEN ];
  
  char* messageIn = AsyncIPConnection_ReadMessage( clientID );
  
  if( messageIn != NULL ) 
  {
    DEBUG_UPDATE( "received input message: %s", messageIn );
  
    for( char* axisCommand = strtok( messageIn, ":" ); axisCommand != NULL; axisCommand = strtok( NULL, ":" ) )
    {
      khint_t shmNetworkAxisID = (khint_t) strtoul( axisCommand, &axisCommand, 0 );
    
      if( !kh_exist( shmNetworkAxesMap, shmNetworkAxisID ) ) continue;
    
      DEBUG_UPDATE( "parsing axis %u command \"%s\"", shmNetworkAxisID, axisCommand );

      bool* statesList = ShmAxisControl_GetStatesList( kh_key( shmNetworkAxesMap, shmNetworkAxisID ) );
      
      statesList[ CONTROL_ENABLED ] = (bool) strtoul( axisCommand, &axisCommand, 0 );
      statesList[ CONTROL_RESET ] = (bool) strtoul( axisCommand, &axisCommand, 0 );

      if( statesList[ CONTROL_RESET ] ) kh_value( shmNetworkAxesMap, shmNetworkAxisID ).dataClientID = -1;
      
      ShmAxisControl_SetStates( kh_key( shmNetworkAxesMap, shmNetworkAxisID ) );
      
      double* parametersList = ShmAxisControl_GetParametersList( kh_key( shmNetworkAxesMap, shmNetworkAxisID ) );
      
      parametersList[ CONTROL_STIFFNESS ] = strtod( axisCommand, &axisCommand );
      parametersList[ CONTROL_DAMPING ] = strtod( axisCommand, &axisCommand );
      parametersList[ CONTROL_OFFSET ] = strtod( axisCommand, NULL );
      
      ShmAxisControl_SetParameters( kh_key( shmNetworkAxesMap, shmNetworkAxisID ) );
    }
    
    strcpy( messageOut, "" );
    for( khint_t shmNetworkAxisID = (khint_t) 0; shmNetworkAxisID != kh_end( shmNetworkAxesMap ); shmNetworkAxisID++ )
    {
      bool* statesList = ShmAxisControl_GetStatesList( kh_key( shmNetworkAxesMap, shmNetworkAxisID ) );
      
      if( strlen( messageOut ) > 0 ) strcat( messageOut, ":" );
      sprintf( &messageOut[ strlen( messageOut ) ], "%u", shmNetworkAxisID );
      for( size_t stateIndex = 0; stateIndex < CONTROL_STATES_NUMBER; stateIndex++ )
        strcat( messageOut, ( statesList[ stateIndex ] == true ) ? " 1" : " 0" );
    }
  
    if( strlen( messageOut ) > 0 ) 
    {
      DEBUG_UPDATE( "sending message %s to client %d", messageOut, clientID );
      AsyncIPConnection_WriteMessage( clientID, messageOut );
    }
  }
  
  return 0;
}

static int UpdateControlData( int clientID )
{
  static char messageOut[ IP_CONNECTION_MSG_LEN ];
  
  static double setpoint, setpointDerivative, setpointsInterval;
  
  char* messageIn = AsyncIPConnection_ReadMessage( clientID );
  
  if( messageIn != NULL ) 
  {
    DEBUG_UPDATE( "received input message: %s", messageIn );
  
    for( char* axisCommand = strtok( messageIn, ":" ); axisCommand != NULL; axisCommand = strtok( NULL, ":" ) )
    {
      khint_t shmNetworkAxisID = (khint_t) strtoul( axisCommand, &axisCommand, 0 );
    
      if( !kh_exist( shmNetworkAxesMap, shmNetworkAxisID ) ) continue;
      
      NetworkAxis* networkAxis = &(kh_value( shmNetworkAxesMap, shmNetworkAxisID ));
      
      if( networkAxis->dataClientID == -1 )
      {
        DEBUG_PRINT( "new client for axis %u: %d", shmNetworkAxisID, clientID );
        networkAxis->dataClientID = clientID;
      }
      else if( networkAxis->dataClientID != clientID ) continue;
      
      DEBUG_UPDATE( "parsing axis %u command \"%s\"", shmNetworkAxisID, axisCommand );
      
      setpoint = strtod( axisCommand, &axisCommand );
      setpointDerivative = strtod( axisCommand, &axisCommand );
      setpointsInterval = strtod( axisCommand, &axisCommand );
      
      //DEBUG_PRINT( "received setpoint: %f", setpoint );
      
      TrajectoryPlanner_SetCurve( networkAxis->trajectoryPlanner, setpoint, setpointDerivative, setpointsInterval );
    }
  }
  
  strcpy( messageOut, "" );
  for( khint_t shmNetworkAxisID = (khint_t) 0; shmNetworkAxisID != kh_end( shmNetworkAxesMap ); shmNetworkAxisID++ )
  {
    double* measuresList = ShmAxisControl_GetMeasuresList( kh_key( shmNetworkAxesMap, shmNetworkAxisID ) );
    
    if( strlen( messageOut ) > 0 ) strcat( messageOut, ":" );

    sprintf( &messageOut[ strlen( messageOut ) ], "%u", shmNetworkAxisID );

    for( size_t dimensionIndex = 0; dimensionIndex < CONTROL_DIMS_NUMBER; dimensionIndex++ )
      sprintf( &messageOut[ strlen( messageOut ) ], " %f", measuresList[ dimensionIndex ] );
  }
  
  if( strlen( messageOut ) > 0 ) 
  {
    DEBUG_UPDATE( "sending message %s to client %d", messageOut, clientID );
    AsyncIPConnection_WriteMessage( clientID, messageOut );
  }
  
  return 0;
}

int main( int argc, char* argv[] )
{
  DEBUG_PRINT( "running %s", argv[ 0 ] );
  
  RobRehabNetwork_Init();
  
  while( 1 )
  {
    RobRehabNetwork_Update();
    Timing_Delay( 1000 );
  }
  
  RobRehabNetwork_End();
  
  return 0;
}
