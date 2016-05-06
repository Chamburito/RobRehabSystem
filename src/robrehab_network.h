#ifndef ROBREHAB_NETWORK_H
#define ROBREHAB_NETWORK_H

#ifdef _CVI_
  #include "ip_network/cvirte_ip_connection.h"
#else
  #include "ip_network/async_ip_network.h"
#endif

#include "interfaces.h"

#include "shm_control.h"
#include "shm_axis_control.h"
#include "shm_joint_control.h"

//#include "config_parser.h"

#include "klib/kvec.h"

#include "debug/async_debug.h"

#define UPDATE_INTERVAL_MS 5

static int eventServerConnectionID;
static int axisServerConnectionID;
static int jointServerConnectionID;

static kvec_t( int ) eventClientsList;
const size_t INFO_BLOCK_SIZE = 2;

static kvec_t( int ) axisClientsList;
static kvec_t( int ) jointClientsList;

SHMController sharedRobotAxesInfo;
SHMController sharedRobotJointsInfo;
SHMController sharedRobotAxesData;
SHMController sharedRobotJointsData;

static kvec_t( int ) axisNetworkControllersList;
static kvec_t( int ) jointNetworkControllersList;

#define SUBSYSTEM RobRehabNetwork

#define ROBREHAB_NETWORK_FUNCTIONS( namespace, function_init ) \
        function_init( int, namespace, Init, const char* ) \
        function_init( void, namespace, End, void ) \
        function_init( void, namespace, Update, void )

INIT_NAMESPACE_INTERFACE( SUBSYSTEM, ROBREHAB_NETWORK_FUNCTIONS )

int RobRehabNetwork_Init( const char* configType )
{
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "Initializing RobRehab Network on thread %lx", THREAD_ID );
  if( (eventServerConnectionID = AsyncIPNetwork.OpenConnection( IP_SERVER | IP_TCP, NULL, 50000 )) == IP_CONNECTION_INVALID_ID )
    return -1;
  //if( (axisServerConnectionID = AsyncIPNetwork.OpenConnection( IP_SERVER | IP_UDP, NULL, 50001 )) == IP_CONNECTION_INVALID_ID )
  if( (axisServerConnectionID = AsyncIPNetwork.OpenConnection( IP_SERVER | IP_UDP, "226.1.1.1", 50001 )) == IP_CONNECTION_INVALID_ID )
    return -1;
  if( (jointServerConnectionID = AsyncIPNetwork.OpenConnection( IP_SERVER | IP_UDP, NULL, 50002 )) == IP_CONNECTION_INVALID_ID )
    return -1;
  
  /*DEBUG_EVENT( 1,*/DEBUG_PRINT( "Received server connection IDs: %d (Info) - %d (Data) - %d(joint)", eventServerConnectionID, axisServerConnectionID, jointServerConnectionID );
  
  kv_init( eventClientsList );
  kv_init( axisClientsList );
  kv_init( jointClientsList );
  
  kv_init( axisNetworkControllersList );
  kv_init( jointNetworkControllersList );
  
  sharedRobotAxesInfo = SHMControl.InitData( "robot_axes_info", SHM_CONTROL_OUT );
  sharedRobotJointsInfo = SHMControl.InitData( "robot_joints_info", SHM_CONTROL_OUT );
  sharedRobotAxesData = SHMControl.InitData( "robot_axes_data", SHM_CONTROL_OUT );
  sharedRobotJointsData = SHMControl.InitData( "robot_joints_data", SHM_CONTROL_OUT );
  
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "RobRehab Network initialized on thread %lx", THREAD_ID );
  
  return 0;
}

void RobRehabNetwork_End()
{
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "ending RobRehab Network on thread %lx", THREAD_ID );
  
  for( size_t eventClientIndex = 0; eventClientIndex < kv_size( eventClientsList ); eventClientIndex++ )
    AsyncIPNetwork.CloseConnection( kv_A( eventClientsList, eventClientIndex ) );
  AsyncIPNetwork.CloseConnection( eventServerConnectionID );
  /*DEBUG_EVENT( 1,*/DEBUG_PRINT( "info server %d closed", eventServerConnectionID );
  
  for( size_t axisClientIndex = 0; axisClientIndex < kv_size( axisClientsList ); axisClientIndex++ )
    AsyncIPNetwork.CloseConnection( kv_A( axisClientsList, axisClientIndex ) );
  AsyncIPNetwork.CloseConnection( axisServerConnectionID );
  /*DEBUG_EVENT( 2,*/DEBUG_PRINT( "data server %d closed", axisServerConnectionID );
  
  for( size_t emgClientIndex = 0; emgClientIndex < kv_size( jointClientsList ); emgClientIndex++ )
    AsyncIPNetwork.CloseConnection( kv_A( jointClientsList, emgClientIndex ) );
  AsyncIPNetwork.CloseConnection( jointServerConnectionID );
  /*DEBUG_EVENT( 3,*/DEBUG_PRINT( "joint server %d closed", jointServerConnectionID );
  
  SHMControl.EndData( sharedRobotAxesInfo );
  SHMControl.EndData( sharedRobotJointsInfo );
  SHMControl.EndData( sharedRobotAxesData );
  SHMControl.EndData( sharedRobotJointsData );
  
  kv_destroy( eventClientsList );
  DEBUG_EVENT( 6, "info clients list %p destroyed", eventClientsList );
  kv_destroy( axisClientsList );
  DEBUG_EVENT( 7, "data clients list %p destroyed", axisClientsList );
  kv_destroy( jointClientsList );
  DEBUG_EVENT( 8, "joint clients list %p destroyed", jointClientsList );
  
  kv_destroy( axisNetworkControllersList );
  kv_destroy( jointNetworkControllersList );
  
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "RobRehab Network ended on thread %lx", THREAD_ID );
}

static void UpdateClientEvent( int );
static void UpdateClientAxis( int );
static void UpdateClientJoint( int );

void RobRehabNetwork_Update()
{
  DEBUG_UPDATE( "updating connections on thread %lx", THREAD_ID );
  
  int newEventClientID = AsyncIPNetwork.GetClient( eventServerConnectionID );
  if( newEventClientID != IP_CONNECTION_INVALID_ID )
  {
    /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "new info client found: %d", newEventClientID );
    kv_push( int, eventClientsList, newEventClientID );
  }
  
  int newAxisClientID = AsyncIPNetwork.GetClient( axisServerConnectionID );
  if( newAxisClientID != IP_CONNECTION_INVALID_ID )
  {
    /*DEBUG_EVENT( 1,*/DEBUG_PRINT( "new data client found: %d", newAxisClientID );
    kv_push( int, axisClientsList, newAxisClientID );
  }
  
  int newjointClientID = AsyncIPNetwork.GetClient( jointServerConnectionID );
  if( newjointClientID != IP_CONNECTION_INVALID_ID )
  {
    /*DEBUG_EVENT( 1,*/DEBUG_PRINT( "new joint client found: %d", newjointClientID );
    kv_push( int, jointClientsList, newjointClientID );
  }
  
  for( size_t clientIndex = 0; clientIndex < kv_size( eventClientsList ); clientIndex++ )
    UpdateClientEvent( kv_A( eventClientsList, clientIndex ) );
  
  for( size_t clientIndex = 0; clientIndex < kv_size( axisClientsList ); clientIndex++ )
    UpdateClientAxis( kv_A( axisClientsList, clientIndex ) );
  
  for( size_t clientIndex = 0; clientIndex < kv_size( jointClientsList ); clientIndex++ )
    UpdateClientJoint( kv_A( jointClientsList, clientIndex ) );
}

static void UpdateClientEvent( int clientID )
{
  static char messageOut[ IP_MAX_MESSAGE_LENGTH ];
  
  char* messageIn = AsyncIPNetwork_ReadMessage( clientID );
  if( messageIn != NULL ) 
  {
    DEBUG_UPDATE( "received input message: %s", messageIn );
    
    uint8_t commandBlocksNumber = (uint8_t) *(messageIn++);
    for( uint8_t commandBlockIndex = 0; commandBlockIndex < commandBlocksNumber; commandBlockIndex++ )
    {
      uint8_t eventIndex = (uint8_t) *(messageIn++);
      uint8_t command = (uint8_t) *(messageIn++);
      DEBUG_PRINT( "received event %u command: %u", eventIndex, command );
      
      SHMControl.SetMaskByte( sharedRobotJointsInfo, eventIndex, 1 );
      SHMControl.SetData( sharedRobotJointsInfo, (void*) &command, eventIndex, sizeof(uint8_t) );
    }
    
    memset( messageOut, 0, IP_MAX_MESSAGE_LENGTH * sizeof(char) );
    
    uint8_t updateCommand = (uint8_t) *(messageIn++);
    if( updateCommand != 0 )
    {
      if( updateCommand == SHM_AXES_LIST ) SHMControl.GetData( sharedRobotAxesInfo, messageOut, 0, SHM_CONTROL_MAX_DATA_SIZE );
      else if( updateCommand == SHM_JOINTS_LIST ) SHMControl.GetData( sharedRobotJointsInfo, messageOut, 0, SHM_CONTROL_MAX_DATA_SIZE );
      
      AsyncIPNetwork.WriteMessage( clientID, messageOut );
    }
  }
}

const size_t AXIS_DATA_BLOCK_SIZE = SHM_AXIS_FLOATS_NUMBER * sizeof(float);
static void UpdateClientAxis( int clientID )
{
  static char messageOut[ IP_MAX_MESSAGE_LENGTH ];
  
  char* messageIn = AsyncIPNetwork_ReadMessage( clientID );
  if( messageIn != NULL ) 
  {
    DEBUG_UPDATE( "received input message: %s", messageIn );
    
    uint8_t setpointBlocksNumber = (uint8_t) *(messageIn++);
    for( uint8_t setpointBlockIndex = 0; setpointBlockIndex < setpointBlocksNumber; setpointBlockIndex++ )
    {
      uint8_t axisIndex = (uint8_t) *(messageIn++);
      uint8_t axisMask = (uint8_t) *(messageIn++);
      
      if( kv_a( int, axisNetworkControllersList, axisIndex ) == IP_CONNECTION_INVALID_ID )
      {
        DEBUG_PRINT( "new client for axis %u: %d", axisIndex, clientID );
        kv_A( axisNetworkControllersList, axisIndex ) = clientID;
      }
      else if( kv_A( axisNetworkControllersList, axisIndex ) != clientID ) continue;
      
      DEBUG_UPDATE( "receiving axis %u setpoints (mask: %x)", axisIndex, axisMask );
      SHMControl.SetMaskByte( sharedRobotAxesData, axisIndex, axisMask );
      SHMControl.SetData( sharedRobotAxesData, messageIn, axisIndex * AXIS_DATA_BLOCK_SIZE, AXIS_DATA_BLOCK_SIZE );
      
      messageIn += AXIS_DATA_BLOCK_SIZE;
    }
  }
  
  memset( messageOut, 0, IP_MAX_MESSAGE_LENGTH * sizeof(char) );
  size_t axisdataOffset = 1;
  for( size_t axisIndex = 0; axisIndex < kv_size( axisNetworkControllersList ); axisIndex++ )
  {
    if( kv_A( axisNetworkControllersList, axisIndex ) == clientID )
    {
      messageOut[ 0 ]++;
      messageOut[ axisdataOffset++ ] = (uint8_t) axisIndex;
    
      SHMControl.GetData( sharedRobotAxesData, messageOut + axisdataOffset, axisIndex * AXIS_DATA_BLOCK_SIZE, AXIS_DATA_BLOCK_SIZE );
      axisdataOffset += AXIS_DATA_BLOCK_SIZE;
    
      //DEBUG_PRINT( "sending axis %u measures: %.3f", messageOut[ measureByteIndex ], measuresList[ 0 ] );
    }
  }
  
  if( messageOut[ 0 ] > 0 ) 
  {
    DEBUG_UPDATE( "sending message %s to client %d (%u axes)", messageOut, clientID, messageOut[ 0 ] );
    AsyncIPNetwork.WriteMessage( clientID, messageOut );
  }
}

const size_t JOINT_DATA_BLOCK_SIZE = SHM_JOINT_FLOATS_NUMBER * sizeof(float);
static void UpdateClientJoint( int clientID )
{
  static char messageOut[ IP_MAX_MESSAGE_LENGTH ];
  
  memset( messageOut, 0, IP_MAX_MESSAGE_LENGTH * sizeof(char) );
  size_t jointDataOffset = 1;
  for( size_t jointIndex = 0; jointIndex < kv_size( jointNetworkControllersList ); jointIndex++ )
  {
    if( kv_A( jointNetworkControllersList, jointIndex ) == clientID )
    {
      messageOut[ 0 ]++;
      messageOut[ jointDataOffset++ ] = (uint8_t) jointIndex;
    
      SHMControl.GetData( sharedRobotJointsData, messageOut + jointDataOffset, jointIndex * JOINT_DATA_BLOCK_SIZE, JOINT_DATA_BLOCK_SIZE );
      jointDataOffset += JOINT_DATA_BLOCK_SIZE;
    }
  }
  
  if( messageOut[ 0 ] > 0 ) 
  {
    DEBUG_UPDATE( "sending message %s to client %d (%u joints)", messageOut, clientID, messageOut[ 0 ] );
    AsyncIPNetwork.WriteMessage( clientID, messageOut );
  }
}

#endif //ROBREHAB_NETWORK_H
