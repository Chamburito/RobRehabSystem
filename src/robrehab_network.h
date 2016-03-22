#ifndef ROBREHAB_NETWORK_H
#define ROBREHAB_NETWORK_H

#ifdef _CVI_
  #include "ip_network/cvirte_ip_connection.h"
#else
  #include "ip_network/async_ip_network.h"
#endif

#include <cvinetv.h>
#include <utility.h>
#include <toolbox.h>

#include "interface.h"

#include "shm_control.h"
#include "shm_axis_control.h"
#include "shm_emg_control.h"

#include "config_parser.h"

#include "klib/kvec.h"

#include "debug/async_debug.h"

const unsigned long UPDATE_INTERVAL_MS = 5;

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

static kvec_t( int ) networkAxesList;
static kvec_t( int ) networkJointsList;

static char axesInfoString[ IP_MAX_MESSAGE_LENGTH ] = ""; // String containing used axes names
static char jointsInfoString[ IP_MAX_MESSAGE_LENGTH ] = ""; // String containing used joint joint names

#define SUBSYSTEM RobRehabNetwork

#define ROBREHAB_NETWORK_FUNCTIONS( namespace, function_init ) \
        function_init( int, namespace, Init, const char* ) \
        function_init( void, namespace, End, void ) \
        function_init( void, namespace, Update, void )

INIT_NAMESPACE_INTERFACE( SUBSYSTEM, ROBREHAB_NETWORK_FUNCTIONS )

int RobRehabNetwork_Init( const char* configType )
{
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "Initializing RobRehab Network on thread %x", THREAD_ID );
  if( (eventServerConnectionID = AsyncIPNetwork.OpenConnection( NULL, "50000", TCP )) == IP_CONNECTION_INVALID_ID )
    return -1;
  if( (axisServerConnectionID = AsyncIPNetwork.OpenConnection( NULL, "50001", UDP )) == IP_CONNECTION_INVALID_ID )
    return -1;
  if( (jointServerConnectionID = AsyncIPNetwork.OpenConnection( NULL, "50002", UDP )) == IP_CONNECTION_INVALID_ID )
    return -1;
  
  /*DEBUG_EVENT( 1,*/DEBUG_PRINT( "Received server connection IDs: %d (Info) - %d (Data) - %d(joint)", eventServerConnectionID, axisServerConnectionID, jointServerConnectionID );
  
  kv_init( eventClientsList );
  kv_init( axisClientsList );
  kv_init( jointClientsList );
  
  kv_init( networkAxesList );
  kv_init( networkJointsList );
  
  //AsyncIPNetwork.SetMessageLength( axisServerConnectionID, axisDataMessageLength );
  //AsyncIPNetwork.SetMessageLength( jointServerConnectionID, jointDataMessageLength );
  
  DEBUG_EVENT( 0, "RobRehab Network initialized on thread %x", THREAD_ID );
  
  return 0;
}

void RobRehabNetwork_End()
{
  DEBUG_EVENT( 0, "ending RobRehab Network on thread %x", THREAD_ID );
  
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
  
  kv_destroy( eventClientsList );
  DEBUG_EVENT( 6, "info clients list %p destroyed", eventClientsList );
  kv_destroy( axisClientsList );
  DEBUG_EVENT( 7, "data clients list %d destroyed", axisClientsList );
  kv_destroy( jointClientsList );
  DEBUG_EVENT( 8, "joint clients list %d destroyed", jointClientsList );
  
  for( size_t networkAxisIndex = 0; networkAxisIndex < kv_size( networkAxesList ); networkAxisIndex++ )
    SHMControl.EndData( kv_A( networkAxesList, networkAxisIndex ).sharedData );
  kv_destroy( networkAxesList );
  
  for( size_t networkJointIndex = 0; networkJointIndex < kv_size( networkJointsList ); networkJointIndex++ )
    SHMControl.EndData( kv_A( networkJointsList, networkJointIndex ) );
  kv_destroy( networkJointsList );
  
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "RobRehab Network ended on thread %x", THREAD_ID );
}

static void UpdateClientEvent( int );
static void UpdateClientAxis( int );
static void UpdateClientJoint( int );

void RobRehabNetwork_Update()
{
  DEBUG_UPDATE( "updating connections on thread %x", THREAD_ID );
  
  int newEventClientID = AsyncIPNetwork.GetClient( eventServerConnectionID );
  if( newEventClientID != IP_CONNECTION_INVALID_ID )
  {
    /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "new info client found: %d", newEventClientID );
    AsyncIPNetwork.WriteMessage( newEventClientID, axesInfoString );
    AsyncIPNetwork.WriteMessage( newEventClientID, jointsInfoString );
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
    
    memset( messageOut, 0, IP_MAX_MESSAGE_LENGTH * sizeof(char) );
    size_t stateByteIndex = 1;
    
    uint8_t commandBlocksNumber = (uint8_t) *(messageIn++);
    for( uint8_t commandBlockIndex = 0; commandBlockIndex < commandBlocksNumber; commandBlockIndex++ )
    {
      uint8_t controllerID = (uint8_t) *(messageIn++);
      uint8_t command = (uint8_t) *(messageIn++);
      DEBUG_PRINT( "received controller %u command: %u", controllerID, command );
      
      SHMController sharedController = NULL;
      if( command > SHM_CONTROL_BYTE_NULL && command < SHM_AXIS_BYTE_END )
      {
        if( controllerID < 0 || controllerID >= kv_size( networkAxesList ) ) continue;
        sharedController = kv_A( networkAxesList, controllerID ).sharedData;
        
        if( command == SHM_COMMAND_RESET ) kv_A( networkAxesList, controllerID ).clientID = IP_CONNECTION_INVALID_ID;
      }
      else if( command < SHM_joint_BYTE_END )
      {
        if( controllerID < 0 || controllerID >= kv_size( networkJointsList ) ) continue;
        sharedController = kv_A( networkJointsList, controllerID );
      }
      
      SHMControl.SetControlByte( sharedController, command );

      uint8_t state = SHMControl.GetControlByte( sharedController, SHM_CONTROL_REMOVE );
      if( state != SHM_CONTROL_BYTE_NULL )
      {
        messageOut[ 0 ]++;
        messageOut[ stateByteIndex++ ] = (char) controllerID;
        messageOut[ stateByteIndex++ ] = (char) state;
      }
    }
  
    if( messageOut[ 0 ] > 0 ) 
    {
      DEBUG_UPDATE( "sending message %s to client %d (%u bytes)", messageOut, clientID, stateByteIndex );
      AsyncIPNetwork.WriteMessage( clientID, messageOut );
    }
  }
}

const size_t AXIS_DATA_BLOCK_SIZE = 1 + SHM_AXIS_FLOATS_NUMBER * sizeof(float);
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
      uint8_t controllerID = (uint8_t) messageIn[ 0 ];
      if( controllerID < 0 || controllerID >= kv_size( networkAxesList ) ) continue;
      
      NetworkAxis* networkAxis = &(kv_A( networkAxesList, controllerID ));
      
      if( networkAxis->clientID == IP_CONNECTION_INVALID_ID )
      {
        DEBUG_PRINT( "new client for axis %u: %d", controllerID, clientID );
        networkAxis->clientID = clientID;
      }
      else if( networkAxis->clientID != clientID ) continue;
      
      uint8_t dataMask = (uint8_t) messageIn[ 1 ];
      
      DEBUG_UPDATE( "receiving axis %u setpoints (mask: %x)", controllerID, dataMask );
      
      float* setpointsList = (float*) &(messageIn[ 2 ]);
      SHMControl.SetData( networkAxis->sharedData, setpointsList, dataMask );
      
      messageIn += AXIS_DATA_BLOCK_SIZE;
    }
  }
  
  memset( messageOut, 0, IP_MAX_MESSAGE_LENGTH * sizeof(char) );
  size_t measureByteIndex = 1;
  for( size_t networkAxisIndex = 0; networkAxisIndex < kv_size( networkAxesList ); networkAxisIndex++ )
  {
    SHMController sharedAxis = kv_A( networkAxesList, networkAxisIndex ).sharedData;
    
    messageOut[ 0 ]++;
    messageOut[ measureByteIndex ] = (uint8_t) networkAxisIndex;
    
    float* measuresList = (float*) ( messageOut + measureByteIndex + 2 );
    
    SHMControl.GetData( sharedAxis, measuresList, SHM_CONTROL_PEEK );
    measureByteIndex += AXIS_DATA_BLOCK_SIZE;
    
    //DEBUG_PRINT( "sending axis %u measures: %.3f", messageOut[ measureByteIndex ], measuresList[ 0 ] );
  }
  
  if( messageOut[ 0 ] > 0 ) 
  {
    DEBUG_UPDATE( "sending message %s to client %d (%u bytes)", messageOut, clientID, measureByteIndex );
    AsyncIPNetwork.WriteMessage( clientID, messageOut );
  }
}

const size_t JOINT_DATA_BLOCK_SIZE = 1 + SHM_JOINT_FLOATS_NUMBER * sizeof(float);
static void UpdateClientJoint( int clientID )
{
  static char messageOut[ IP_MAX_MESSAGE_LENGTH ];
  
  memset( messageOut, 0, IP_MAX_MESSAGE_LENGTH * sizeof(char) );
  size_t measureByteIndex = 1;
  for( size_t networkJointIndex = 0; networkJointIndex < kv_size( networkAxesList ); networkJointIndex++ )
  {
    SHMController sharedjointJoint = kv_A( networkJointsList, networkJointIndex );
    
    messageOut[ 0 ]++;
    messageOut[ measureByteIndex ] = (uint8_t) networkJointIndex;
    
    SHMControl.GetData( sharedjointJoint, (float*) ( messageOut + measureByteIndex + 1 ), SHM_CONTROL_PEEK );
    measureByteIndex += JOINT_DATA_BLOCK_SIZE;
  }
  
  if( messageOut[ 0 ] > 0 ) 
  {
    DEBUG_UPDATE( "sending message %s to client %d (%u bytes)", messageOut, clientID, measureByteIndex );
    AsyncIPNetwork.WriteMessage( clientID, messageOut );
  }
}

#endif //ROBREHAB_NETWORK_H
