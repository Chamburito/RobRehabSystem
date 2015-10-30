#ifndef ROBREHAB_NETWORK_H
#define ROBREHAB_NETWORK_H

#ifdef _CVI_
  #include "ip_network/cvirte_ip_connection.h"
#else
  #include "ip_network/async_ip_connection.h"
#endif

#include <cvinetv.h>
#include <utility.h>
#include <toolbox.h>

#include "interface.h"

#include "shm_axis_control.h"
#include "shm_emg_control.h"

#include "config_parser.h"

#include "klib/kvec.h"

#include "debug/async_debug.h"

const unsigned long UPDATE_INTERVAL_MS = 5;

static int eventServerConnectionID;
static int axisServerConnectionID;
static int emgServerConnectionID;

static kvec_t( int ) eventClientsList;
const size_t INFO_BLOCK_SIZE = 2;

static kvec_t( int ) axisClientsList;
static kvec_t( int ) emgClientsList;
const size_t DATA_BLOCK_SIZE = 1 + SHM_CONTROL_FLOATS_NUMBER * sizeof(float);

typedef struct _NetworkAxis
{
  int clientID;
  SHMController sharedAxis;
} 
NetworkAxis;

static kvec_t( NetworkAxis ) networkAxesList;

static kvec_t( SHMController ) networkEMGJointsList;


static char axesInfoString[ IP_MAX_MESSAGE_LENGTH ] = ""; // String containing used axes names

#define SUBSYSTEM RobRehabNetwork

#define ROBREHAB_NETWORK_FUNCTIONS( namespace, function_init ) \
        function_init( int, namespace, Init, void ) \
        function_init( void, namespace, End, void ) \
        function_init( void, namespace, Update, void )

INIT_NAMESPACE_INTERFACE( SUBSYSTEM, ROBREHAB_NETWORK_FUNCTIONS )

const char* CONFIG_KEY = "robots";
int RobRehabNetwork_Init()
{
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "Initializing RobRehab Network on thread %x", THREAD_ID );
  if( (eventServerConnectionID = AsyncIPNetwork.OpenConnection( NULL, "50000", TCP )) == -1 )
    return -1;
  if( (axisServerConnectionID = AsyncIPNetwork.OpenConnection( NULL, "50001", UDP )) == -1 )
    return -1;
  if( (emgServerConnectionID = AsyncIPNetwork.OpenConnection( NULL, "50002", UDP )) == -1 )
    return -1;
  
  /*DEBUG_EVENT( 1,*/DEBUG_PRINT( "Received server connection IDs: %d (Info) - %d (Data) - %d(EMG)", eventServerConnectionID, axisServerConnectionID, emgServerConnectionID );
  
  kv_init( eventClientsList );
  kv_init( axisClientsList );
  kv_init( emgClientsList );
  
  kv_init( networkAxesList );
  kv_init( networkEMGJointsList );
  
  if( ConfigParser.Init( "JSON" ) )
  {
    char searchPath[ PARSER_MAX_FILE_PATH_LENGTH ];
    sprintf( searchPath, "shared_%s", CONFIG_KEY );
    int configFileID = ConfigParser.LoadFileData( searchPath );
    if( configFileID != PARSED_DATA_INVALID_ID )
    {
      if( ConfigParser.HasKey( configFileID, CONFIG_KEY ) )
      {
        size_t sharedRobotsNumber = ConfigParser.GetListSize( configFileID, CONFIG_KEY );
      
        DEBUG_PRINT( "shared objects list size: %lu", sharedRobotsNumber );
      
        size_t infoMessageLength = 1 + sharedRobotsNumber * INFO_BLOCK_SIZE; 
        AsyncIPNetwork_SetMessageLength( eventServerConnectionID, infoMessageLength );
      
        DEBUG_PRINT( "info message length: %u", infoMessageLength );
      
        size_t dataMessageLength = 1 + sharedRobotsNumber * DATA_BLOCK_SIZE;
        AsyncIPNetwork_SetMessageLength( axisServerConnectionID, dataMessageLength );
        AsyncIPNetwork_SetMessageLength( emgServerConnectionID, dataMessageLength );
      
        DEBUG_PRINT( "data message length: %u", dataMessageLength );
      
        for( size_t sharedRobotIndex = 0; sharedRobotIndex < sharedRobotsNumber; sharedRobotIndex++ )
        {
          sprintf( searchPath, "%s.%lu.name", CONFIG_KEY, sharedRobotIndex );
          char* deviceName = ConfigParser.GetStringValue( configFileID, searchPath, NULL );
          if( deviceName != NULL )
          {
            DEBUG_PRINT( "found shared object %s", deviceName );
            
            sprintf( searchPath, "%s.%lu.dofs", CONFIG_KEY, sharedRobotIndex );
            size_t sharedDoFsNumber = ConfigParser.GetListSize( configFileID, searchPath );
            for( size_t dofIndex = 0; dofIndex < sharedDoFsNumber; dofIndex++ )
            {
              NetworkAxis newNetworkAxis;
              sprintf( searchPath, "%s.%lu.dofs.%lu", CONFIG_KEY, sharedRobotIndex, dofIndex );
              sprintf( searchPath, "%s-%s", deviceName, ConfigParser.GetStringValue( configFileID, searchPath, "" ) );
              if( (newNetworkAxis.sharedAxis = SHMControl.InitData( searchPath, SHM_CONTROL_OUT )) != NULL )
              {
                kv_push( NetworkAxis, networkAxesList, newNetworkAxis );
            
                if( strlen( axesInfoString ) > 0 ) strcat( axesInfoString, "|" );
                snprintf( &(axesInfoString[ strlen( axesInfoString ) ]), IP_MAX_MESSAGE_LENGTH, "%u:%s", kv_size( networkAxesList ) - 1, searchPath );
            
                DEBUG_PRINT( "got network axis %u", kv_size( networkAxesList ) - 1 );
              }
            }
            
            sprintf( searchPath, "%s.%lu.joints", CONFIG_KEY, sharedRobotIndex );
            size_t sharedJointsNumber = ConfigParser.GetListSize( configFileID, searchPath );
            for( size_t jointIndex = 0; jointIndex < sharedJointsNumber; jointIndex++ )
            {
              sprintf( searchPath, "%s.%lu.joints.%lu", CONFIG_KEY, sharedRobotIndex, jointIndex );
              SHMController sharedJoint = SHMControl.InitData( ConfigParser.GetStringValue( configFileID, searchPath, "" ), SHM_CONTROL_OUT );
              if( sharedJoint != NULL )
              {
                kv_push( SHMController, networkEMGJointsList, sharedJoint );
                DEBUG_PRINT( "got network EMG joint %u", kv_size( networkEMGJointsList ) - 1 );
              }
            }
          }
        }
      }
    
      ConfigParser.UnloadFile( configFileID );
    }
  }
  
  DEBUG_EVENT( 0, "RobRehab Network initialized on thread %x", THREAD_ID );
  
  return 0;
}

void RobRehabNetwork_End()
{
  DEBUG_EVENT( 0, "ending RobRehab Network on thread %x", THREAD_ID );
  
  for( size_t eventClientIndex = 0; eventClientIndex < kv_size( eventClientsList ); eventClientIndex++ )
    AsyncIPNetwork.CloseConnection( kv_A( eventClientsList, eventClientIndex ) );
  AsyncIPNetwork.CloseConnection( eventServerConnectionID );
  /*DEBUG_EVENT( 1,*/ "info server %d closed", eventServerConnectionID );
  
  for( size_t axisClientIndex = 0; axisClientIndex < kv_size( axisClientsList ); axisClientIndex++ )
    AsyncIPNetwork.CloseConnection( kv_A( axisClientsList, axisClientIndex ) );
  AsyncIPNetwork.CloseConnection( axisServerConnectionID );
  /*DEBUG_EVENT( 2,*/ "data server %d closed", axisServerConnectionID );
  
  for( size_t emgClientIndex = 0; emgClientIndex < kv_size( emgClientsList ); emgClientIndex++ )
    AsyncIPNetwork.CloseConnection( kv_A( emgClientsList, emgClientIndex ) );
  AsyncIPNetwork.CloseConnection( emgServerConnectionID );
  /*DEBUG_EVENT( 3,*/ "EMG server %d closed", emgServerConnectionID );
  
  kv_destroy( eventClientsList );
  DEBUG_EVENT( 6, "info clients list %p destroyed", eventClientsList );
  kv_destroy( axisClientsList );
  DEBUG_EVENT( 7, "data clients list %d destroyed", axisClientsList );
  kv_destroy( emgClientsList );
  DEBUG_EVENT( 8, "EMG clients list %d destroyed", emgClientsList );
  
  for( size_t networkAxisIndex = 0; networkAxisIndex < kv_size( networkAxesList ); networkAxisIndex++ )
    SHMControl.EndData( kv_A( networkAxesList, networkAxisIndex ).sharedAxis );
  kv_destroy( networkAxesList );
  
  for( size_t networkJointIndex = 0; networkJointIndex < kv_size( networkEMGJointsList ); networkJointIndex++ )
    SHMControl.EndData( kv_A( networkEMGJointsList, networkJointIndex ) );
  kv_destroy( networkEMGJointsList );
  
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "RobRehab Network ended on thread %x", THREAD_ID );
}

static void UpdateClientEvent( int );
static void UpdateClientAxis( int );
static void UpdateClientEMG( int );

void RobRehabNetwork_Update()
{
  DEBUG_UPDATE( "updating connections on thread %x", THREAD_ID );
  
  int newEventClientID = AsyncIPNetwork.GetClient( eventServerConnectionID );
  if( newEventClientID != INVALID_IP_CONNECTION_ID )
  {
    /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "new info client found: %d", newEventClientID );
    AsyncIPNetwork.WriteMessage( newEventClientID, axesInfoString );
    kv_push( int, eventClientsList, newEventClientID );
  }
  
  int newAxisClientID = AsyncIPNetwork.GetClient( axisServerConnectionID );
  if( newAxisClientID != INVALID_IP_CONNECTION_ID )
  {
    /*DEBUG_EVENT( 1,*/DEBUG_PRINT( "new data client found: %d", newAxisClientID );
    kv_push( int, axisClientsList, newAxisClientID );
  }
  
  int newEMGClientID = AsyncIPNetwork.GetClient( emgServerConnectionID );
  if( newEMGClientID != INVALID_IP_CONNECTION_ID )
  {
    /*DEBUG_EVENT( 1,*/DEBUG_PRINT( "new EMG client found: %d", newEMGClientID );
    kv_push( int, emgClientsList, newEMGClientID );
  }
  
  for( size_t clientIndex = 0; clientIndex < kv_size( eventClientsList ); clientIndex++ )
    UpdateClientEvent( kv_A( eventClientsList, clientIndex ) );
  
  for( size_t clientIndex = 0; clientIndex < kv_size( axisClientsList ); clientIndex++ )
    UpdateClientAxis( kv_A( axisClientsList, clientIndex ) );
  
  for( size_t clientIndex = 0; clientIndex < kv_size( emgClientsList ); clientIndex++ )
    UpdateClientEMG( kv_A( emgClientsList, clientIndex ) );
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
      DEBUG_UPDATE( "received controller %u command: %u", controllerID, command );
      
      if( command > SHM_CONTROL_BYTE_NULL && command < SHM_AXIS_BYTE_END )
      {
        if( controllerID < 0 || controllerID >= kv_size( networkAxesList ) ) continue;
        
        SHMController sharedAxis = kv_A( networkAxesList, controllerID ).sharedAxis;
        
        SHMAxisControl.SetByteValue( networkAxis->sharedData, command );
      
        uint8_t state = SHMAxisControl.GetByteValue( networkAxis->sharedData, SHM_CONTROL_REMOVE );
        if( state != SHM_CONTROL_BYTE_NULL )
        {
          messageOut[ 0 ]++;
          messageOut[ stateByteIndex++ ] = (char) controllerID;
          messageOut[ stateByteIndex++ ] = (char) state;
        }
      }
      else if( command < SHM_EMG_BYTE_END )
      {
        SHMController sharedEMGJoint = &(kv_A( networkEMGJointsList, controllerID ));
        SHMAxisControl.SetByteValue( sharedEMGJoint, command );
      }
    }
  
    if( messageOut[ 0 ] > 0 ) 
    {
      DEBUG_UPDATE( "sending message %s to client %d (%u bytes)", messageOut, clientID, stateByteIndex );
      AsyncIPNetwork_WriteMessage( clientID, messageOut );
    }
  }
}

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
      
      if( networkAxis->clientID == -1 )
      {
        DEBUG_PRINT( "new client for axis %u: %d", controllerID, clientID );
        networkAxis->clientID = clientID;
      }
      else if( networkAxis->clientID != clientID ) continue;
      
      uint8_t dataMask = (uint8_t) messageIn[ 1 ];
      
      DEBUG_UPDATE( "receiving axis %u setpoints (mask: %x)", controllerID, dataMask );
      
      float* setpointsList = (float*) &(messageIn[ 2 ]);
      SHMAxisControl.SetNumericValuesList( networkAxis->sharedData, setpointsList, dataMask );
      
      messageIn += DATA_BLOCK_SIZE;
    }
  }
  
  memset( messageOut, 0, IP_MAX_MESSAGE_LENGTH * sizeof(char) );
  size_t measureByteIndex = 1;
  for( size_t networkAxisIndex = 0; networkAxisIndex < kv_size( networkAxesList ); networkAxisIndex++ )
  {
    NetworkAxis* networkAxis = &(kv_A( networkAxesList, networkAxisIndex ));
    
    messageOut[ 0 ]++;
    messageOut[ measureByteIndex++ ] = (uint8_t) networkAxisIndex;
    
    SHMAxisControl.GetNumericValuesList( networkAxis->sharedAxis, (float*) ( messageOut + measureByteIndex ), SHM_CONTROL_PEEK );
    measureByteIndex += sizeof(float) * SHM_AXIS_FLOATS_NUMBER;
  }
  
  if( messageOut[ 0 ] > 0 ) 
  {
    DEBUG_UPDATE( "sending message %s to client %d (%u bytes)", messageOut, clientID, measureByteIndex );
    AsyncIPNetwork.WriteMessage( clientID, messageOut );
  }
}

static void UpdateClientEMG( int clientID )
{
  static char messageOut[ IP_MAX_MESSAGE_LENGTH ];
  
  memset( messageOut, 0, IP_MAX_MESSAGE_LENGTH * sizeof(char) );
  size_t measureByteIndex = 1;
  for( size_t networkJointIndex = 0; networkJointIndex < kv_size( networkAxesList ); networkJointIndex++ )
  {
    SHMController sharedEMGJoint = &(kv_A( networkEMGJointsList, networkJointIndex ));
    
    messageOut[ 0 ]++;
    messageOut[ measureByteIndex++ ] = (uint8_t) networkJointIndex;
    
    SHMAxisControl.GetNumericValuesList( sharedEMGJoint, (float*) ( messageOut + measureByteIndex ), SHM_CONTROL_PEEK );
    measureByteIndex += sizeof(float) * SHM_EMG_FLOATS_NUMBER;
  }
  
  if( messageOut[ 0 ] > 0 ) 
  {
    DEBUG_UPDATE( "sending message %s to client %d (%u bytes)", messageOut, clientID, measureByteIndex );
    AsyncIPNetwork.WriteMessage( clientID, messageOut );
  }
}

#endif //ROBREHAB_NETWORK_H
