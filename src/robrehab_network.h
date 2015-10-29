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

static int infoServerConnectionID;
static int dataServerConnectionID;
static int emgServerConnectionID;

static kvec_t( int ) infoClientsList;
const size_t INFO_BLOCK_SIZE = 2;

static kvec_t( int ) dataClientsList;
static kvec_t( int ) emgClientsList;
const size_t DATA_BLOCK_SIZE = 1 + SHM_CONTROL_FLOATS_NUMBER * sizeof(float);

typedef struct _NetworkAxis
{
  int axisClientID;
  SHMController sharedAxis;
} 
NetworkAxis;

static kvec_t( NetworkAxis ) networkAxesList;

static kvec_t( SHMController ) networEMGsList;


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
  if( (infoServerConnectionID = AsyncIPNetwork.OpenConnection( NULL, "50000", TCP )) == -1 )
    return -1;
  if( (dataServerConnectionID = AsyncIPNetwork.OpenConnection( NULL, "50001", UDP )) == -1 )
    return -1;
  if( (emgServerConnectionID = AsyncIPNetwork.OpenConnection( NULL, "50002", UDP )) == -1 )
    return -1;
  
  /*DEBUG_EVENT( 1,*/DEBUG_PRINT( "Received server connection IDs: %d (Info) - %d (Data) - %d(EMG)", infoServerConnectionID, dataServerConnectionID, emgServerConnectionID );
  
  kv_init( infoClientsList );
  kv_init( dataClientsList );
  kv_init( emgClientsList );
  
  kv_init( networkAxesList );
  
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
        AsyncIPNetwork_SetMessageLength( infoServerConnectionID, infoMessageLength );
      
        DEBUG_PRINT( "info message length: %u", infoMessageLength );
      
        size_t dataMessageLength = 1 + sharedRobotsNumber * DATA_BLOCK_SIZE;
        AsyncIPNetwork_SetMessageLength( dataServerConnectionID, dataMessageLength );
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
            size_t sharedDoFsNumber = ConfigParser.GetListSize( robotConfigFileID, searchPath );
            for( size_t dofIndex = 0; dofIndex < sharedDoFsNumber; dofIndex++ )
            {
              NetworkAxis newNetworkAxis;
              sprintf( searchPath, "%s.%lu.dofs.%lu", CONFIG_KEY, sharedRobotIndex, dofIndex );
              sprintf( searchPath, "%s-%s", deviceName, ConfigParser.GetStringValue( configFileID, searchPath, "" ) );
              newNetworkAxis.sharedAxis = SHMControl.InitData( searchPath, SHM_CONTROL_OUT );
              if( sharedAxisData != NULL )
              {
                kv_push( NetworkAxis, networkAxesList, newNetworkAxis );
            
                if( strlen( axesInfoString ) > 0 ) strcat( axesInfoString, "|" );
                snprintf( &(axesInfoString[ strlen( axesInfoString ) ]), IP_MAX_MESSAGE_LENGTH, "%u:%s", kv_size( networkAxesList ) - 1, deviceName );
            
                DEBUG_PRINT( "got network axis %u", kv_size( networkAxesList ) - 1 );
              }
            }
            
          
            SHMController sharedAxis = SHMAxisControl.InitData( deviceName, SHM_CONTROL_OUT );
            
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
  
  for( size_t infoClientIndex = 0; infoClientIndex < kv_size( infoClientsList ); infoClientIndex++ )
    AsyncIPNetwork.CloseConnection( kv_A( infoClientsList, infoClientIndex ) );
  
  /*DEBUG_EVENT( 1,*/DEBUG_PRINT( "info server %d clients closed", infoServerConnectionID );
  
  for( size_t dataClientIndex = 0; dataClientIndex < kv_size( dataClientsList ); dataClientIndex++ )
    AsyncIPNetwork.CloseConnection( kv_A( dataClientsList, dataClientIndex ) );
  
  /*DEBUG_EVENT( 2,*/DEBUG_PRINT( "data server %d clients closed", dataServerConnectionID );
  
  AsyncIPNetwork.CloseConnection( infoServerConnectionID );
  DEBUG_EVENT( 3, "info server %d closed", infoServerConnectionID );
  
  AsyncIPNetwork.CloseConnection( dataServerConnectionID );
  DEBUG_EVENT( 4, "data server %d closed", dataServerConnectionID );
  
  kv_destroy( infoClientsList );
  DEBUG_EVENT( 5, "info clients list %p destroyed", infoClientsList );
  
  kv_destroy( dataClientsList );
  DEBUG_EVENT( 6, "data clients list %d destroyed", dataClientsList );
  
  for( size_t controllerID = 0; controllerID < kv_size( networkAxesList ); controllerID++ )
  {
    //TrajectoryPlanner_End( networkAxesList[ controllerID ].trajectoryPlanner );
    SHMAxisControl.EndData( kv_A( networkAxesList, controllerID ).sharedData );
  }
  kv_destroy( networkAxesList );
  
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "RobRehab Network ended on thread %x", THREAD_ID );
}

static void UpdateClientInfo( int );
static void UpdateClientData( int );

void RobRehabNetwork_Update()
{
  static int newInfoClientID, newDataClientID;
  
  DEBUG_UPDATE( "updating connections on thread %x", THREAD_ID );
  
  if( (newInfoClientID = AsyncIPNetwork_GetClient( infoServerConnectionID )) != -1 )
  {
    /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "new info client found: %d", newInfoClientID );
    AsyncIPNetwork_WriteMessage( newInfoClientID, axesInfoString );
    kv_push( int, infoClientsList, newInfoClientID );
  }
  
  if( (newDataClientID = AsyncIPNetwork_GetClient( dataServerConnectionID )) != -1 )
  {
    /*DEBUG_EVENT( 1,*/DEBUG_PRINT( "new data client found: %d", newDataClientID );
    kv_push( int, dataClientsList, newDataClientID );
  }
  
  for( size_t clientIndex = 0; clientIndex < kv_size( infoClientsList ); clientIndex++ )
    UpdateClientInfo( kv_A( infoClientsList, clientIndex ) );
  
  for( size_t clientIndex = 0; clientIndex < kv_size( dataClientsList ); clientIndex++ )
    UpdateClientData( kv_A( dataClientsList, clientIndex ) );
}

static void UpdateClientInfo( int clientID )
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
      if( controllerID < 0 || controllerID >= kv_size( networkAxesList ) ) continue;
      
      NetworkAxis* networkAxis = &(kv_A( networkAxesList, controllerID ));
      
      uint8_t command = (uint8_t) *(messageIn++);
      SHMAxisControl.SetByteValue( networkAxis->sharedData, command );
      
      DEBUG_UPDATE( "received axis %u command: %u", controllerID, command );
      
      uint8_t state = SHMAxisControl.GetByteValue( networkAxis->sharedData, SHM_CONTROL_REMOVE );
      if( state != SHM_CONTROL_NULL_BYTE )
      {
        messageOut[ 0 ]++;
        messageOut[ stateByteIndex++ ] = (char) controllerID;
        messageOut[ stateByteIndex++ ] = (char) state;
      }
    }
  
    if( messageOut[ 0 ] > 0 ) 
    {
      DEBUG_UPDATE( "sending message %s to client %d (%u bytes)", messageOut, clientID, stateByteIndex );
      AsyncIPNetwork_WriteMessage( clientID, messageOut );
    }
  }
}

static void UpdateClientData( int clientID )
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
      
      //TrajectoryPlanner_SetCurve( networkAxis->trajectoryPlanner, setpoint, setpointDerivative, setpointsInterval );
    }
  }
  
  memset( messageOut, 0, IP_MAX_MESSAGE_LENGTH * sizeof(char) );
  size_t measureByteIndex = 1;
  for( size_t controllerID = 0; controllerID < kv_size( networkAxesList ); controllerID++ )
  {
    NetworkAxis* networkAxis = &(kv_A( networkAxesList, controllerID ));
    
    messageOut[ 0 ]++;
    messageOut[ measureByteIndex++ ] = (uint8_t) controllerID;
    
    SHMAxisControl.GetNumericValuesList( networkAxis->sharedData, (float*) ( messageOut + measureByteIndex ), SHM_CONTROL_PEEK );
    measureByteIndex += sizeof(float) * SHM_CONTROL_FLOATS_NUMBER;
  }
  
  if( messageOut[ 0 ] > 0 ) 
  {
    DEBUG_UPDATE( "sending message %s to client %d (%u bytes)", messageOut, clientID, measureByteIndex );
    AsyncIPNetwork_WriteMessage( clientID, messageOut );
  }
}

#endif //ROBREHAB_NETWORK_H
