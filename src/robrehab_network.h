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

//#include "spline3_interpolation.h"
#include "shm_axis_control.h"
//#include "network_axis.h"

#include "file_parsing/json_parser.h"

#include "klib/kvec.h"

#include "debug/async_debug.h"

const unsigned long UPDATE_INTERVAL_MS = 5;

static int infoServerConnectionID;
static int dataServerConnectionID;

static kvec_t( int ) infoClientsList;
const size_t INFO_BLOCK_SIZE = 2;

static kvec_t( int ) dataClientsList;
const size_t DATA_BLOCK_SIZE = 1 + SHM_CONTROL_FLOATS_NUMBER * sizeof(float);

typedef struct _NetworkAxis
{
  int clientID;
  SHMAxis sharedData;
  //TrajectoryPlanner* trajectoryPlanner;
} 
NetworkAxis;

static kvec_t( NetworkAxis ) networkAxesList;

static char axesInfoString[ IP_MAX_MESSAGE_LENGTH ] = ""; // String containing used axes names

#define SUBSYSTEM RobRehabNetwork

#define ROBREHAB_NETWORK_FUNCTIONS( namespace, function_init ) \
        function_init( int, namespace, Init, void ) \
        function_init( void, namespace, End, void ) \
        function_init( void, namespace, Update, void )

INIT_NAMESPACE_INTERFACE( SUBSYSTEM, ROBREHAB_NETWORK_FUNCTIONS )

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
  
  /*DEBUG_EVENT( 1,*/DEBUG_PRINT( "Received server connection IDs: %d (Info) - %d (Data)", infoServerConnectionID, dataServerConnectionID );
  
  kv_init( infoClientsList );
  kv_init( dataClientsList );
  
  kv_init( networkAxesList );
  
  SET_PATH( "config/" );
  
  FileParserOperations parser = JSONParser;
  int configFileID = parser.LoadFile( "shared_axes" );
  if( configFileID != -1 )
  {
    if( parser.HasKey( configFileID, "axes" ) )
    {
      size_t sharedAxesNumber = parser.GetListSize( configFileID, "axes" );
      
      DEBUG_PRINT( "shared axes list size: %u", sharedAxesNumber );
      
      size_t infoMessageLength = 1 + sharedAxesNumber * INFO_BLOCK_SIZE; 
      AsyncIPConnection_SetMessageLength( infoServerConnectionID, infoMessageLength );
      
      DEBUG_PRINT( "info message length: %u", infoMessageLength );
      
      size_t dataMessageLength = 1 + sharedAxesNumber * DATA_BLOCK_SIZE;
      AsyncIPConnection_SetMessageLength( dataServerConnectionID, dataMessageLength );
      
      DEBUG_PRINT( "data message length: %u", dataMessageLength );
      
      char searchPath[ FILE_PARSER_MAX_PATH_LENGTH ];
      for( size_t sharedAxisDataIndex = 0; sharedAxisDataIndex < sharedAxesNumber; sharedAxisDataIndex++ )
      {
        sprintf( searchPath, "axes.%u", sharedAxisDataIndex );
        char* deviceName = parser.GetStringValue( configFileID, searchPath );
        
        if( deviceName != NULL )
        {
          DEBUG_PRINT( "found shared axis %s", deviceName );
          
          SHMAxis sharedAxisData = SHMAxisControl.InitData( deviceName, SHM_CONTROL_OUT );
          if( sharedAxisData != NULL )
          {
            NetworkAxis newAxis = { .sharedData = sharedAxisData, .clientID = -1 /*, .trajectoryPlanner = TrajectoryPlanner_Init()*/ };
            
            kv_push( NetworkAxis, networkAxesList, newAxis );
            
            if( strlen( axesInfoString ) > 0 ) strcat( axesInfoString, "|" );
            snprintf( &(axesInfoString[ strlen( axesInfoString ) ]), IP_MAX_MESSAGE_LENGTH, "%u:%s", kv_size( networkAxesList ) - 1, deviceName );
            
            DEBUG_PRINT( "got network axis %u", kv_size( networkAxesList ) - 1 );
          }
        }
      }
    }
    
    parser.UnloadFile( configFileID );
  }
  
  SET_PATH( ".." );
  
  DEBUG_EVENT( 0, "RobRehab Network initialized on thread %x", THREAD_ID );
  
  return 0;
}

void RobRehabNetwork_End()
{
  DEBUG_EVENT( 0, "ending RobRehab Network on thread %x", THREAD_ID );
  
  for( size_t infoClientIndex = 0; infoClientIndex < kv_size( infoClientsList ); infoClientIndex++ )
    AsyncIPConnection_Close( kv_A( infoClientsList, infoClientIndex ) );
  
  /*DEBUG_EVENT( 1,*/DEBUG_PRINT( "info server %d clients closed", infoServerConnectionID );
  
  for( size_t dataClientIndex = 0; dataClientIndex < kv_size( dataClientsList ); dataClientIndex++ )
    AsyncIPConnection_Close( kv_A( dataClientsList, dataClientIndex ) );
  
  /*DEBUG_EVENT( 2,*/DEBUG_PRINT( "data server %d clients closed", dataServerConnectionID );
  
  AsyncIPConnection_Close( infoServerConnectionID );
  DEBUG_EVENT( 3, "info server %d closed", infoServerConnectionID );
  
  AsyncIPConnection_Close( dataServerConnectionID );
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
  
  if( (newInfoClientID = AsyncIPConnection_GetClient( infoServerConnectionID )) != -1 )
  {
    /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "new info client found: %d", newInfoClientID );
    AsyncIPConnection_WriteMessage( newInfoClientID, axesInfoString );
    kv_push( int, infoClientsList, newInfoClientID );
  }
  
  //DEBUG_PRINT( "updating data server %d", dataServerConnectionID );
  
  //static float measuresList[ SHM_CONTROL_FLOATS_NUMBER ];
  /*if( SHMAxisControl.GetNumericValuesList( kv_A( networkAxesList, 0 ).sharedData, measuresList, SHM_PEEK ) )
  {
    DEBUG_PRINT( "measures: p: %.3f - v: %.3f - f: %.3f", measuresList[ SHM_CONTROL_POSITION ], measuresList[ SHM_CONTROL_VELOCITY ], measuresList[ SHM_CONTROL_FORCE ] );
  }*/
  
  //SHMAxisControl.SetNumericValuesList( kv_A( networkAxesList, 0 ).sharedData, measuresList, 15 );
  
  if( (newDataClientID = AsyncIPConnection_GetClient( dataServerConnectionID )) != -1 )
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
  
  char* messageIn = AsyncIPConnection_ReadMessage( clientID );
  
  if( messageIn != NULL ) 
  {
    ///*DEBUG_UPDATE*/DEBUG_PRINT( "received input message: %s", messageIn );
    
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
      
      uint8_t state;
      if( SHMAxisControl.GetByteValue( networkAxis->sharedData, &state, SHM_REMOVE ) )
      {
        messageOut[ 0 ]++;
        
        messageOut[ stateByteIndex++ ] = (char) controllerID;
        messageOut[ stateByteIndex++ ] = (char) state;
      }
    }
  
    if( messageOut[ 0 ] > 0 ) 
    {
      DEBUG_UPDATE( "sending message %s to client %d (%u bytes)", messageOut, clientID, stateByteIndex );
      AsyncIPConnection_WriteMessage( clientID, messageOut );
    }
  }
}

static void UpdateClientData( int clientID )
{
  static char messageOut[ IP_MAX_MESSAGE_LENGTH ];
  
  char* messageIn = AsyncIPConnection_ReadMessage( clientID );
  
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
    
    SHMAxisControl.GetNumericValuesList( networkAxis->sharedData, (float*) ( messageOut + measureByteIndex ), SHM_PEEK );
    measureByteIndex += sizeof(float) * SHM_CONTROL_FLOATS_NUMBER;
  }
  
  if( messageOut[ 0 ] > 0 ) 
  {
    DEBUG_UPDATE( "sending message %s to client %d (%u bytes)", messageOut, clientID, measureByteIndex );
    AsyncIPConnection_WriteMessage( clientID, messageOut );
  }
}

#endif //ROBREHAB_NETWORK_H
