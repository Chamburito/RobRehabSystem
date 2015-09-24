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
#include "Common.h"

#include "interface.h"

//#include "spline3_interpolation.h"
#include "shm_axis_control.h"
//#include "network_axis.h"

#include "file_parsing/json_parser.h"

#include "klib/kvec.h"

#include "debug/async_debug.h"

static int infoServerConnectionID;
static int dataServerConnectionID;

static kvec_t( int ) infoClientsList;
static char* infoMessageOut;
static size_t infoMessageLength;
const size_t INFO_BLOCK_SIZE = 2;

static kvec_t( int ) dataClientsList;
static char* dataMessageOut;
static size_t dataMessageLength;
const size_t DATA_BLOCK_SIZE = 1 + SHM_CONTROL_FLOATS_NUMBER * sizeof(float);

typedef struct _NetworkAxis
{
  int clientID;
  SHMAxisController sharedController;
  //TrajectoryPlanner* trajectoryPlanner;
} 
NetworkAxis;

static kvec_t( NetworkAxis ) networkAxesList;

static char axesInfoString[ IP_MAX_MESSAGE_LENGTH ] = ""; // String containing used axes names

#define NAMESPACE RobRehabNetwork

#define NAMESPACE_FUNCTIONS( FUNCTION_INIT, namespace ) \
        FUNCTION_INIT( int, namespace, Init, void ) \
        FUNCTION_INIT( void, namespace, End, void ) \
        FUNCTION_INIT( void, namespace, Update, void )

NAMESPACE_FUNCTIONS( INIT_NAMESPACE_FILE, NAMESPACE )

const struct { NAMESPACE_FUNCTIONS( INIT_NAMESPACE_POINTER, NAMESPACE ) } NAMESPACE = { NAMESPACE_FUNCTIONS( INIT_NAMESPACE_STRUCT, NAMESPACE ) };

#undef NAMESPACE_FUNCTIONS
#undef NAMESPACE

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
  
  /*DEBUG_EVENT( 1,*/DEBUG_PRINT( "Received server connection IDs: %d (State), %d (Data)", infoServerConnectionID, dataServerConnectionID );
  
  kv_init( infoClientsList );
  kv_init( dataClientsList );
  
  kv_init( networkAxesList );
  
  SET_PATH( "../config/" );
  
  FileParser parser = JSONParser;
  int configFileID = parser.LoadFile( "shared_axes" );
  if( configFileID != -1 )
  {
    DEBUG_PRINT( "found configuration file %s.json", "shared_axes" );
    if( parser.HasKey( configFileID, "axes" ) )
    {
      size_t sharedAxesNumber = parser.GetListSize( configFileID, "axes" );
      
      DEBUG_PRINT( "shared axes list size: %u", sharedAxesNumber );
      
      infoMessageLength = sharedAxesNumber * INFO_BLOCK_SIZE; 
      infoMessageOut = (char*) calloc( infoMessageLength, sizeof(char) );
      AsyncIPConnection_SetMessageLength( infoServerConnectionID, infoMessageLength );
      
      DEBUG_PRINT( "info message length: %u", infoMessageLength );
      
      dataMessageLength = sharedAxesNumber * DATA_BLOCK_SIZE;
      dataMessageOut = (char*) calloc( dataMessageLength, sizeof(char) );
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
          
          SHMAxisController sharedAxisController = SHMAxisControl.InitController( deviceName );
          if( sharedAxisController != NULL )
          {
            NetworkAxis newAxis = { .sharedController = sharedAxisController, .clientID = -1 /*, .trajectoryPlanner = TrajectoryPlanner_Init();*/ };
            
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
  
  DEBUG_EVENT( 0, "RobRehab Network initialized on thread %x", THREAD_ID );
  
  return 0;
}

void RobRehabNetwork_End()
{
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "ending RobRehab Network on thread %x", THREAD_ID );
  
  for( size_t infoClientIndex = 0; infoClientIndex < kv_size( infoClientsList ); infoClientIndex++ )
    AsyncIPConnection_Close( kv_A( infoClientsList, infoClientIndex ) );
  
  DEBUG_PRINT( "info server %d clients closed", infoServerConnectionID );
  
  for( size_t dataClientIndex = 0; dataClientIndex < kv_size( dataClientsList ); dataClientIndex++ )
    AsyncIPConnection_Close( kv_A( dataClientsList, dataClientIndex ) );
  
  DEBUG_PRINT( "data server %d clients closed", dataServerConnectionID );
  
  AsyncIPConnection_Close( infoServerConnectionID );
  DEBUG_PRINT( "info server %d closed", infoServerConnectionID );
  
  AsyncIPConnection_Close( dataServerConnectionID );
  DEBUG_PRINT( "data server %d closed", dataServerConnectionID );
  
  kv_destroy( infoClientsList );
  //DEBUG_PRINT( "info clients list %p destroyed", infoClientsList );
  
  kv_destroy( dataClientsList );
  //DEBUG_PRINT( "data clients list %d destroyed", dataClientsList );
  
  for( size_t controllerID = 0; controllerID < kv_size( networkAxesList ); controllerID++ )
  {
    //TrajectoryPlanner_End( networkAxesList[ controllerID ].trajectoryPlanner );
    SHMAxisControl.EndController( kv_A( networkAxesList, controllerID ).sharedController );
  }
  kv_destroy( networkAxesList );
  
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "RobRehab Network ended on thread %x", THREAD_ID );
}

static void UpdateClientInfo( int );
static void UpdateClientData( int );

void RobRehabNetwork_Update()
{
  static int newInfoClientID, newDataClientID;
  
  //DEBUG_PRINT( "updating info server %d", infoServerConnectionID );
  
  if( (newInfoClientID = AsyncIPConnection_GetClient( infoServerConnectionID )) != -1 )
  {
    AsyncIPConnection_WriteMessage( newInfoClientID, axesInfoString );
    kv_push( int, infoClientsList, newInfoClientID );
  }
  
  //DEBUG_PRINT( "updating data server %d", dataServerConnectionID );
  
  if( (newDataClientID = AsyncIPConnection_GetClient( dataServerConnectionID )) != -1 )
    kv_push( int, dataClientsList, newDataClientID );
  
  for( size_t clientIndex = 0; clientIndex < kv_size( infoClientsList ); clientIndex++ )
    UpdateClientInfo( kv_A( infoClientsList, clientIndex ) );
  
  for( size_t clientIndex = 0; clientIndex < kv_size( dataClientsList ); clientIndex++ )
    UpdateClientData( kv_A( dataClientsList, clientIndex ) );
}

static void UpdateClientInfo( int clientID )
{
  char* messageIn = AsyncIPConnection_ReadMessage( clientID );
  
  if( messageIn != NULL ) 
  {
    /*DEBUG_UPDATE*/DEBUG_PRINT( "received input message: %s", messageIn );
    
    memset( infoMessageOut, 0, infoMessageLength * sizeof(char) );
    size_t stateBytesCount = 0;
    
    uint8_t commandBlocksNumber = (uint8_t) *(messageIn++);
    for( uint8_t commandBlockIndex = 0; commandBlockIndex < commandBlocksNumber; commandBlockIndex++ )
    {
      uint8_t controllerID = (uint8_t) *(messageIn++);
      if( controllerID < 0 || controllerID >= kv_size( networkAxesList ) ) continue;
      
      /*DEBUG_UPDATE*/DEBUG_PRINT( "received axis %u command data", controllerID );
      
      NetworkAxis* networkAxis = &(kv_A( networkAxesList, controllerID ));
      
      uint8_t command = (uint8_t) *(messageIn++);
      SHMAxisControl.SetByteValue( networkAxis->sharedController, SHM_CONTROL_IN, command );
      
      uint8_t state = SHMAxisControl.GetByteValue( networkAxis->sharedController, SHM_CONTROL_OUT, SHM_REMOVE );
      infoMessageOut[ stateBytesCount++ ] = (char) controllerID;
      infoMessageOut[ stateBytesCount++ ] = (char) state;
    }
  
    if( stateBytesCount > 0 ) 
    {
      /*DEBUG_UPDATE*/DEBUG_PRINT( "sending message %s to client %d (%u bytes)", infoMessageOut, clientID, stateBytesCount );
      AsyncIPConnection_WriteMessage( clientID, infoMessageOut );
    }
  }
}

static void UpdateClientData( int clientID )
{
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
      
      DEBUG_UPDATE( "receiving axis %u setpoints", controllerID );
      
      uint8_t dataMask = (uint8_t) messageIn[ 1 ];
      
      float* setpointsList = (float*) &(messageIn[ 2 ]);
      SHMAxisControl.SetNumericValuesList( networkAxis->sharedController, SHM_CONTROL_IN, setpointsList, dataMask );
      
      messageIn += DATA_BLOCK_SIZE;
      //DEBUG_PRINT( "received setpoint: %f", setpoint );
      
      //TrajectoryPlanner_SetCurve( networkAxis->trajectoryPlanner, setpoint, setpointDerivative, setpointsInterval );
    }
  }
  
  memset( dataMessageOut, 0, dataMessageLength * sizeof(char) );
  size_t measureBytesCount = 0;
  for( size_t controllerID = 0; controllerID < kv_size( networkAxesList ); controllerID++ )
  {
    NetworkAxis* networkAxis = &(kv_A( networkAxesList, controllerID ));
    
    uint8_t dataMask;
    float* measuresList = SHMAxisControl.GetNumericValuesList( networkAxis->sharedController, SHM_CONTROL_OUT, &dataMask, SHM_PEEK );
    if( measuresList != NULL )
    {
      dataMessageOut[ measureBytesCount++ ] = (uint8_t) controllerID;
      dataMessageOut[ measureBytesCount++ ] = dataMask;
      memcpy( dataMessageOut + measureBytesCount, measuresList, sizeof(float) * SHM_CONTROL_FLOATS_NUMBER );
      measureBytesCount += sizeof(float) * SHM_CONTROL_FLOATS_NUMBER;
    }
  }
  
  if( measureBytesCount > 0 ) 
  {
    DEBUG_UPDATE( "sending message %s to client %d (%u bytes)", dataMessageOut, clientID, measureBytesCount );
    AsyncIPConnection_WriteMessage( clientID, dataMessageOut );
  }
}

#endif //ROBREHAB_NETWORK_H
