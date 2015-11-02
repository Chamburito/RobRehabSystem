/////////////////////////////////////////////////////////////////////////////////////
///// Real-Time platform library for creation and handling of IP sockets        /////
///// connections as server or client, using TCP or UDP protocols               /////
/////////////////////////////////////////////////////////////////////////////////////

#ifndef ASYNC_IP_CONNECTION_H
#define ASYNC_IP_CONNECTION_H

#include "threads/thread_safe_data.h"

#include "interface.h"

#include "cvidef.h"
#include "debug/async_debug.h"

#include "klib/khash.h"

#include <tcpsupp.h>
#include <udpsupp.h>
#include <utility.h>
#include <userint.h>

//////////////////////////////////////////////////////////////////////////
/////                         DATA STRUCTURES                        /////
//////////////////////////////////////////////////////////////////////////

#define IP_MAX_MESSAGE_LENGTH CMT_MAX_MESSAGE_BUF_SIZE  // 256

const int IP_CONNECTION_INVALID_ID = -1;

const int QUEUE_MAX_ITEMS = 10;

const size_t IP_HOST_LENGTH = 40;
const size_t IP_PORT_LENGTH = 6;
const size_t IP_ADDRESS_LENGTH = IP_HOST_LENGTH + IP_PORT_LENGTH;

enum Property { DISCONNECTED = 0x00, CLIENT = 0x01, SERVER = 0x02, LISTENER = 0x03, NETWORK_ROLE_MASK = 0x0f, TCP = 0x10, UDP = 0x20, PROTOCOL_MASK = 0xf0 };

// Generic structure to store methods and data of any connection type handled by the library
typedef struct _AsyncConnectionData AsyncConnectionData;

typedef AsyncConnectionData* AsyncConnection;

KHASH_MAP_INIT_INT( IPInt, AsyncConnection )

struct _AsyncConnectionData
{
  unsigned int handle;
  uint8_t protocol, networkRole;
  uint16_t messageLength;
  Thread callbackThread;
  int (*ref_SendMessage)( AsyncConnection, const char* );
  struct
  {
    unsigned int port;
    char host[ IP_HOST_LENGTH ];
  } address;
  ThreadSafeQueue readQueue;
  union
  {
    khash_t( IPInt )* clientsList;
    AsyncConnection ref_server;
  };
};

// Internal (private) list of asyncronous connections created (accessible only by ID)
static khash_t( IPInt )* globalConnectionsList = NULL;


/////////////////////////////////////////////////////////////////////////////
/////                             INTERFACE                             /////
/////////////////////////////////////////////////////////////////////////////

#define ASYNC_IP_CONNECTION_FUNCTIONS( namespace, function_init ) \
        function_init( char*, namespace, GetAddress, int ) \
        function_init( size_t, namespace, GetActivesNumber, void ) \
        function_init( size_t, namespace, GetClientsNumber, int ) \
        function_init( size_t, namespace, SetMessageLength, int, size_t ) \
        function_init( int, namespace, OpenConnection, const char*, const char*, uint8_t ) \
        function_init( void, namespace, CloseConnection, int ) \
        function_init( char*, namespace, ReadMessage, int ) \
        function_init( int, namespace, WriteMessage, int, const char* ) \
        function_init( int, namespace, GetClient, int )

INIT_NAMESPACE_INTERFACE( AsyncIPNetwork, ASYNC_IP_CONNECTION_FUNCTIONS )

/////////////////////////////////////////////////////////////////////////////
/////                         NETWORK UTILITIES                         /////
/////////////////////////////////////////////////////////////////////////////

// Utility method to request an address (host and port) string for a connection structure
static char* GetAddress( AsyncConnection connection )
{
  static char address_string[ IP_ADDRESS_LENGTH ];
  
  DEBUG_PRINT( "getting address string for connection handle: %u", connection->handle );
  
  sprintf( address_string, "%s/%u", connection->address.host, connection->address.port );
  
  return address_string;
}

// Returns the address string relative to the given connection ID
char* AsyncIPNetwork_GetAddress( int connectionID )
{
  khint_t connectionIndex = kh_get( IPInt, globalConnectionsList, (khint_t) connectionID );
  if( connectionIndex == kh_end( globalConnectionsList ) )
  {
    ERROR_EVENT( "invalid connection ID: %d", connectionID );
    return NULL;
  }
  
  return GetAddress( kh_value( globalConnectionsList, /*(khint_t) connectionID*/connectionIndex ) );
}

// Returns total number of active connections
inline size_t AsyncIPNetwork_GetActivesNumber()
{
  return kh_size( globalConnectionsList );
}

// Returns number of active clients for a connection 
size_t AsyncIPNetwork_GetClientsNumber( int connectionID )
{
  khint_t connectionIndex = kh_get( IPInt, globalConnectionsList, (khint_t) connectionID );
  if( connectionIndex == kh_end( globalConnectionsList ) )
  {
    ERROR_EVENT( "invalid connection ID: %d", connectionID );
    return 0;
  }
  
  AsyncConnection connection = kh_value( globalConnectionsList, /*(khint_t) connectionID*/connectionIndex );
  if( connection == NULL ) return 0;
  
  if( connection->networkRole == SERVER )
    return kh_size( connection->clientsList );
  
  return 1;
}

inline size_t AsyncIPNetwork_SetMessageLength( int connectionID, size_t messageLength )
{
  khint_t connectionIndex = kh_get( IPInt, globalConnectionsList, (khint_t) connectionID );
  if( connectionIndex == kh_end( globalConnectionsList ) )
  {
    ERROR_EVENT( "invalid connection ID: %d", connectionID );
    return 0;
  }
  
  AsyncConnection connection = kh_value( globalConnectionsList, /*(khint_t) connectionID*/connectionIndex );
  if( connection == NULL ) return 0;
  
  if( connection->networkRole == SERVER ) return 0;
  
  connection->messageLength = (uint16_t) messageLength;
  
  return (size_t) connection->messageLength;
}

//////////////////////////////////////////////////////////////////////////////////
/////                             INITIALIZATION                             /////
//////////////////////////////////////////////////////////////////////////////////

// Forward Declaration
static int CVICALLBACK AcceptTCPClient( unsigned, int, int, void* );
static int CVICALLBACK AcceptUDPClient( unsigned, int, int, void* );
static int CVICALLBACK ReceiveTCPMessage( unsigned, int, int, void* );
static int CVICALLBACK ReceiveUDPMessage( unsigned, int, int, void* );

static int SendTCPMessage( AsyncConnection, const char* );
static int SendUDPMessage( AsyncConnection, const char* );
static int SendMessageToAll( AsyncConnection, const char* );

static void CloseConnection( AsyncConnection );

// Asyncronous callback for ip connection setup and event processing
static void* AsyncConnect( void* connectionData )
{
  AsyncConnection connection = (AsyncConnection) connectionData;
  
  static int statusCode;
  
  if( connection->protocol == TCP )
  {
    if( connection->networkRole == SERVER )
    {
      statusCode = RegisterTCPServer( connection->address.port, AcceptTCPClient, connection );
      connection->handle = connection->address.port;
    }
    else
      statusCode = ConnectToTCPServer( &(connection->handle), connection->address.port, connection->address.host, ReceiveTCPMessage, connection, 1000 );
  }
  else
  {
    if( connection->networkRole == SERVER )
      statusCode = CreateUDPChannelConfig( connection->address.port, UDP_ANY_ADDRESS, 0, AcceptUDPClient, connection, &(connection->handle) );
    else
      statusCode = CreateUDPChannelConfig( UDP_ANY_LOCAL_PORT, UDP_ANY_ADDRESS, 0, ReceiveUDPMessage, connection, &(connection->handle) );
  }
  
  if( statusCode < 0 )
  {
    if( connection->protocol == TCP ) ERROR_EVENT( "%s: %s", GetTCPErrorString( statusCode ), GetTCPSystemErrorString() );
    else ERROR_EVENT( "%s", GetUDPErrorString( statusCode ) );
    
    connection->networkRole = DISCONNECTED;
  }
  else
    DEBUG_EVENT( 0, "starting to process system events for connection %u on thread %x", connection->handle, THREAD_ID );
  
  while( connection->networkRole != DISCONNECTED ) 
  {
    ProcessSystemEvents();
    Timing_Delay( 1 );
  }
  
  DEBUG_PRINT( "connection handle %u closed. exiting event loop on thread %x", connection->handle, THREAD_ID );
  
  return NULL;
}

// Generate connection data structure based on input arguments
static khint_t AddConnection( unsigned int connectionHandle, unsigned int addressPort, const char* addressHost, uint8_t connectionType )
{
  AsyncConnection newConnection = (AsyncConnection) malloc( sizeof(AsyncConnectionData) );
  
  newConnection->handle = connectionHandle;
  newConnection->protocol = (connectionType & PROTOCOL_MASK);  
  newConnection->networkRole = (connectionType & NETWORK_ROLE_MASK);
  
  newConnection->messageLength = IP_MAX_MESSAGE_LENGTH;
  
  newConnection->callbackThread = INVALID_THREAD_HANDLE;
  
  newConnection->address.port = addressPort;
  strncpy( newConnection->address.host, addressHost, IP_HOST_LENGTH );
  
  if( newConnection->networkRole == SERVER )
  {
    newConnection->clientsList = NULL;
    newConnection->readQueue = ThreadSafeQueues.Create( QUEUE_MAX_ITEMS, sizeof(int) );
    newConnection->ref_SendMessage = SendMessageToAll;
    newConnection->clientsList = kh_init( IPInt );
  }
  else
  {
    if( newConnection->protocol == TCP ) newConnection->ref_SendMessage = SendTCPMessage;
    else newConnection->ref_SendMessage = SendUDPMessage;
    
    newConnection->readQueue = ThreadSafeQueues.Create( QUEUE_MAX_ITEMS, IP_MAX_MESSAGE_LENGTH );
    newConnection->ref_server = NULL;
  }
  
  int insertionStatus;
  int connectionKey = kh_str_hash_func( addressHost ) + addressPort + newConnection->protocol;
  khint_t newConnectionIndex = kh_put( IPInt, globalConnectionsList, connectionKey, &insertionStatus );
  kh_value( globalConnectionsList, newConnectionIndex ) = newConnection;
  
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "%u active connections listed", kh_size( globalConnectionsList ) );
  
  return newConnectionIndex;
}

// Generic method for opening a new socket and providing a corresponding Connection structure for use
const char* ANY_HOST = "0.0.0.0";
int AsyncIPNetwork_OpenConnection( const char* host, const char* port, uint8_t protocol )
{
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "trying to open %s connection for host %s and port %s", ( protocol == TCP ) ? "TCP" : "UDP", ( host != NULL ) ? host : ANY_HOST, port );
  
  // Assure that the port number is in the Dynamic/Private range (49152-65535)
  unsigned int portNumber = (unsigned int) strtoul( port, NULL, 0 );
  if( portNumber < 49152 || portNumber > 65535 )
  {
    ERROR_EVENT( "invalid port number value: %s", port );
    return IP_CONNECTION_INVALID_ID;
  }

  if( ( protocol != TCP ) && ( protocol != UDP ) )
  {
    ERROR_EVENT( "invalid protocol option: %x", protocol );
    return IP_CONNECTION_INVALID_ID;
  }
  
  if( globalConnectionsList == NULL ) globalConnectionsList = kh_init( IPInt );
  
  int connectionKey = kh_str_hash_func( ( host != NULL ) ? host : ANY_HOST ) + portNumber + protocol;
  khint_t newConnectionIndex = kh_get( IPInt, globalConnectionsList, connectionKey );
  if( newConnectionIndex == kh_end( globalConnectionsList ) )
  {
    if( host == NULL ) newConnectionIndex = AddConnection( 0, portNumber, ANY_HOST, SERVER | protocol );
    else newConnectionIndex = AddConnection( 0, portNumber, host, CLIENT | protocol );
    AsyncConnection newConnection = kh_value( globalConnectionsList, newConnectionIndex );

    newConnection->callbackThread = Threading.StartThread( AsyncConnect, newConnection, THREAD_JOINABLE );
    (void) Threading.WaitExit( newConnection->callbackThread, 1000 );

    if( newConnection->networkRole == DISCONNECTED )
    {
      newConnection->callbackThread = INVALID_THREAD_HANDLE;
      AsyncIPNetwork_CloseConnection( newConnectionIndex );
      return IP_CONNECTION_INVALID_ID;
    }

    /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "new %s %s connection opened: %s address: %s", ( newConnection->protocol == TCP ) ? "TCP" : "UDP",
                                                                                   ( newConnection->networkRole == SERVER ) ? "local" : "remote",
                                                                                   ( newConnection->networkRole == SERVER ) ? "Server" : "Client",
                                                                                   GetAddress( newConnection ) );
  }
  
  return (int) kh_key( globalConnectionsList, newConnectionIndex );
}


/////////////////////////////////////////////////////////////////////////////////
/////                       ASYNCRONOUS COMMUNICATION                       /////
/////////////////////////////////////////////////////////////////////////////////

// Try to receive incoming message from the given TCP client connection and store it on its buffer
static int CVICALLBACK ReceiveTCPMessage( unsigned int connectionHandle, int eventType, int errorCode, void* connectionData )
{
  AsyncConnection connection = (AsyncConnection) connectionData;
  
  char messageBuffer[ IP_MAX_MESSAGE_LENGTH ];
  
  DEBUG_UPDATE( "called for connection handle %u on thread %x", connectionHandle, THREAD_ID );
  
  if( eventType == TCP_DISCONNECT )
  {
    DEBUG_PRINT( "sender %u disconnected", connectionHandle ); 
    CloseConnection( connection );
  }    
  else if( eventType == TCP_DATAREADY )
  {
    if( (errorCode = ClientTCPRead( connectionHandle, messageBuffer, connection->messageLength, 0 )) < 0 )
    {
      /*ERROR_EVENT*/DEBUG_PRINT( "ClientTCPRead: %s: %s", GetTCPErrorString( errorCode ), GetTCPSystemErrorString() );
      CloseConnection( connection );
    }

    ThreadSafeQueues.Enqueue( connection->readQueue, messageBuffer, QUEUE_APPEND_WAIT );
    DEBUG_UPDATE( "TCP connection handle %u received message: %s", connection->handle, messageBuffer );
  }
  
  return 0;
}

// Send given message through the given TCP connection
static int SendTCPMessage( AsyncConnection connection, const char* message )
{
  int errorCode;
  
  DEBUG_UPDATE( "called for connection handle %u on thread %x", connection->handle, THREAD_ID );
  
  if( connection->networkRole == CLIENT )
  {
    DEBUG_UPDATE( "TCP connection handle %u sending message: %s", connection->handle, message );
    
    if( connection->ref_server == NULL ) errorCode = ClientTCPWrite( connection->handle, message, connection->messageLength, 0 );
    else errorCode = ServerTCPWrite( connection->handle, message, connection->messageLength, 0 );
    
    if( errorCode < 0 )
    {
      ERROR_EVENT( "ServerTCPWrite: %s: %s", GetTCPErrorString( errorCode ), GetTCPSystemErrorString() );
      return errorCode;
    }
  }
  else
  {
    DEBUG_EVENT( 0, "connection handle %u is closed", connection->handle );
    return -1;
  }
  
  return 0;
}

// Try to receive incoming message from the given UDP client connection and store it on its buffer
static int CVICALLBACK ReceiveUDPMessage( unsigned int channel, int eventType, int errorCode, void* connectionData )
{
  AsyncConnection connection = (AsyncConnection) connectionData;
  
  char messageBuffer[ IP_MAX_MESSAGE_LENGTH ];
  
  unsigned int sourcePort;
  char sourceHost[ IP_HOST_LENGTH ];
  
  DEBUG_UPDATE( "called for connection handle %u on thread %x", channel, THREAD_ID );
  
  if( eventType == UDP_DATAREADY )
  {
    if( (errorCode = UDPRead( channel, NULL, connection->messageLength, UDP_DO_NOT_WAIT, &sourcePort, sourceHost )) < 0 )
    {
      ERROR_EVENT( "UDPRead: %s", GetUDPErrorString( errorCode ) );
      CloseConnection( connection ); 
      return -1;
    }
    
    if( connection->address.port == sourcePort )
    {
      if( strncmp( connection->address.host, sourceHost, IP_HOST_LENGTH ) == 0 )
      {
        UDPRead( channel, messageBuffer, connection->messageLength, UDP_DO_NOT_WAIT, NULL, NULL );
        ThreadSafeQueues.Enqueue( connection->readQueue, messageBuffer, QUEUE_APPEND_WAIT );
        DEBUG_UPDATE( "UDP connection handle %u (server %u) received right message: %s", connection->handle, channel, messageBuffer );
      }
    }
  }
  
  // Default return value (the received one was not destined to this connection) 
  return 0; 
}

// Send given message through the given UDP connection
static int SendUDPMessage( AsyncConnection connection, const char* message )
{
  DEBUG_UPDATE( "called for connection handle %u on thread %x", connection->handle, THREAD_ID );
  
  if( connection->networkRole == CLIENT )
  {
    unsigned int channel = ( connection->ref_server == NULL ) ? connection->handle : connection->ref_server->handle;
    
    DEBUG_UPDATE( "UDP connection handle %u (channel %u) sending message: %s", connection->handle, channel, message );
    
    int statusCode = UDPWrite( channel, connection->address.port, connection->address.host, message, connection->messageLength );
    if( statusCode < 0 )
    {
      /*ERROR_EVENT*/DEBUG_PRINT( "UDPWrite: %s", GetUDPErrorString( statusCode ) );
      return statusCode;
    }
  }
  else
  {
    DEBUG_EVENT( 0, "connection handle %u is closed", connection->handle );
    return -1;
  } 
  
  return 0;
}

// Send given message to all the clients of the given server connection
static int SendMessageToAll( AsyncConnection connection, const char* message )
{
  DEBUG_UPDATE( "called for connection handle %u on thread %x", connection->handle, THREAD_ID );
  
  int statusCode = 0;
  for( khint_t clientIndex = 0; clientIndex != kh_end( connection->clientsList ); clientIndex++ )
  {
    if( !kh_exist( connection->clientsList, clientIndex ) ) continue;
    
    AsyncConnection client = kh_value( connection->clientsList, clientIndex );
    if( client->ref_SendMessage( client, (const char*) message ) == -1 ) statusCode--;
  }
  
  return statusCode;
}

static khint_t AddClient( AsyncConnection server, const char* clientHost, unsigned int clientHandle, uint8_t protocol )
{
  khint_t newConnectionIndex = AddConnection( clientHandle, clientHandle, clientHost, protocol | CLIENT );
  AsyncConnection newClient = kh_value( globalConnectionsList, newConnectionIndex );
  newClient->ref_server = server;

  DEBUG_PRINT( "added connection ID %u", newConnectionIndex );
  
  if( server->clientsList == NULL ) server->clientsList = kh_init( IPInt );

  int insertionStatus;
  khint_t newClientIndex = kh_put( IPInt, server->clientsList, clientHandle, &insertionStatus );
  DEBUG_PRINT( "new client in list with key %u and iterator %u", clientHandle, newClientIndex );
  kh_value( server->clientsList, newClientIndex ) = newClient;

  DEBUG_PRINT( "enqueueing new connection ID %u", kh_key( globalConnectionsList, newConnectionIndex ) );
  ThreadSafeQueues.Enqueue( server->readQueue, &(kh_key( globalConnectionsList, newConnectionIndex )), QUEUE_APPEND_OVERWRITE );
  
  return newClientIndex;
}

// Waits for a remote connection to be added to the client list of the given TCP server connection
static int CVICALLBACK AcceptTCPClient( unsigned int clientHandle, int eventType, int errorCode, void* connectionData )
{
  AsyncConnection connection = (AsyncConnection) connectionData;
  
  char messageBuffer[ IP_MAX_MESSAGE_LENGTH ], clientHost[ IP_HOST_LENGTH ];
  
  /*DEBUG_UPDATE*/DEBUG_PRINT( "called for connection handle %u on server %u on thread %x", clientHandle, connection->handle, THREAD_ID );
  
  khint_t clientIndex = kh_get( IPInt, connection->clientsList, clientHandle );
  
  if( eventType == TCP_DISCONNECT && clientIndex != kh_end( connection->clientsList ) )
  {
    AsyncConnection client = kh_value( connection->clientsList, clientIndex );
    CloseConnection( client );
    /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "TCP client handle %u disconnected", client->handle );
  }
  else if( eventType == TCP_CONNECT && clientIndex == kh_end( connection->clientsList ) )
  {
    if( connection->networkRole == SERVER )
    {
      GetTCPPeerAddr( clientHandle, clientHost, IP_HOST_LENGTH );
      (void) AddClient( connection, clientHost, clientHandle, TCP );

      DEBUG_PRINT( "new TCP client from host %s with handle %u accepted !", clientHost, clientHandle );
    }
    else
      DEBUG_EVENT( 0, "TCP connection handle %u not accepting more clients", connection->handle );
  }
  else if( eventType == TCP_DATAREADY && clientIndex != kh_end( connection->clientsList ) )
  {
    AsyncConnection client = kh_value( connection->clientsList, clientIndex );
    if( (errorCode = ServerTCPRead( clientHandle, messageBuffer, connection->messageLength, 0 )) < 0 )
    {
      ERROR_EVENT( 0, "ServerTCPRead: %s: %s", GetTCPErrorString( errorCode ), GetTCPSystemErrorString() );
      //CloseConnection( client );
    }
    
    ThreadSafeQueues.Enqueue( client->readQueue, messageBuffer, QUEUE_APPEND_WAIT );
    DEBUG_UPDATE( "TCP connection handle %u (server_handle %u) received message: %s", client->handle, connection->handle, messageBuffer );
  }
  
  return errorCode;
}

// Waits for a remote connection to be added to the client list of the given UDP server connection
static int CVICALLBACK AcceptUDPClient( unsigned int channel, int eventType, int errorCode, void* connectionData )
{
  AsyncConnection connection = (AsyncConnection) connectionData;
  
  char messageBuffer[ IP_MAX_MESSAGE_LENGTH ], clientHost[ IP_HOST_LENGTH ];
  unsigned int clientPort;
  
  DEBUG_UPDATE( "called for connection handle %u on thread %x", channel, THREAD_ID );
  
  if( eventType == UDP_DATAREADY )
  {
    if( (errorCode = UDPRead( channel, messageBuffer, connection->messageLength, UDP_DO_NOT_WAIT, &(clientPort), clientHost )) < 0 )
    {
      /*ERROR_EVENT*/DEBUG_PRINT( "UDPRead: %s", GetUDPErrorString( errorCode ) );
      return -1;
    }
    
    khint_t clientIndex = kh_get( IPInt, connection->clientsList, clientPort );
    if( clientIndex == kh_end( connection->clientsList ) )
    {
      clientIndex = AddClient( connection, clientHost, clientPort, UDP );
      /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "new UDP client from host %s and port %u accepted !", clientHost, clientPort );
    }

    AsyncConnection client = kh_value( connection->clientsList, clientIndex );
    ThreadSafeQueues.Enqueue( client->readQueue, messageBuffer, QUEUE_APPEND_WAIT );
    /*DEBUG_UPDATE*/DEBUG_PRINT( "UDP connection handle %u (server handle %u) received message: %s", client->handle, client->ref_server->handle, messageBuffer );
  }
  
  return 0;
}


//////////////////////////////////////////////////////////////////////////////////
/////                        SYNCRONOUS COMMUNICATION                        /////
//////////////////////////////////////////////////////////////////////////////////

// Get (and remove) message from the beginning (oldest) of the given index corresponding read queue
// Method to be called from the main thread
char* AsyncIPNetwork_ReadMessage( int clientID )
{
  static char messageBuffer[ IP_MAX_MESSAGE_LENGTH ];
  
  khint_t clientIndex = kh_get( IPInt, globalConnectionsList, (khint_t) clientID );
  if( clientIndex == kh_end( globalConnectionsList ) ) return NULL;
  
  AsyncConnection connection = kh_value( globalConnectionsList, clientIndex );
  if( connection == NULL ) return NULL;
  
  if( connection->networkRole != CLIENT )
  {
    ERROR_EVENT( "not a client connection: ID %d", clientID );  
	  return NULL;
  }
  
  if( ThreadSafeQueues.GetItemsCount( connection->readQueue ) <= 0 )
  {
	  DEBUG_UPDATE( "no messages available for this connection: ID %d", clientID );
    return NULL;
  }
  
  ThreadSafeQueues.Dequeue( connection->readQueue, messageBuffer, QUEUE_READ_WAIT );
  
  return messageBuffer;
}

int AsyncIPNetwork_WriteMessage( int connectionID, const char* message )
{
  khint_t connectionIndex = kh_get( IPInt, globalConnectionsList, (khint_t) connectionID );
  if( connectionIndex == kh_end( globalConnectionsList ) ) return -1;
  
  AsyncConnection connection = kh_value( globalConnectionsList, connectionIndex );
  if( connection == NULL ) return -1;
  
  DEBUG_UPDATE( "writing \"%s\" to handle %d", message, connectionID );
  
  return connection->ref_SendMessage( connection, message );
}

int AsyncIPNetwork_GetClient( int serverID )
{
  static int clientID;
  
  khint_t serverIndex = kh_get( IPInt, globalConnectionsList, (khint_t) serverID );
  if( serverIndex == kh_end( globalConnectionsList ) ) return -1;
  
  AsyncConnection connection = kh_value( globalConnectionsList, serverIndex );
  
  if( connection->networkRole != SERVER )
  {
    ERROR_EVENT( "connection ID %d is not a server connection ID", serverID );  
	  return -1;
  }
  
  if( ThreadSafeQueues.GetItemsCount( connection->readQueue ) <= 0 )
  {
	  DEBUG_UPDATE( "no new clients available for server connection ID %d", serverID );
    return -1;
  }
  else if( connection->protocol == TCP ) DEBUG_PRINT( "clients available: %u", ThreadSafeQueues.GetItemsCount( connection->readQueue ) );
  
  ThreadSafeQueues.Dequeue( connection->readQueue, &clientID, QUEUE_READ_WAIT );
  
  DEBUG_PRINT( "new client connection ID %d from server connection ID %d", clientID , serverID );  
  
  return clientID; 
}


//////////////////////////////////////////////////////////////////////////////////
/////                               FINALIZING                               /////
//////////////////////////////////////////////////////////////////////////////////

// Handle proper destruction of any given connection type
static void CloseConnection( AsyncConnection connection )
{
  static int statusCode;
  
  if( connection == NULL ) return;

  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "closing connection handle %u", connection->handle );
  
  if( connection->networkRole == SERVER || connection->networkRole == LISTENER )
  {
    if( kh_size( connection->clientsList ) == 0 )
    {
      if( connection->protocol == TCP )
      {
        if( (statusCode = UnregisterTCPServer( connection->address.port )) < 0 )
          ERROR_EVENT( "UnregisterTCPServer: error closing TCP server: %s: %s", GetTCPErrorString( statusCode ), GetTCPSystemErrorString() );
        else
          DEBUG_EVENT( 0, "TCP server %u closed", connection->handle );
      }
      else
      {
        if( (statusCode = DisposeUDPChannel( connection->handle )) < 0 )
          ERROR_EVENT( "DisposeUDPChannel: %s", GetUDPErrorString( statusCode ) );
        else
          DEBUG_EVENT( 0, "UDP server %u closed", connection->handle );
      }

      kh_destroy( IPInt, connection->clientsList );
      connection->clientsList = NULL;
    }
    else
    {
      connection->networkRole = LISTENER;
      return;
    }
  }
  else
  {
    if( connection->protocol == TCP )
    {
      if( connection->ref_server == NULL ) statusCode = DisconnectFromTCPServer( connection->handle );
      else statusCode = DisconnectTCPClient( connection->handle );
      
      if( statusCode < 0 ) ERROR_EVENT( "error closing TCP client: %s: %s", GetTCPErrorString( statusCode ), GetTCPSystemErrorString() );
      else DEBUG_EVENT( 0, "TCP client %u closed", connection->handle ); 
    }
    else if( connection->ref_server == NULL )
    {
      if( (statusCode = DisposeUDPChannel( connection->handle )) < 0 ) 
        ERROR_EVENT( "error closing UDP client: %s", GetUDPErrorString( statusCode ) );
      else
        DEBUG_EVENT( 0, "UDP client %u closed", connection->handle );
    }
      
    if( connection->ref_server != NULL )
    {
      khint_t clientID = kh_get( IPInt, connection->ref_server->clientsList, connection->handle );
      if( clientID != kh_end( connection->ref_server->clientsList ) )
      {
        kh_del( IPInt, connection->ref_server->clientsList, clientID );
        if( kh_size( connection->ref_server->clientsList ) == 0 )
        {
          if( connection->ref_server->networkRole == LISTENER ) CloseConnection( connection->ref_server );
        }
      }
    }
  }

  connection->networkRole = DISCONNECTED;

  if( connection->callbackThread != INVALID_THREAD_HANDLE )
  {
    DEBUG_EVENT( 0, "waiting connection callback thread %x return", connection->callbackThread );
    (void) Threading.WaitExit( connection->callbackThread, INFINITE );
    DEBUG_EVENT( 0, "connection callback thread %x returned successfully", connection->callbackThread );
    
    connection->callbackThread = INVALID_THREAD_HANDLE;
  }

  DEBUG_EVENT( 0, "destroying read queue %p", connection->readQueue );
  ThreadSafeQueues.Discard( connection->readQueue );

  DEBUG_EVENT( 0, "freeing connection %p", connection );
  free( connection );
  
  for( size_t connectionID = 0; connectionID < kh_end( globalConnectionsList ); connectionID++ )
  {
    if( kh_exist( globalConnectionsList, connectionID ) )
    {
      if( kh_value( globalConnectionsList, connectionID ) == connection )
        kh_value( globalConnectionsList, connectionID ) = NULL;
    }
  }
}

void AsyncIPNetwork_CloseConnection( int connectionID )
{
  DEBUG_PRINT( "trying to close connection %d", connectionID );
  
  khint_t connectionIndex = kh_get( IPInt, globalConnectionsList, (khint_t) connectionID );
  if( connectionIndex == kh_end( globalConnectionsList ) ) return;

  CloseConnection( kh_value( globalConnectionsList, connectionIndex ) );
  kh_del( IPInt, globalConnectionsList, connectionIndex );
  DEBUG_PRINT( "connection %d closed (remaining: %u)", connectionID, kh_size( globalConnectionsList ) );
  
  if( kh_size( globalConnectionsList ) == 0 )
  {
    kh_destroy( IPInt, globalConnectionsList );
    DEBUG_PRINT( "connections list %p destroyed", globalConnectionsList );
    globalConnectionsList = NULL;
  }
}


#endif  /* ASYNC_IP_CONNECTION_H */
