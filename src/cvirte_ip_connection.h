/////////////////////////////////////////////////////////////////////////////////////
///// Real-Time platform library for creation and handling of IP sockets        /////
///// connections as server or client, using TCP or UDP protocols               /////
/////////////////////////////////////////////////////////////////////////////////////

#ifndef ASYNC_IP_CONNECTION_H
#define ASYNC_IP_CONNECTION_H

#include "cvidef.h"
#include "async_debug.h"
  
#include <tcpsupp.h>
#include <udpsupp.h>
#include <utility.h>
#include <toolbox.h>

  
//////////////////////////////////////////////////////////////////////////
/////                         DATA STRUCTURES                        /////
//////////////////////////////////////////////////////////////////////////

const size_t IP_CONNECTION_MSG_LEN = CMT_MAX_MESSAGE_BUF_SIZE; // 256

const int QUEUE_MAX_ITEMS = 10;

const size_t IP_HOST_LENGTH = 40;
const size_t IP_PORT_LENGTH = 6;
const size_t IP_ADDRESS_LENGTH = IP_HOST_LENGTH + IP_PORT_LENGTH;

enum Property { DISCONNECTED = 0x00, CLIENT = 0x01, SERVER = 0x02, LISTENER = 0x03, NETWORK_ROLE_MASK = 0x0f, TCP = 0x10, UDP = 0x20, PROTOCOL_MASK = 0xf0 };

// Generic structure to store methods and data of any connection type handled by the library
typedef struct _AsyncConnection AsyncConnection;

struct _AsyncConnection
{
  unsigned int handle;
  uint8_t protocol, networkRole;
  CmtThreadFunctionID callbackThreadID;
  int (CVICALLBACK *ref_SendMessage)( void* );
  struct
  {
    unsigned int port;
    char host[ IP_HOST_LENGTH ];
  } address;
  CmtTSQHandle readQueue;
  char writeBuffer[ IP_CONNECTION_MSG_LEN ];
  union
  {
    ListType clientsList;
    AsyncConnection* ref_server;
  };
};

// Internal (private) list of asyncronous connections created (accessible only by ID)
static ListType globalConnectionsList = NULL;


/////////////////////////////////////////////////////////////////////////////
/////                         NETWORK UTILITIES                         /////
/////////////////////////////////////////////////////////////////////////////

// Connection comparison auxiliary function
static int CVICALLBACK CompareConnections( void* item_1, void* item_2 )
{
  AsyncConnection* connection_1 = (*((AsyncConnection**) item_1));
  AsyncConnection* connection_2 = (*((AsyncConnection**) item_2));
  
  DEBUG_UPDATE( "comparing handles: %u - %u", connection_1->handle, connection_2->handle );
  
  return ( connection_1->handle - connection_2->handle );
}

enum { PEEK, REMOVE };
// Looks for a connection with given ID, and returns it if something is found
static AsyncConnection* FindConnection( ListType connectionsList, unsigned int connectionID, uint8_t remove )
{
  AsyncConnection connection_data = { .handle = connectionID };
  AsyncConnection* connection = &connection_data;
  
  DEBUG_UPDATE( "looking for connection handle %u", connection->handle );
  
  size_t index = ListFindItem( connectionsList, &connection, FRONT_OF_LIST, CompareConnections );
      
  if( index > 0 )
  {
    if( remove == 1 )
      ListRemoveItem( connectionsList, &connection, index );
    else
      ListGetItem( connectionsList, &connection, index );
    return connection;
  }
  
  return NULL;
}

// Utility method to request an address (host and port) string for a connection structure
static char* GetAddress( AsyncConnection* connection )
{
  static char address_string[ IP_ADDRESS_LENGTH ];
  
  DEBUG_EVENT( 0, "getting address string for connection handle: %u", connection->handle );
  
  sprintf( address_string, "%s/%u", connection->address.host, connection->address.port );
  
  return address_string;
}

// Returns the address string relative to the given connection ID
char* AsyncIPConnection_GetAddress( int connectionID )
{
  static AsyncConnection* connection;
  
  connection = FindConnection( globalConnectionsList, (unsigned int) connectionID, PEEK ); 

  if( connectionID < 0 || connection == NULL )
  {
    ERROR_EVENT( "invalid connection ID: %d", connectionID );
    return NULL;
  }
  
  return GetAddress( connection );
}

// Returns total number of active connections
extern inline size_t AsyncIPConnection_GetActivesNumber()
{
  return ListNumItems( globalConnectionsList );
}

// Returns number of active clients for a connection 
size_t AsyncIPConnection_GetClientsNumber( int connectionID )
{
  static AsyncConnection* connection;
  
  connection = FindConnection( globalConnectionsList, (unsigned int) connectionID, PEEK ); 

  if( connectionID < 0 || connection == NULL )
  {
    ERROR_EVENT( "invalid connection ID: %d", connectionID );
    return -1;
  }
  
  if( connection->networkRole == SERVER )
    return ListNumItems( connection->clientsList );
  
  return 1;
}

//////////////////////////////////////////////////////////////////////////////////
/////                             INITIALIZATION                             /////
//////////////////////////////////////////////////////////////////////////////////

// Forward Declaration
static int CVICALLBACK AcceptTCPClient( unsigned, int, int, void* );
static int CVICALLBACK AcceptUDPClient( unsigned, int, int, void* );
static int CVICALLBACK ReceiveTCPMessage( unsigned, int, int, void* );
static int CVICALLBACK ReceiveUDPMessage( unsigned, int, int, void* );
static int CVICALLBACK SendTCPMessage( void* );
static int CVICALLBACK SendUDPMessage( void* );
static int CVICALLBACK SendMessageToAll( void* );

static int CVICALLBACK CloseConnection( int, void*, void* );

// Asyncronous callback for ip connection setup and event processing
static int CVICALLBACK AsyncConnect( void* connectionData )
{
  AsyncConnection* connection = (AsyncConnection*) connectionData;
  
  static int statusCode;
  
  switch( connection->protocol | connection->networkRole )
  {
    case ( TCP | SERVER ):
      
      if( (statusCode = RegisterTCPServer( connection->address.port, AcceptTCPClient, connection )) < 0 )
        ERROR_EVENT( "%s: %s", GetTCPErrorString( statusCode ), GetTCPSystemErrorString() );
      else
        connection->handle = connection->address.port;
      
      break;
      
    case ( TCP | CLIENT ):
      
      if( (statusCode = ConnectToTCPServer( &(connection->handle), connection->address.port, connection->address.host, ReceiveTCPMessage, connection, 1000 )) < 0 )
        ERROR_EVENT( "%s: %s", GetTCPErrorString( statusCode ), GetTCPSystemErrorString() );
      
      break;
      
    case ( UDP | SERVER ):
      
      if( (statusCode = CreateUDPChannelConfig( connection->address.port, UDP_ANY_ADDRESS, 0, AcceptUDPClient, connection, &(connection->handle) )) < 0 )
        ERROR_EVENT( "%s", GetUDPErrorString( statusCode ) );
      
      break;
      
    case ( UDP | CLIENT ):
      
      if( (statusCode = CreateUDPChannelConfig( UDP_ANY_LOCAL_PORT, UDP_ANY_ADDRESS, 0, ReceiveUDPMessage, connection, &(connection->handle) )) < 0 )
        ERROR_EVENT( "%s", GetUDPErrorString( statusCode ) );
      
      break;
      
      default: break;
  }
  
  if( statusCode < 0 ) 
    connection->networkRole = DISCONNECTED;
  else
    DEBUG_EVENT( 0, "starting to process system events for connection %u on thread %x", connection->handle, CmtGetCurrentThreadID() );
  
  while( connection->networkRole != DISCONNECTED ) 
  {
    ProcessSystemEvents();
    /*#ifdef _LINK_CVI_LVRT_
    SleepUS( 1000 );
    #else
    Sleep( 1 );
    #endif*/
  }
  
  DEBUG_EVENT( 1, "connection handle %u closed. exiting event loop on thread %x", connection->handle, CmtGetCurrentThreadID() );
  
  CmtExitThreadPoolThread( statusCode );
  
  return statusCode;
}

// Generate connection data structure based on input arguments
static AsyncConnection* AddConnection( unsigned int connectionHandle, unsigned int addressPort, const char* addressHost, uint8_t connectionType )
{
  AsyncConnection* connection = (AsyncConnection*) malloc( sizeof(AsyncConnection) );
  
  connection->handle = connectionHandle;
  connection->protocol = (connectionType & PROTOCOL_MASK);  
  connection->networkRole = (connectionType & NETWORK_ROLE_MASK);
  
  connection->callbackThreadID = -1;
  
  connection->address.port = addressPort;
  if( addressHost != NULL ) strncpy( connection->address.host, addressHost, IP_HOST_LENGTH );
  
  if( connection->networkRole == SERVER )
  {
    connection->clientsList = NULL;
    
    CmtNewTSQ( QUEUE_MAX_ITEMS, sizeof(int), 0, &(connection->readQueue) );
    
    connection->ref_SendMessage = SendMessageToAll;
    
    connection->clientsList = ListCreate( sizeof(AsyncConnection*) );
  }
  else
  {
    if( connection->protocol == TCP )
    {
      connection->ref_SendMessage = SendTCPMessage;
      CmtNewTSQ( QUEUE_MAX_ITEMS, IP_CONNECTION_MSG_LEN, 0, &(connection->readQueue) );
    }
    else
    {
      connection->ref_SendMessage = SendUDPMessage;
      CmtNewTSQ( QUEUE_MAX_ITEMS, IP_CONNECTION_MSG_LEN, OPT_TSQ_AUTO_FLUSH_EXACT, &(connection->readQueue) );
    }
    
    connection->ref_server = NULL;
  }
  
  if( globalConnectionsList == NULL )
    globalConnectionsList = ListCreate( sizeof(AsyncConnection*) ); 
  
  ListInsertItem( globalConnectionsList, &connection, END_OF_LIST );
  
  DEBUG_EVENT( 0, "%u active connections listed", ListNumItems( globalConnectionsList ) );
  
  return connection;
}

// Generic method for opening a new socket and providing a corresponding Connection structure for use
int AsyncIPConnection_Open( const char* host, const char* port, uint8_t protocol )
{
  static AsyncConnection* newConnection;
  
  static unsigned int portNumber;
  
  DEBUG_EVENT( 0, "trying to open %s connection for host %s and port %s", ( protocol == TCP ) ? "TCP" : "UDP", 
                                                                       ( host != NULL ) ? host : "0.0.0.0", port );
  
  // Assure that the port number is in the Dynamic/Private range (49152-65535)
  portNumber = (unsigned int) strtoul( port, NULL, 0 );
  if( portNumber < 49152 || portNumber > 65535 )
  {
    ERROR_EVENT( "invalid port number value: %s", port );
    return -1;
  }
  
  if( ( protocol != TCP ) && ( protocol != UDP ) )
  {
    ERROR_EVENT( "invalid protocol option: %x", protocol );
    return -1;
  }
  
    newConnection = AddConnection( 0, portNumber, 
                                   ( host != NULL ) ? host : "0.0.0.0",
                                   ( ( host == NULL ) ? SERVER : CLIENT ) | protocol );
  
  CmtScheduleThreadPoolFunction( DEFAULT_THREAD_POOL_HANDLE, AsyncConnect, newConnection, &(newConnection->callbackThreadID) );
  
  SetBreakOnLibraryErrors( 0 );
  (void) CmtWaitForThreadPoolFunctionCompletionEx( DEFAULT_THREAD_POOL_HANDLE, newConnection->callbackThreadID, 0, 1000 );
  SetBreakOnLibraryErrors( 1 );
  
  if( newConnection->networkRole == DISCONNECTED )
  {
    ListRemoveItem( globalConnectionsList, NULL, END_OF_LIST );
    
    CmtReleaseThreadPoolFunctionID( DEFAULT_THREAD_POOL_HANDLE, newConnection->callbackThreadID );
    newConnection->callbackThreadID = -1;
    
    CloseConnection( 0, newConnection, NULL );
    
    return -1;
  }

  DEBUG_EVENT( 0, "new %s %s connection opened: %s address: %s", ( newConnection->protocol == TCP ) ? "TCP" : "UDP",
                                                                 ( newConnection->networkRole == SERVER ) ? "Server" : "Client",
                                                                 ( newConnection->networkRole == SERVER ) ? "local" : "remote",
                                                                 GetAddress( newConnection ) );
  
  return (int) newConnection->handle;
}


/////////////////////////////////////////////////////////////////////////////////
/////                       ASYNCRONOUS COMMUNICATION                       /////
/////////////////////////////////////////////////////////////////////////////////

// Try to receive incoming message from the given TCP client connection and store it on its buffer
static int CVICALLBACK ReceiveTCPMessage( unsigned int connectionHandle, int eventType, int errorCode, void* connectionData )
{
  AsyncConnection* connection = (AsyncConnection*) connectionData;
  
  char messageBuffer[ IP_CONNECTION_MSG_LEN ];
  
  DEBUG_UPDATE( "called for connection handle %u on thread %x", connectionHandle, CmtGetCurrentThreadID() );
  
  switch( eventType )
  {
    case TCP_DISCONNECT:

      if( connectionHandle == connection->handle )
            CloseConnection( 0, &connection, NULL );
      break;
      
    case TCP_DATAREADY:
        
      if( (errorCode = ClientTCPRead( connectionHandle, messageBuffer, IP_CONNECTION_MSG_LEN, 0 )) < 0 )
      {
        ERROR_EVENT( "%s: %s", GetTCPErrorString( errorCode ), GetTCPSystemErrorString() );
        if( connectionHandle == connection->handle )
                CloseConnection( 0, &connection, NULL );
        break;
      }
      
      if( (errorCode = CmtWriteTSQData( connection->readQueue, messageBuffer, 1, TSQ_INFINITE_TIMEOUT, NULL )) < 0 )
      {
        CmtGetErrorMessage( errorCode, messageBuffer );
        ERROR_EVENT( "%s", messageBuffer );
        if( connectionHandle == connection->handle )
                CloseConnection( 0, &connection, NULL );
        break;
      }
      
      DEBUG_UPDATE( "TCP connection handle %u received message: %s", connection->handle, messageBuffer );
    
      break;
  }
  
  return 0;
}

// Send given message through the given TCP connection
static int CVICALLBACK SendTCPMessage( void* connectionData )
{
  AsyncConnection* connection = (AsyncConnection*) connectionData;
  
  int errorCode;
  
  DEBUG_UPDATE( "called for connection handle %u on thread %x", connection->handle, CmtGetCurrentThreadID() );
  
  if( connection->networkRole == CLIENT )
  {
    DEBUG_UPDATE( "TCP connection handle %u sending message: %s", connection->handle, connection->writeBuffer );
    
    if( (errorCode = ClientTCPWrite( connection->handle, connection->writeBuffer, IP_CONNECTION_MSG_LEN, 0 )) < 0 )
    {
      ERROR_EVENT( "%s: %s", GetTCPErrorString( errorCode ), GetTCPSystemErrorString() );
      CloseConnection( 0, &connection, NULL );
      CmtExitThreadPoolThread( errorCode );
      return errorCode;
    }
  }
  else
  {
    DEBUG_EVENT( 0, "connection handle %u is closed", connection->handle );
    CmtExitThreadPoolThread( -1 );
    return -1;
  }
  
  CmtExitThreadPoolThread( 0 );
  return 0;
}

// Try to receive incoming message from the given UDP client connection and store it on its buffer
static int CVICALLBACK ReceiveUDPMessage( unsigned int channel, int eventType, int errorCode, void* connectionData )
{
  AsyncConnection* connection = (AsyncConnection*) connectionData;
  
  char messageBuffer[ IP_CONNECTION_MSG_LEN ];
  
  unsigned int sourcePort;
  char sourceHost[ IP_HOST_LENGTH ];
  
  DEBUG_UPDATE( "called for connection handle %u on thread %x", channel, CmtGetCurrentThreadID() );
  
  if( eventType == UDP_DATAREADY )
  {
    if( (errorCode = UDPRead( channel, NULL, IP_CONNECTION_MSG_LEN, UDP_DO_NOT_WAIT, &sourcePort, sourceHost )) < 0 )
    {
      ERROR_EVENT( "%s", GetUDPErrorString( errorCode ) );
      if( channel == connection->ref_server->handle ) CloseConnection( 0, &connection, NULL ); 
      return -1;
    }
    
    if( connection->address.port == sourcePort )
    {
      if( strncmp( connection->address.host, sourceHost, IP_HOST_LENGTH ) == 0 )
      {
        UDPRead( channel, messageBuffer, IP_CONNECTION_MSG_LEN, UDP_DO_NOT_WAIT, NULL, NULL );
    
        if( (errorCode = CmtWriteTSQData( connection->readQueue, messageBuffer, 1, TSQ_INFINITE_TIMEOUT, NULL )) < 0 )
        {
          CmtGetErrorMessage( errorCode, messageBuffer );
          ERROR_EVENT( "%s", messageBuffer );
          CloseConnection( 0, &connection, NULL );
          return -1;
        }
        
        DEBUG_UPDATE( "UDP connection handle %u (server %u) received right message: %s", connection->handle, channel, messageBuffer );
      }
    }
  }
  
  // Default return value (the received one was not destined to this connection) 
  return 0; 
}

// Send given message through the given UDP connection
static int CVICALLBACK SendUDPMessage( void* connectionData )
{
  AsyncConnection* connection = (AsyncConnection*) connectionData;
  
  int errorCode;
  
  DEBUG_UPDATE( "called for connection handle %u on thread %x", connection->handle, CmtGetCurrentThreadID() );
  
  if( connection->networkRole == CLIENT )
  {
    DEBUG_UPDATE( "UDP connection handle %u sending message: %s", connection->handle, connection->writeBuffer );
    
    if( (errorCode = UDPWrite( connection->handle, connection->address.port, connection->address.host, connection->writeBuffer, IP_CONNECTION_MSG_LEN )) )
    {
      ERROR_EVENT( "%s", GetUDPErrorString( errorCode ) );
      CloseConnection( 0, &connection, NULL );
      CmtExitThreadPoolThread( errorCode );
      return errorCode;
    }
  }
  else
  {
    DEBUG_EVENT( 0, "connection handle %u is closed", connection->handle );
    CmtExitThreadPoolThread( -1 );
    return -1;
  } 
  
  CmtExitThreadPoolThread( 0 );
  return 0;
}

static int CVICALLBACK SendMessageToClient( size_t index, void* ref_client, void* messageData )
{
  AsyncConnection* client = *((AsyncConnection**) ref_client);
  
  strncpy( client->writeBuffer, (const char*) messageData, IP_CONNECTION_MSG_LEN );
  
  return client->ref_SendMessage( client );
}

// Send given message to all the clients of the given server connection
static int CVICALLBACK SendMessageToAll( void* connectionData )
{
  AsyncConnection* connection = (AsyncConnection*) connectionData;
  
  static int statusCode;
  
  DEBUG_UPDATE( "called for connection handle %u on thread %x", connection->handle, CmtGetCurrentThreadID() );
  
  statusCode = ListApplyToEachEx( connection->clientsList, 1, SendMessageToClient, (void*) connection->writeBuffer );
  
  CmtExitThreadPoolThread( statusCode );
  
  return statusCode;
}

// Waits for a remote connection to be added to the client list of the given TCP server connection
static int CVICALLBACK AcceptTCPClient( unsigned int clientHandle, int eventType, int errorCode, void* connectionData )
{
  AsyncConnection* connection = (AsyncConnection*) connectionData;
  
  AsyncConnection* client;
  
  char messageBuffer[ IP_CONNECTION_MSG_LEN ];
  
  DEBUG_UPDATE( "called for connection handle %u on server %u on thread %x", clientHandle, connection->handle, CmtGetCurrentThreadID() );
  
  switch( eventType )
  {
    case TCP_DISCONNECT:
    
      if( (client = FindConnection( connection->clientsList, clientHandle, REMOVE )) != NULL )
      {
        DEBUG_EVENT( 0, "TCP client handle %u disconnected", client->handle );
        
        (void) CloseConnection( 0, &client, NULL );
      }
      
      break;
      
    case TCP_CONNECT:
      
      if( connection->networkRole == SERVER )
      {
        client = AddConnection( clientHandle, 0, NULL, TCP | CLIENT );
        GetTCPPeerAddr( client->handle, client->address.host, IP_HOST_LENGTH );
        client->ref_server = connection;
      
        ListInsertItem( connection->clientsList, &client, END_OF_LIST );
      
        if( (errorCode = CmtWriteTSQData( connection->readQueue, &(client->handle), 1, TSQ_INFINITE_TIMEOUT, NULL )) < 0 )
        {
          CmtGetErrorMessage( errorCode, messageBuffer );
          ERROR_EVENT( "%s", messageBuffer );
          break;
        }
        
        DEBUG_EVENT( 0, "TCP connection handle %u accepted client: %u (%s/%u)", connection->handle, client->handle, client->address.host, client->address.port );
      }
      else
        DEBUG_EVENT( 0, "TCP connection handle %u not accepting more clients", connection->handle );
    
      break;
      
    case TCP_DATAREADY:
      
      if( (client = FindConnection( connection->clientsList, clientHandle, PEEK )) != NULL )
      {
        if( (errorCode = ServerTCPRead( client->handle, messageBuffer, IP_CONNECTION_MSG_LEN, 0 )) < 0 )
        {
          ERROR_EVENT( "%s: %s", GetTCPErrorString( errorCode ), GetTCPSystemErrorString() );
          (void) CloseConnection( 0, &client, NULL );
          break;
        }
      
        if( (errorCode = CmtWriteTSQData( client->readQueue, messageBuffer, 1, TSQ_INFINITE_TIMEOUT, NULL )) < 0 )
        { 
          CmtGetErrorMessage( errorCode, messageBuffer );
          ERROR_EVENT( "%s", messageBuffer );
          break;
        }
      }
      
      DEBUG_UPDATE( "TCP connection handle %u (server_handle %u) received message: %s", clientHandle, connection->handle, messageBuffer );
    
      break;
  }
  
  return errorCode;
}

// Waits for a remote connection to be added to the client list of the given UDP server connection
static int CVICALLBACK AcceptUDPClient( unsigned int channel, int eventType, int errorCode, void* connectionData )
{
  AsyncConnection* connection = (AsyncConnection*) connectionData;
  
  AsyncConnection* client = NULL;
  
  char messageBuffer[ IP_CONNECTION_MSG_LEN ];
  
  static unsigned int clientPort;
  static char clientHost[ IP_HOST_LENGTH ];
  
  DEBUG_UPDATE( "called for connection handle %u on thread %x", channel, CmtGetCurrentThreadID() );
  
  if( eventType == UDP_DATAREADY )
  {
    if( (errorCode = UDPRead( channel, messageBuffer, IP_CONNECTION_MSG_LEN, UDP_DO_NOT_WAIT, &(clientPort), clientHost )) < 0 )
    {
      ERROR_EVENT( "%s", GetUDPErrorString( errorCode ) );
      (void) CloseConnection( 0, &client, NULL );
      return -1;
    }
    
    if( connection->networkRole == SERVER )
    {
      unsigned int clientHandle = channel + clientPort;
      if( (client = FindConnection( connection->clientsList, clientHandle, PEEK )) == NULL )
      {
        client = AddConnection( clientHandle, clientPort, clientHost, UDP | CLIENT );
        client->ref_server = connection;
        
        ListInsertItem( connection->clientsList, &client, END_OF_LIST );
        
        if( (errorCode = CmtWriteTSQData( connection->readQueue, &(client->handle), 1, TSQ_INFINITE_TIMEOUT, NULL )) < 0 )
        {
          CmtGetErrorMessage( errorCode, messageBuffer );
          ERROR_EVENT( "%s", messageBuffer );
          return -1;
        }
      
        DEBUG_EVENT( 0, "client with handle %u accepted !", client->handle );
      }
      
      if( (errorCode = CmtWriteTSQData( client->readQueue, messageBuffer, 1, TSQ_INFINITE_TIMEOUT, NULL )) < 0 )
      {
        CmtGetErrorMessage( errorCode, messageBuffer );
        ERROR_EVENT( "%s", messageBuffer );
        return -1;
      }

      DEBUG_UPDATE( "UDP connection handle %u (server handle %u) received message: %s", client->handle, client->ref_server->handle, messageBuffer );
    }
  }
  
  return 0;
}


//////////////////////////////////////////////////////////////////////////////////
/////                        SYNCRONOUS COMMUNICATION                        /////
//////////////////////////////////////////////////////////////////////////////////

// Get (and remove) message from the beginning (oldest) of the given index corresponding read queue
// Method to be called from the main thread
char* AsyncIPConnection_ReadMessage( int clientID )
{
  static char messageBuffer[ IP_CONNECTION_MSG_LEN ];
  
  static AsyncConnection* connection;
  static size_t messagesCount;
  static int errorCode;
  
  connection = FindConnection( globalConnectionsList, (unsigned int) clientID, PEEK ); 

  if( clientID < 0 || connection == NULL )
  {
    ERROR_EVENT( "no queue available for this connection: ID %d", clientID );
    return NULL;
  }
  
  if( connection->networkRole != CLIENT )
  {
    ERROR_EVENT( "not a client connection: ID %d", clientID );  
	  return NULL;
  }

  if( (errorCode = CmtGetTSQAttribute( connection->readQueue, ATTR_TSQ_ITEMS_IN_QUEUE, &messagesCount )) < 0 )
  {
    CmtGetErrorMessage( errorCode, messageBuffer );
    ERROR_EVENT( "failed getting queue items count: %s", messageBuffer );
    return NULL;
  }
  
  if( messagesCount <= 0 )
  {
	  DEBUG_UPDATE( "no messages available for this connection: ID %d", clientID );
    return NULL;
  }
  
  if( (errorCode = CmtReadTSQData( connection->readQueue, messageBuffer, 1, TSQ_INFINITE_TIMEOUT, 0 )) < 0 )
  {
    CmtGetErrorMessage( errorCode, messageBuffer );
    ERROR_EVENT( "failed reading data from queue: %s", messageBuffer );
    connection->networkRole = DISCONNECTED;
    return NULL;
  }
  
  DEBUG_UPDATE( "message from connection ID %d: %s", clientID, messageBuffer );  
  
  return messageBuffer;
}

int AsyncIPConnection_WriteMessage( int connectionID, const char* message )
{
  static AsyncConnection* connection;

  connection = FindConnection( globalConnectionsList, (unsigned int) connectionID, PEEK ); 

  if( connectionID < 0 || connection == NULL )
  {
    ERROR_EVENT( "no queue available for this connection: ID %d", connectionID );    
    return -1;
  }
  
  strncpy( connection->writeBuffer, message, IP_CONNECTION_MSG_LEN );
  
  if( CmtScheduleThreadPoolFunction( DEFAULT_THREAD_POOL_HANDLE, connection->ref_SendMessage, connection, NULL ) < 0 )
    return -1;
  
  return 0;
}

int AsyncIPConnection_GetClient( int serverID )
{
  static int clientID;

  static AsyncConnection* connection;
  static size_t clientsCount;
  static int errorCode;
  
  connection = FindConnection( globalConnectionsList, (unsigned int) serverID, PEEK ); 

  if( serverID < 0 || connection == NULL )
  {
    ERROR_EVENT( "no queue available for connection ID %d", serverID );
    return -1;
  }
  
  if( connection->networkRole != SERVER )
  {
    ERROR_EVENT( "connection ID %d is not a server connection ID", serverID );  
	  return -1;
  }
  
  if( (errorCode = CmtGetTSQAttribute( connection->readQueue, ATTR_TSQ_ITEMS_IN_QUEUE, &clientsCount )) < 0 )
  {
    char errorMessage[ DEBUG_MESSAGE_LENGTH ]; 
    CmtGetErrorMessage( errorCode, errorMessage );
    ERROR_EVENT( "%s", errorMessage );
    return -1;
  }
  
  if( clientsCount <= 0 )
  {
	  DEBUG_UPDATE( "no new clients available for server connection ID %d", serverID );
    return -1;
  }
  
  
  if( (errorCode = CmtReadTSQData( connection->readQueue, &clientID, 1, TSQ_INFINITE_TIMEOUT, 0 )) < 0 )
  {
    char errorMessage[ DEBUG_MESSAGE_LENGTH ]; 
    CmtGetErrorMessage( errorCode, errorMessage );
    ERROR_EVENT( "%s", errorMessage );
    return -1;
  }
  
  DEBUG_EVENT( 0, "new client connection ID %d from server connection ID %d", clientID , serverID );  
  
  return clientID; 
}


//////////////////////////////////////////////////////////////////////////////////
/////                               FINALIZING                               /////
//////////////////////////////////////////////////////////////////////////////////

// Handle proper destruction of any given connection type
static int CVICALLBACK CloseConnection( int index, void* ref_connection, void* closingData )
{
  AsyncConnection* connection = *((AsyncConnection**) ref_connection);
  
  static int statusCode;
  
  if( connection != NULL )
  {
    DEBUG_EVENT( 0, "closing connection handle %u\n", connection->handle );
    
    (void) FindConnection( globalConnectionsList, connection->handle, REMOVE );
    
    switch( connection->protocol | connection->networkRole )
    {
      case ( TCP | LISTENER ):
      case ( TCP | SERVER ):
        
        if( ListNumItems( connection->clientsList ) <= 0 )
        {
          if( (statusCode = UnregisterTCPServer( connection->address.port )) < 0 )
            ERROR_EVENT( "error closing TCP server: %s: %s", GetTCPErrorString( statusCode ), GetTCPSystemErrorString() );
        
          ListDispose( connection->clientsList );
          connection->clientsList = NULL;
        }
        else
        {
          connection->networkRole = LISTENER;
          return 0;
        }
      
        break;
      
      case ( TCP | CLIENT ):
      
        if( connection->ref_server == NULL )
        {
          if( (statusCode = DisconnectFromTCPServer( connection->handle )) < 0 )
            ERROR_EVENT( "error closing TCP server: %s: %s", GetTCPErrorString( statusCode ), GetTCPSystemErrorString() );
        }
        else
        {
          if( (statusCode = DisconnectTCPClient( connection->handle )) < 0 )
            ERROR_EVENT( "%s: %s", GetTCPErrorString( statusCode ), GetTCPSystemErrorString() );
          
          (void) FindConnection( connection->ref_server->clientsList, connection->handle, REMOVE );
          
          CloseConnection( 0, &(connection->ref_server), NULL );
        }
      
        break;
      
      case ( UDP | LISTENER ):
      case ( UDP | SERVER ):
      
        if( ListNumItems( connection->clientsList ) <= 0 )
        {
          if( (statusCode = DisposeUDPChannel( connection->handle )) < 0 )
            ERROR_EVENT( "%s", GetUDPErrorString( statusCode ) );
        
          ListDispose( connection->clientsList );
          connection->clientsList = NULL;
        }
        else
        {
          connection->networkRole = LISTENER;
          return 0;
        }
      
        break;
      
      case ( UDP | CLIENT ):
      
        if( (statusCode = DisposeUDPChannel( connection->handle )) < 0 )
          ERROR_EVENT( "%s", GetUDPErrorString( statusCode ) );
        
        if( connection->ref_server != NULL )
        {
          (void) FindConnection( connection->ref_server->clientsList, connection->handle, REMOVE );
          
          CloseConnection( 0, &(connection->ref_server), NULL );
        }
      
        break;
        
      default: break;
    }
    
    connection->networkRole = DISCONNECTED;
    
    if( connection->callbackThreadID != -1 )
    {
      CmtWaitForThreadPoolFunctionCompletion( DEFAULT_THREAD_POOL_HANDLE, connection->callbackThreadID, OPT_TP_PROCESS_EVENTS_WHILE_WAITING );
      CmtReleaseThreadPoolFunctionID( DEFAULT_THREAD_POOL_HANDLE, connection->callbackThreadID );
      
      DEBUG_EVENT( 0, "connection callback thread %x returned successfully", connection->callbackThreadID );
      
      connection->callbackThreadID = -1;
    }
    
    (void) CmtDiscardTSQ( connection->readQueue );
    
    free( connection );
    *((AsyncConnection**) ref_connection) = NULL;
  }

  DEBUG_EVENT( 0, "%u active connections listed", ListNumItems( globalConnectionsList ) );
  
  return 0;
}

void AsyncIPConnection_Close( int connectionID )
{
  static AsyncConnection* connection;
  
  connection = FindConnection( globalConnectionsList, (unsigned int) connectionID, PEEK ); 

  if( connectionID < 0 || connection == NULL )
  {
    ERROR_EVENT( "invalid connection ID %d", connectionID );
    return;
  }
  
  (void) CloseConnection( 0, &connection, NULL );
  
  if( AsyncIPConnection_GetActivesNumber() <= 0 )
  {
    ListDispose( globalConnectionsList );
    globalConnectionsList = NULL;
  }
}


#endif  /* ASYNC_IP_CONNECTION_H */
