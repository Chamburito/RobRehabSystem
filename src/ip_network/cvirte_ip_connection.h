/////////////////////////////////////////////////////////////////////////////////////
///// Real-Time platform library for creation and handling of IP sockets        /////
///// connections as server or client, using TCP or UDP protocols               /////
/////////////////////////////////////////////////////////////////////////////////////

#ifndef ASYNC_IP_CONNECTION_H
#define ASYNC_IP_CONNECTION_H

#include "threads/thread_safe_data.h"

#include "cvidef.h"
#include "debug/async_debug.h"

#include "interface.h"

#include <tcpsupp.h>
#include <udpsupp.h>
#include <utility.h>
#include <toolbox.h>
#include <userint.h>

//////////////////////////////////////////////////////////////////////////
/////                         DATA STRUCTURES                        /////
//////////////////////////////////////////////////////////////////////////

const size_t IP_MAX_MESSAGE_LENGTH = CMT_MAX_MESSAGE_BUF_SIZE;  // 256

const int QUEUE_MAX_ITEMS = 10;

const size_t IP_HOST_LENGTH = 40;
const size_t IP_PORT_LENGTH = 6;
const size_t IP_ADDRESS_LENGTH = IP_HOST_LENGTH + IP_PORT_LENGTH;

enum Property { DISCONNECTED = 0x00, CLIENT = 0x01, SERVER = 0x02, LISTENER = 0x03, NETWORK_ROLE_MASK = 0x0f, TCP = 0x10, UDP = 0x20, PROTOCOL_MASK = 0xf0 };

// Generic structure to store methods and data of any connection type handled by the library
typedef struct _AsyncConnectionData AsyncConnectionData;

typedef AsyncConnectionData* AsyncConnection;

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
  //CmtTSQHandle readQueue;
  ThreadSafeQueue readQueue;
  union
  {
    ListType clientsList;
    AsyncConnection ref_server;
  };
};

// Internal (private) list of asyncronous connections created (accessible only by ID)
static ListType globalConnectionsList = NULL;


/////////////////////////////////////////////////////////////////////////////
/////                             INTERFACE                             /////
/////////////////////////////////////////////////////////////////////////////

#define ASYNC_IP_CONNECTION_FUNCTIONS( namespace, function_init ) \
        function_init( char*, namespace, GetAddress, int ) \
        function_init( size_t, namespace, GetActivesNumber, void ) \
        function_init( size_t, namespace, GetClientsNumber, int ) \
        function_init( size_t, namespace, SetMessageLength, int, size_t ) \
        function_init( int, namespace, Open, const char*, const char*, uint8_t ) \
        function_init( void, namespace, Close, int ) \
        function_init( char*, namespace, ReadMessage, int ) \
        function_init( int, namespace, WriteMessage, int, const char* ) \
        function_init( int, namespace, GetClient, int )

INIT_NAMESPACE_INTERFACE( AsyncIPConnection, ASYNC_IP_CONNECTION_FUNCTIONS )

/////////////////////////////////////////////////////////////////////////////
/////                         NETWORK UTILITIES                         /////
/////////////////////////////////////////////////////////////////////////////

// Connection comparison auxiliary function
static int CVICALLBACK CompareConnections( void* item_1, void* item_2 )
{
  AsyncConnection connection_1 = (*((AsyncConnection*) item_1));
  AsyncConnection connection_2 = (*((AsyncConnection*) item_2));
  
  DEBUG_UPDATE( "comparing handles: %u - %u", connection_1->handle, connection_2->handle );
  
  return ( connection_1->handle - connection_2->handle );
}

enum { PEEK, REMOVE };
// Looks for a connection with given ID, and returns it if something is found
static AsyncConnection FindConnection( ListType connectionsList, unsigned int connectionID, uint8_t remove )
{
  AsyncConnectionData connection_data = { .handle = connectionID };
  AsyncConnection connection = &connection_data;
  
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
static char* GetAddress( AsyncConnection connection )
{
  static char address_string[ IP_ADDRESS_LENGTH ];
  
  DEBUG_PRINT( "getting address string for connection handle: %u", connection->handle );
  
  sprintf( address_string, "%s/%u", connection->address.host, connection->address.port );
  
  return address_string;
}

// Returns the address string relative to the given connection ID
char* AsyncIPConnection_GetAddress( int connectionID )
{
  static AsyncConnection connection;
  
  connection = FindConnection( globalConnectionsList, (unsigned int) connectionID, PEEK ); 

  if( connectionID < 0 || connection == NULL )
  {
    ERROR_EVENT( "invalid connection ID: %d", connectionID );
    return NULL;
  }
  
  return GetAddress( connection );
}

// Returns total number of active connections
inline size_t AsyncIPConnection_GetActivesNumber()
{
  return ListNumItems( globalConnectionsList );
}

// Returns number of active clients for a connection 
size_t AsyncIPConnection_GetClientsNumber( int connectionID )
{
  static AsyncConnection connection;
  
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

inline size_t AsyncIPConnection_SetMessageLength( int connectionID, size_t messageLength )
{
  static AsyncConnection connection;
  
  connection = FindConnection( globalConnectionsList, (unsigned int) connectionID, PEEK );
  
  if( connectionID < 0 || connection == NULL )
  {
    ERROR_EVENT( "invalid connection ID: %d", connectionID );
    return -1;
  }
  
  if( connection->networkRole == SERVER ) return 0;
  
  connection->messageLength = /*( messageLength <= CMT_MAX_MESSAGE_BUF_SIZE ) ?*/ (uint16_t) messageLength /*: CMT_MAX_MESSAGE_BUF_SIZE*/;
  
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
static AsyncConnection AddConnection( unsigned int connectionHandle, unsigned int addressPort, const char* addressHost, uint8_t connectionType )
{
  AsyncConnection connection = (AsyncConnection) malloc( sizeof(AsyncConnectionData) );
  
  connection->handle = connectionHandle;
  connection->protocol = (connectionType & PROTOCOL_MASK);  
  connection->networkRole = (connectionType & NETWORK_ROLE_MASK);
  
  connection->messageLength = CMT_MAX_MESSAGE_BUF_SIZE;
  
  connection->callbackThread = INVALID_THREAD_HANDLE;
  
  connection->address.port = addressPort;
  if( addressHost != NULL ) strncpy( connection->address.host, addressHost, IP_HOST_LENGTH );
  
  if( connection->networkRole == SERVER )
  {
    connection->clientsList = NULL;
    
    //CmtNewTSQ( QUEUE_MAX_ITEMS, sizeof(int), 0, &(connection->readQueue) );
    connection->readQueue = ThreadSafeQueues.Create( QUEUE_MAX_ITEMS, sizeof(int) );
    
    connection->ref_SendMessage = SendMessageToAll;
    
    connection->clientsList = ListCreate( sizeof(AsyncConnection) );
  }
  else
  {
    if( connection->protocol == TCP )
    {
      connection->ref_SendMessage = SendTCPMessage;
      //CmtNewTSQ( QUEUE_MAX_ITEMS, CMT_MAX_MESSAGE_BUF_SIZE, 0, &(connection->readQueue) );
    }
    else
    {
      connection->ref_SendMessage = SendUDPMessage;
      //CmtNewTSQ( QUEUE_MAX_ITEMS, CMT_MAX_MESSAGE_BUF_SIZE, OPT_TSQ_AUTO_FLUSH_EXACT, &(connection->readQueue) );
    }
    
    connection->readQueue = ThreadSafeQueues.Create( QUEUE_MAX_ITEMS, CMT_MAX_MESSAGE_BUF_SIZE );
    
    connection->ref_server = NULL;
  }
  
  if( globalConnectionsList == NULL )
    globalConnectionsList = ListCreate( sizeof(AsyncConnection) ); 
  
  ListInsertItem( globalConnectionsList, &connection, END_OF_LIST );
  
  DEBUG_EVENT( 0, "%u active connections listed", ListNumItems( globalConnectionsList ) );
  
  return connection;
}

// Generic method for opening a new socket and providing a corresponding Connection structure for use
int AsyncIPConnection_Open( const char* host, const char* port, uint8_t protocol )
{
  static AsyncConnection newConnection;
  
  static unsigned int portNumber;
  
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "trying to open %s connection for host %s and port %s", ( protocol == TCP ) ? "TCP" : "UDP", 
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
  
  newConnection->callbackThread = Threading_StartThread( AsyncConnect, newConnection, THREAD_JOINABLE );
  
  (void) Threading_WaitExit( newConnection->callbackThread, 1000 );
  
  if( newConnection->networkRole == DISCONNECTED )
  {
    ListRemoveItem( globalConnectionsList, NULL, END_OF_LIST );
    
    newConnection->callbackThread = INVALID_THREAD_HANDLE;
    
    CloseConnection( newConnection );
    
    return -1;
  }

  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "new %s %s connection opened: %s address: %s", ( newConnection->protocol == TCP ) ? "TCP" : "UDP",
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
  AsyncConnection connection = (AsyncConnection) connectionData;
  
  char messageBuffer[ CMT_MAX_MESSAGE_BUF_SIZE ];
  
  DEBUG_UPDATE( "called for connection handle %u on thread %x", connectionHandle, THREAD_ID );
  
  switch( eventType )
  {
    case TCP_DISCONNECT:

      if( connectionHandle == connection->handle )
      {
        DEBUG_PRINT( "sender %u disconnected", connectionHandle ); 
        CloseConnection( connection );
      }
      break;
      
    case TCP_DATAREADY:
        
      if( (errorCode = ClientTCPRead( connectionHandle, messageBuffer, connection->messageLength, 0 )) < 0 )
      {
        /*ERROR_EVENT*/DEBUG_PRINT( "ClientTCPRead: %s: %s", GetTCPErrorString( errorCode ), GetTCPSystemErrorString() );
        if( connectionHandle == connection->handle )
          CloseConnection( connection );
        break;
      }
      
      /*if( (errorCode = CmtWriteTSQData( connection->readQueue, messageBuffer, 1, TSQ_INFINITE_TIMEOUT, NULL )) < 0 )
      {
        CmtGetErrorMessage( errorCode, messageBuffer );
        ERROR_EVENT( "CmtWriteTSQData: %s", messageBuffer );
        if( connectionHandle == connection->handle )
          CloseConnection( connection );
        break;
      }*/
      
      ThreadSafeQueues.Enqueue( connection->readQueue, messageBuffer, QUEUE_APPEND_WAIT );
      
      DEBUG_UPDATE( "TCP connection handle %u received message: %s", connection->handle, messageBuffer );
    
      break;
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
    else errorCode = ServerTCPWrite( connection->handle - connection->ref_server->handle, message, connection->messageLength, 0 );
    
    if( errorCode < 0 )
    {
      ERROR_EVENT( "ServerTCPWrite: %s: %s", GetTCPErrorString( errorCode ), GetTCPSystemErrorString() );
      CloseConnection( connection );
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
  
  char messageBuffer[ CMT_MAX_MESSAGE_BUF_SIZE ];
  
  unsigned int sourcePort;
  char sourceHost[ IP_HOST_LENGTH ];
  
  DEBUG_UPDATE( "called for connection handle %u on thread %x", channel, THREAD_ID );
  
  if( eventType == UDP_DATAREADY )
  {
    if( (errorCode = UDPRead( channel, NULL, connection->messageLength, UDP_DO_NOT_WAIT, &sourcePort, sourceHost )) < 0 )
    {
      ERROR_EVENT( "UDPRead: %s", GetUDPErrorString( errorCode ) );
      if( channel == connection->ref_server->handle ) CloseConnection( connection ); 
      return -1;
    }
    
    if( connection->address.port == sourcePort )
    {
      if( strncmp( connection->address.host, sourceHost, IP_HOST_LENGTH ) == 0 )
      {
        UDPRead( channel, messageBuffer, connection->messageLength, UDP_DO_NOT_WAIT, NULL, NULL );
    
        /*if( (errorCode = CmtWriteTSQData( connection->readQueue, messageBuffer, 1, TSQ_INFINITE_TIMEOUT, NULL )) < 0 )
        {
          CmtGetErrorMessage( errorCode, messageBuffer );
          ERROR_EVENT( "CmtWriteTSQData: %s", messageBuffer );
          CloseConnection( connection );
          return -1;
        }*/
        
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
  int errorCode;
  
  DEBUG_UPDATE( "called for connection handle %u on thread %x", connection->handle, THREAD_ID );
  
  if( connection->networkRole == CLIENT )
  {
    unsigned int channel = ( connection->ref_server == NULL ) ? connection->handle : connection->ref_server->handle;
    
    DEBUG_UPDATE( "UDP connection handle %u (channel %u) sending message: %s", connection->handle, channel, message );
    
    if( (errorCode = UDPWrite( channel, connection->address.port, connection->address.host, message, connection->messageLength )) )
    {
      /*ERROR_EVENT*/DEBUG_PRINT( "UDPWrite: %s", GetUDPErrorString( errorCode ) );
      //CloseConnection( connection );
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

static int CVICALLBACK SendMessageToClient( size_t index, void* ref_client, void* messageData )
{
  AsyncConnection client = *((AsyncConnection*) ref_client);
  
  client->ref_SendMessage( client, (const char*) messageData );
  
  return 0;
}

// Send given message to all the clients of the given server connection
static int SendMessageToAll( AsyncConnection connection, const char* message )
{
  static int statusCode;
  
  DEBUG_UPDATE( "called for connection handle %u on thread %x", connection->handle, THREAD_ID );
  
  statusCode = ListApplyToEachEx( connection->clientsList, 1, SendMessageToClient, (void*) message );
  
  return statusCode;
}

// Waits for a remote connection to be added to the client list of the given TCP server connection
static int CVICALLBACK AcceptTCPClient( unsigned int clientHandle, int eventType, int errorCode, void* connectionData )
{
  AsyncConnection connection = (AsyncConnection) connectionData;
  
  AsyncConnection client;
  
  char messageBuffer[ CMT_MAX_MESSAGE_BUF_SIZE ];
  
  DEBUG_UPDATE( "called for connection handle %u on server %u on thread %x", clientHandle, connection->handle, CmtGetCurrentThreadID() );
  
  switch( eventType )
  {
    case TCP_DISCONNECT:
    
      if( (client = FindConnection( connection->clientsList, clientHandle, REMOVE )) != NULL )
      {
        ERROR_EVENT( 0, "TCP client handle %u disconnected", client->handle );
        
        CloseConnection( client );
      }
      
      break;
      
    case TCP_CONNECT:
      
      if( connection->networkRole == SERVER )
      {
        client = AddConnection( clientHandle + connection->handle, 0, NULL, TCP | CLIENT );
        GetTCPPeerAddr( client->handle, client->address.host, IP_HOST_LENGTH );
        client->ref_server = connection;
      
        ListInsertItem( connection->clientsList, &client, END_OF_LIST );
      
        /*if( (errorCode = CmtWriteTSQData( connection->readQueue, &(client->handle), 1, TSQ_INFINITE_TIMEOUT, NULL )) < 0 )
        {
          CmtGetErrorMessage( errorCode, messageBuffer );
          ERROR_EVENT( "CmtWriteTSQData: %s", messageBuffer );
          break;
        }*/
        
        ThreadSafeQueues.Enqueue( connection->readQueue, &(client->handle), QUEUE_APPEND_OVERWRITE );
        
        DEBUG_PRINT( "TCP connection handle %u accepted client: %u (%s/%u)", connection->handle, client->handle, client->address.host, client->address.port );
      }
      else
        DEBUG_EVENT( 0, "TCP connection handle %u not accepting more clients", connection->handle );
    
      break;
      
    case TCP_DATAREADY:
      
      if( (errorCode = ServerTCPRead( clientHandle, messageBuffer, connection->messageLength, 0 )) < 0 )
      {
        ERROR_EVENT( 0, "ServerTCPRead: %s: %s", GetTCPErrorString( errorCode ), GetTCPSystemErrorString() );
        //CloseConnection( client );
        break;
      }
      
      if( (client = FindConnection( connection->clientsList, clientHandle + connection->handle, PEEK )) != NULL )
      {
        /*if( (errorCode = CmtWriteTSQData( client->readQueue, messageBuffer, 1, TSQ_INFINITE_TIMEOUT, NULL )) < 0 )
        { 
          CmtGetErrorMessage( errorCode, messageBuffer );
          ERROR_EVENT( 0, "CmtWriteTSQData: %s", messageBuffer );
          break;
        }*/
        
        ThreadSafeQueues.Enqueue( client->readQueue, messageBuffer, QUEUE_APPEND_WAIT );
        
        DEBUG_PRINT( "TCP connection handle %u (server_handle %u) received message: %s", client->handle, connection->handle, messageBuffer );
      }
    
      break;
  }
  
  return errorCode;
}

// Waits for a remote connection to be added to the client list of the given UDP server connection
static int CVICALLBACK AcceptUDPClient( unsigned int channel, int eventType, int errorCode, void* connectionData )
{
  AsyncConnection connection = (AsyncConnection) connectionData;
  
  AsyncConnection client = NULL;
  
  char messageBuffer[ CMT_MAX_MESSAGE_BUF_SIZE ];
  
  static unsigned int clientPort;
  static char clientHost[ IP_HOST_LENGTH ];
  
  DEBUG_UPDATE( "called for connection handle %u on thread %x", channel, THREAD_ID );
  
  if( eventType == UDP_DATAREADY )
  {
    if( (errorCode = UDPRead( channel, messageBuffer, connection->messageLength, UDP_DO_NOT_WAIT, &(clientPort), clientHost )) < 0 )
    {
      /*ERROR_EVENT*/DEBUG_PRINT( "UDPRead: %s", GetUDPErrorString( errorCode ) );
      CloseConnection( client );
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
        
        /*if( (errorCode = CmtWriteTSQData( connection->readQueue, &(client->handle), 1, TSQ_INFINITE_TIMEOUT, NULL )) < 0 )
        {
          CmtGetErrorMessage( errorCode, messageBuffer );
          ERROR_EVENT( "CmtWriteTSQData: %s", messageBuffer );
          return -1;
        }*/
        
        ThreadSafeQueues.Enqueue( connection->readQueue, &(client->handle), QUEUE_APPEND_OVERWRITE );
      
        DEBUG_EVENT( 0, "client with handle %u accepted !", client->handle );
      }
      
      /*if( (errorCode = CmtWriteTSQData( client->readQueue, messageBuffer, 1, TSQ_INFINITE_TIMEOUT, NULL )) < 0 )
      {
        CmtGetErrorMessage( errorCode, messageBuffer );
        ERROR_EVENT( "CmtWriteTSQData: %s", messageBuffer );
        return -1;
      }*/
      
      ThreadSafeQueues.Enqueue( client->readQueue, messageBuffer, QUEUE_APPEND_WAIT );

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
  static char messageBuffer[ CMT_MAX_MESSAGE_BUF_SIZE ];
  
  //static size_t messagesCount;
  //static int errorCode;
  
  AsyncConnection connection = FindConnection( globalConnectionsList, (unsigned int) clientID, PEEK ); 

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

  /*if( (errorCode = CmtGetTSQAttribute( connection->readQueue, ATTR_TSQ_ITEMS_IN_QUEUE, &messagesCount )) < 0 )
  {
    CmtGetErrorMessage( errorCode, messageBuffer );
    ERROR_EVENT( "failed getting queue items count: %s", messageBuffer );
    return NULL;
  }*/
  
  if( /*messagesCount*/ThreadSafeQueues.GetItemsCount( connection->readQueue ) <= 0 )
  {
	  DEBUG_UPDATE( "no messages available for this connection: ID %d", clientID );
    return NULL;
  }
  
  /*if( (errorCode = CmtReadTSQData( connection->readQueue, messageBuffer, 1, TSQ_INFINITE_TIMEOUT, 0 )) < 0 )
  {
    CmtGetErrorMessage( errorCode, messageBuffer );
    ERROR_EVENT( "failed reading data from queue: %s", messageBuffer );
    connection->networkRole = DISCONNECTED;
    return NULL;
  }*/
  
  ThreadSafeQueues.Dequeue( connection->readQueue, messageBuffer, QUEUE_READ_WAIT );
  
  return messageBuffer;
}

int AsyncIPConnection_WriteMessage( int connectionID, const char* message )
{
  AsyncConnection connection = FindConnection( globalConnectionsList, (unsigned int) connectionID, PEEK ); 

  if( connectionID < 0 || connection == NULL )
  {
    ERROR_EVENT( "no queue available for this connection: ID %d", connectionID );    
    return -1;
  }
  
  DEBUG_UPDATE( "writing \"%s\" to handle %d", message, connectionID );
  
  return connection->ref_SendMessage( connection, message );
}

int AsyncIPConnection_GetClient( int serverID )
{
  static int clientID;

  //static size_t clientsCount;
  //static int errorCode;
  
  AsyncConnection connection = FindConnection( globalConnectionsList, (unsigned int) serverID, PEEK ); 
  
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
  
  /*if( (errorCode = CmtGetTSQAttribute( connection->readQueue, ATTR_TSQ_ITEMS_IN_QUEUE, &clientsCount )) < 0 )
  {
    char errorMessage[ DEBUG_MESSAGE_LENGTH ]; 
    CmtGetErrorMessage( errorCode, errorMessage );
    ERROR_EVENT( "%s", errorMessage );
    return -1;
  }*/
  
  if( /*clientsCount*/ThreadSafeQueues.GetItemsCount( connection->readQueue ) <= 0 )
  {
	  DEBUG_UPDATE( "no new clients available for server connection ID %d", serverID );
    return -1;
  }
  
  /*if( (errorCode = CmtReadTSQData( connection->readQueue, &clientID, 1, TSQ_INFINITE_TIMEOUT, 0 )) < 0 )
  {
    char errorMessage[ DEBUG_MESSAGE_LENGTH ]; 
    CmtGetErrorMessage( errorCode, errorMessage );
    ERROR_EVENT( "%s", errorMessage );
    return -1;
  }*/
  
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
            ERROR_EVENT( "UnregisterTCPServer: error closing TCP server: %s: %s", GetTCPErrorString( statusCode ), GetTCPSystemErrorString() );
        
          ListDispose( connection->clientsList );
          connection->clientsList = NULL;
          
          DEBUG_PRINT( "TCP server %u closed", connection->handle );
        }
        else
        {
          connection->networkRole = LISTENER;
          return;
        }
      
        break;
      
      case ( TCP | CLIENT ):
      
        if( connection->ref_server == NULL )
        {
          if( (statusCode = DisconnectFromTCPServer( connection->handle )) < 0 )
            ERROR_EVENT( "DisconnectFromTCPServer: error closing TCP server: %s: %s", GetTCPErrorString( statusCode ), GetTCPSystemErrorString() );
        }
        else
        {
          if( (statusCode = DisconnectTCPClient( connection->handle )) < 0 )
            ERROR_EVENT( "DisconnectTCPClient: %s: %s", GetTCPErrorString( statusCode ), GetTCPSystemErrorString() );
          
          (void) FindConnection( connection->ref_server->clientsList, connection->handle, REMOVE );
          
          DEBUG_PRINT( "TCP client %u closed", connection->handle );
          //CloseConnection( 0, &(connection->ref_server), NULL );
        }
      
        break;
      
      case ( UDP | LISTENER ):
      case ( UDP | SERVER ):
      
        if( ListNumItems( connection->clientsList ) <= 0 )
        {
          if( (statusCode = DisposeUDPChannel( connection->handle )) < 0 )
            ERROR_EVENT( "DisposeUDPChannel: %s", GetUDPErrorString( statusCode ) );
        
          ListDispose( connection->clientsList );
          connection->clientsList = NULL;
        }
        else
        {
          connection->networkRole = LISTENER;
          return;
        }
      
        break;
      
      case ( UDP | CLIENT ):
      
        if( connection->ref_server == NULL )
        {
          if( (statusCode = DisposeUDPChannel( connection->handle )) < 0 )
            ERROR_EVENT( "DisposeUDPChannel: %s", GetUDPErrorString( statusCode ) );
        }
        else
        {
          (void) FindConnection( connection->ref_server->clientsList, connection->handle, REMOVE );
          
          CloseConnection( connection->ref_server );
        }
      
        break;
        
      default: break;
    }
    
    connection->networkRole = DISCONNECTED;
    
    if( connection->callbackThread != INVALID_THREAD_HANDLE )
    {
      DEBUG_PRINT( "waiting connection callback thread %x return", connection->callbackThread );
      
      (void) Threading_WaitExit( connection->callbackThread, INFINITE );
      
      /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "connection callback thread %x returned successfully", connection->callbackThread );
      
      connection->callbackThread = INVALID_THREAD_HANDLE;
    }
    
    DEBUG_PRINT( "destroying message queue %p", connection->readQueue );
    //(void) CmtDiscardTSQ( connection->readQueue );
    ThreadSafeQueues.Discard( connection->readQueue );
    
    DEBUG_PRINT( "freeing connection %p", connection );
    free( connection );
  }

  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "%u active connections listed", ListNumItems( globalConnectionsList ) );
}

void AsyncIPConnection_Close( int connectionID )
{
  static AsyncConnection connection;
  
  connection = FindConnection( globalConnectionsList, (unsigned int) connectionID, PEEK ); 

  if( connectionID < 0 || connection == NULL )
  {
    ERROR_EVENT( "invalid connection ID %d", connectionID );
    return;
  }
  
  CloseConnection( connection );
  
  DEBUG_PRINT( "connection %d closed", connectionID );
  
  if( ListNumItems( globalConnectionsList )/*AsyncIPConnection_GetActivesNumber()*/ <= 0 )
  {
    ListDispose( globalConnectionsList );
    globalConnectionsList = NULL;
    
    DEBUG_PRINT( "connections list %p destroyed", globalConnectionsList );
  }
}


#endif  /* ASYNC_IP_CONNECTION_H */
