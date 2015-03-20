/////////////////////////////////////////////////////////////////////////////////////
///// Real-Time platform library for creation and handling of IP sockets        /////
///// connections as server or client, using TCP or UDP protocols               /////
/////////////////////////////////////////////////////////////////////////////////////

#ifndef CONNECT_H
#define CONNECT_H

#ifdef __cplusplus
extern "C" {
#endif

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

const int MAX_DATA = 10;

const size_t HOST_LENGTH = 40;
const size_t PORT_LENGTH = 6;
const size_t ADDRESS_LENGTH = HOST_LENGTH + PORT_LENGTH;

enum Property { DISCONNECTED = 0x00, CLIENT = 0x01, SERVER = 0x02, LISTENER = 0x03, NETWORK_ROLE_MASK = 0x0f, TCP = 0x10, UDP = 0x20, PROTOCOL_MASK = 0xf0 };

// Generic structure to store methods and data of any connection type handled by the library
typedef struct _Async_Connection Async_Connection;

struct _Async_Connection
{
  unsigned int handle;
  uint8_t protocol, network_role;
  CmtThreadFunctionID callback_thread_id;
  int (CVICALLBACK *send_message)( void* );
  struct
  {
    unsigned int port;
    char host[ HOST_LENGTH ];
  } address;
  CmtTSQHandle read_queue;
  char write_buffer[ IP_CONNECTION_MSG_LEN ];
  union
  {
    ListType client_list;
    Async_Connection* server_ref;
  };
};

// Internal (private) list of asyncronous connections created (accessible only by ID)
static ListType global_connection_list = NULL;


/////////////////////////////////////////////////////////////////////////////
/////                         NETWORK UTILITIES                         /////
/////////////////////////////////////////////////////////////////////////////

// Connection comparison auxiliary function
static int CVICALLBACK compare_connections( void* item_1, void* item_2 )
{
  Async_Connection* connection_1 = (*((Async_Connection**) item_1));
  Async_Connection* connection_2 = (*((Async_Connection**) item_2));
  
  //if( connection_1 == NULL ) return -1;
  //else if( connection_2 == NULL ) return 1;
  
  DEBUG_UPDATE( "comparing handles: %u - %u", connection_1->handle, connection_2->handle );
  
  return ( connection_1->handle - connection_2->handle );
}

enum { PEEK, REMOVE };
// Looks for a connection with given ID, and returns it if something is found
static Async_Connection* find_connection( ListType connection_list, unsigned int connection_id, uint8_t remove )
{
  Async_Connection connection_data = { .handle = connection_id };
  Async_Connection* connection = &connection_data;
  
  DEBUG_UPDATE( "looking for connection handle %u", connection->handle );
  
  size_t index = ListFindItem( connection_list, &connection, FRONT_OF_LIST, compare_connections );
      
  if( index > 0 )
  {
    if( remove == 1 )
      ListRemoveItem( connection_list, &connection, index );
    else
      ListGetItem( connection_list, &connection, index );
    return connection;
  }
  
  return NULL;
}

// Utility method to request an address (host and port) string for a connection structure
static char* get_address( Async_Connection* connection )
{
  static char address_string[ ADDRESS_LENGTH ];
  
  DEBUG_EVENT( "getting address string for connection handle: %u", connection->handle );
  
  sprintf( address_string, "%s/%u", connection->address.host, connection->address.port );
  
  return address_string;
}
// Returns the address string relative to the given connection ID
char* get_id_address( ssize_t connection_id )
{
  static Async_Connection* connection;
  
  connection = find_connection( global_connection_list, (unsigned int) connection_id, PEEK ); 

  if( connection_id < 0 || connection == NULL )
  {
    ERROR_EVENT( "invalid connection ID: %d", connection_id );
    return NULL;
  }
  
  return get_address( connection );
}

// Returns total number of active connections
extern inline size_t get_connections_number()
{
  return ListNumItems( global_connection_list );
}

// Returns number of active clients for a connection 
size_t async_connection_get_clients_count( ssize_t connection_id )
{
  static Async_Connection* connection;
  
  connection = find_connection( global_connection_list, (unsigned int) connection_id, PEEK ); 

  if( connection_id < 0 || connection == NULL )
  {
    ERROR_EVENT( "invalid connection ID: %d", connection_id );
    return -1;
  }
  
  if( connection->network_role == SERVER )
    return ListNumItems( connection->client_list );
  
  return 1;
}

//////////////////////////////////////////////////////////////////////////////////
/////                             INITIALIZATION                             /////
//////////////////////////////////////////////////////////////////////////////////

// Forward Declaration
static int CVICALLBACK accept_tcp_client( unsigned, int, int, void* );
static int CVICALLBACK accept_udp_client( unsigned, int, int, void* );
static int CVICALLBACK receive_tcp_message( unsigned, int, int, void* );
static int CVICALLBACK receive_udp_message( unsigned, int, int, void* );
static int CVICALLBACK send_tcp_message( void* );
static int CVICALLBACK send_udp_message( void* );
static int CVICALLBACK send_message_all( void* );

static int CVICALLBACK close_connection( int, void*, void* );

// Asyncronous callback for ip connection setup and event processing
static int CVICALLBACK async_connect( void* connection_data )
{
  Async_Connection* connection = (Async_Connection*) connection_data;
  
  static int status_code;
  
  switch( connection->protocol | connection->network_role )
  {
    case ( TCP | SERVER ):
      
      if( (status_code = RegisterTCPServer( connection->address.port, accept_tcp_client, connection )) < 0 )
        ERROR_EVENT( "%s: %s", GetTCPErrorString( status_code ), GetTCPSystemErrorString() );
      else
        connection->handle = connection->address.port;
      
      break;
      
    case ( TCP | CLIENT ):
      
      if( (status_code = ConnectToTCPServer( &(connection->handle), connection->address.port, connection->address.host, receive_tcp_message, connection, 1000 )) < 0 )
        ERROR_EVENT( "%s: %s", GetTCPErrorString( status_code ), GetTCPSystemErrorString() );
      
      break;
      
    case ( UDP | SERVER ):
      
      if( (status_code = CreateUDPChannelConfig( connection->address.port, UDP_ANY_ADDRESS, 0, accept_udp_client, connection, &(connection->handle) )) < 0 )
        ERROR_EVENT( "%s", GetUDPErrorString( status_code ) );
      
      break;
      
    case ( UDP | CLIENT ):
      
      if( (status_code = CreateUDPChannelConfig( UDP_ANY_LOCAL_PORT, UDP_ANY_ADDRESS, 0, receive_udp_message, connection, &(connection->handle) )) < 0 )
        ERROR_EVENT( "%s", GetUDPErrorString( status_code ) );
      
      break;
      
      default: break;
  }
  
  if( status_code < 0 ) 
    connection->network_role = DISCONNECTED;
  else
    DEBUG_EVENT( "starting to process system events for connection %u on thread %x", connection->handle, CmtGetCurrentThreadID() );
  
  while( connection->network_role != DISCONNECTED ) 
  {
    ProcessSystemEvents();
    #ifdef _LINK_CVI_LVRT_
    SleepUS( 100 );
    #else
    Sleep( 1 );
    #endif
  }
  
  DEBUG_EVENT( "connection handle %u closed. exiting event loop on thread %x", connection->handle, CmtGetCurrentThreadID() );
  
  CmtExitThreadPoolThread( status_code );
  
  return status_code;
}

// Generate connection data structure based on input arguments
static Async_Connection* add_connection( unsigned int connection_handle, unsigned int address_port, const char* address_host, uint8_t connection_type )
{
  Async_Connection* connection = (Async_Connection*) malloc( sizeof(Async_Connection) );
  
  connection->handle = connection_handle;
  connection->protocol = (connection_type & PROTOCOL_MASK);  
  connection->network_role = (connection_type & NETWORK_ROLE_MASK);
  
  connection->callback_thread_id = -1;
  
  connection->address.port = address_port;
  if( address_host != NULL ) strncpy( connection->address.host, address_host, HOST_LENGTH );
  
  if( connection->network_role == SERVER )
  {
    connection->client_list = NULL;
    
    CmtNewTSQ( MAX_DATA, sizeof(Async_Connection*), 0, &(connection->read_queue) );
    
    connection->send_message = send_message_all;
    
    connection->client_list = ListCreate( sizeof(Async_Connection*) );
  }
  else
  {
    if( connection->protocol == TCP )
      connection->send_message = send_tcp_message;
    else
      connection->send_message = send_udp_message;
    
    connection->server_ref = NULL;
    
    CmtNewTSQ( MAX_DATA, IP_CONNECTION_MSG_LEN, 0, &(connection->read_queue) );
  }
  
  if( global_connection_list == NULL )
    global_connection_list = ListCreate( sizeof(Async_Connection*) ); 
  
  ListInsertItem( global_connection_list, &connection, END_OF_LIST );
  
  DEBUG_EVENT( "%u active connections listed", ListNumItems( global_connection_list ) );
  
  return connection;
}

// Generic method for opening a new socket and providing a corresponding Connection structure for use
ssize_t async_connection_open( const char* host, const char* port, uint8_t protocol )
{
  static Async_Connection* new_connection;
  
  static unsigned int port_number;
  
  DEBUG_EVENT( "trying to open %s connection for host %s and port %s", ( protocol == TCP ) ? "TCP" : "UDP", 
                                                                       ( host != NULL ) ? host : "0.0.0.0", port );
  
  // Assure that the port number is in the Dynamic/Private range (49152-65535)
  port_number = (unsigned int) strtoul( port, NULL, 0 );
  if( port_number < 49152 || port_number > 65535 )
  {
    ERROR_EVENT( "invalid port number value: %s", port );
    return -1;
  }
  
  if( ( protocol != TCP ) && ( protocol != UDP ) )
  {
    ERROR_EVENT( "invalid protocol option: %x", protocol );
    return -1;
  }
  
  new_connection = add_connection( 0, port_number, 
                                   ( host != NULL ) ? host : "0.0.0.0",
                                   ( ( host == NULL ) ? SERVER : CLIENT ) | protocol );
  
  CmtScheduleThreadPoolFunction( DEFAULT_THREAD_POOL_HANDLE, async_connect, new_connection, &(new_connection->callback_thread_id) );
  
  SetBreakOnLibraryErrors( 0 );
  (void) CmtWaitForThreadPoolFunctionCompletionEx( DEFAULT_THREAD_POOL_HANDLE, new_connection->callback_thread_id, 0, 1000 );
  SetBreakOnLibraryErrors( 1 );
  
  if( new_connection->network_role == DISCONNECTED )
  {
    ListRemoveItem( global_connection_list, NULL, END_OF_LIST );
    
    CmtReleaseThreadPoolFunctionID( DEFAULT_THREAD_POOL_HANDLE, new_connection->callback_thread_id );
    new_connection->callback_thread_id = -1;
    
    close_connection( 0, new_connection, NULL );
    
    return -1;
  }

  DEBUG_EVENT( "new %s %s connection opened: %s address: %s", ( new_connection->protocol == TCP ) ? "TCP" : "UDP",
                                                              ( new_connection->network_role == SERVER ) ? "Server" : "Client",
                                                              ( new_connection->network_role == SERVER ) ? "local" : "remote",
                                                              get_address( new_connection ) );
  
  return (ssize_t) new_connection->handle;
}


/////////////////////////////////////////////////////////////////////////////////
/////                       ASYNCRONOUS COMMUNICATION                       /////
/////////////////////////////////////////////////////////////////////////////////

// Try to receive incoming message from the given TCP client connection and store it on its buffer
static int CVICALLBACK receive_tcp_message( unsigned int connection_handle, int event_type, int error_code, void* connection_data )
{
  Async_Connection* connection = (Async_Connection*) connection_data;
  
  char message_buffer[ IP_CONNECTION_MSG_LEN ];
  
  DEBUG_UPDATE( "called for connection handle %u on thread %x", connection_handle, CmtGetCurrentThreadID() );
  
  switch( event_type )
	{
		case TCP_DISCONNECT:
			
      if( connection_handle == connection->handle )
        close_connection( 0, &connection, NULL );
			break;
      
		case TCP_DATAREADY:
        
			if( (error_code = ClientTCPRead( connection_handle, message_buffer, IP_CONNECTION_MSG_LEN, 0 )) < 0 )
      {
        ERROR_EVENT( "%s: %s", GetTCPErrorString( error_code ), GetTCPSystemErrorString() );
        if( connection_handle == connection->handle )
          close_connection( 0, &connection, NULL );
        break;
      }
      
      if( (error_code = CmtWriteTSQData( connection->read_queue, message_buffer, 1, TSQ_INFINITE_TIMEOUT, NULL )) < 0 )
      {
        CmtGetErrorMessage( error_code, message_buffer );
        ERROR_EVENT( "%s", message_buffer );
        if( connection_handle == connection->handle )
          close_connection( 0, &connection, NULL );
        break;
      }
      
      DEBUG_UPDATE( "TCP connection handle %u received message: %s", connection->handle, message_buffer );
		  
      break;
	}
  
  return 0;
}

// Send given message through the given TCP connection
static int CVICALLBACK send_tcp_message( void* connection_data )
{
  Async_Connection* connection = (Async_Connection*) connection_data;
  
  int error_code;
  
  DEBUG_UPDATE( "called for connection handle %u on thread %x", connection->handle, CmtGetCurrentThreadID() );
  
  if( connection->network_role == CLIENT )
  {
    DEBUG_UPDATE( "TCP connection handle %u sending message: %s", connection->handle, connection->write_buffer );
    
  	if( (error_code = ClientTCPWrite( connection->handle, connection->write_buffer, IP_CONNECTION_MSG_LEN, 0 )) < 0 )
    {
      ERROR_EVENT( "%s: %s", GetTCPErrorString( error_code ), GetTCPSystemErrorString() );
      close_connection( 0, &connection, NULL );
      CmtExitThreadPoolThread( error_code );
      return error_code;
    }
  }
  else
  {
    DEBUG_EVENT( "connection handle %u is closed", connection->handle );
    CmtExitThreadPoolThread( -1 );
    return -1;
  }
  
  CmtExitThreadPoolThread( 0 );
  return 0;
}

// Try to receive incoming message from the given UDP client connection and store it on its buffer
static int CVICALLBACK receive_udp_message( unsigned int connection_handle, int event_type, int error_code, void* connection_data )
{
  Async_Connection* connection = (Async_Connection*) connection_data;
  
  char message_buffer[ IP_CONNECTION_MSG_LEN ];
  
  unsigned int source_port;
  char source_host[ HOST_LENGTH ];
  
  DEBUG_UPDATE( "called for connection handle %u on thread %x", connection_handle, CmtGetCurrentThreadID() );
  
  if( event_type == UDP_DATAREADY )
	{
		if( (error_code = UDPRead( connection_handle, NULL, IP_CONNECTION_MSG_LEN, UDP_DO_NOT_WAIT, &source_port, source_host )) < 0 )
    {
      ERROR_EVENT( "%s", GetUDPErrorString( error_code ) );
      if( connection_handle == connection->handle )
        close_connection( 0, &connection, NULL ); 
      return -1;
    }
    
    if( connection->address.port == source_port )
    {
      if( strncmp( connection->address.host, source_host, HOST_LENGTH ) == 0 )
      {
        UDPRead( connection_handle, message_buffer, IP_CONNECTION_MSG_LEN, UDP_DO_NOT_WAIT, NULL, NULL );
    
        if( (error_code = CmtWriteTSQData( connection->read_queue, message_buffer, 1, TSQ_INFINITE_TIMEOUT, NULL )) < 0 )
        {
          CmtGetErrorMessage( error_code, message_buffer );
          ERROR_EVENT( "%s", message_buffer );
          close_connection( 0, &connection, NULL );
          return -1;
        }
        
        DEBUG_UPDATE( "UDP connection handle %u (received %u) received right message: %s", connection->handle, connection_handle, message_buffer );
      }
    }
	}
  
  // Default return value (the received one was not destined to this connection) 
  return 0; 
}

// Send given message through the given UDP connection
static int CVICALLBACK send_udp_message( void* connection_data )
{
  Async_Connection* connection = (Async_Connection*) connection_data;
  
  int error_code;
  
  DEBUG_UPDATE( "called for connection handle %u on thread %x", connection->handle, CmtGetCurrentThreadID() );
  
  if( connection->network_role == CLIENT )
  {
    DEBUG_UPDATE( "UDP connection handle %u sending message: %s", connection->handle, connection->write_buffer );
    
    if( (error_code = UDPWrite( connection->handle, connection->address.port, connection->address.host, connection->write_buffer, IP_CONNECTION_MSG_LEN )) )
    {
      ERROR_EVENT( "%s", GetUDPErrorString( error_code ) );
      close_connection( 0, &connection, NULL );
      CmtExitThreadPoolThread( error_code );
      return error_code;
    }
  }
  else
  {
    DEBUG_EVENT( "connection handle %u is closed", connection->handle );
    CmtExitThreadPoolThread( -1 );
    return -1;
  } 
  
  CmtExitThreadPoolThread( 0 );
  return 0;
}

static int CVICALLBACK send_message_client( size_t index, void* client_ref, void* message_data )
{
  Async_Connection* client = *((Async_Connection**) client_ref);
  
  strncpy( client->write_buffer, (const char*) message_data, IP_CONNECTION_MSG_LEN );
  
  return client->send_message( client );
}

// Send given message to all the clients of the given server connection
static int CVICALLBACK send_message_all( void* connection_data )
{
  Async_Connection* connection = (Async_Connection*) connection_data;
  
  static int status_code;
  
  DEBUG_UPDATE( "called for connection handle %u on thread %x", connection->handle, CmtGetCurrentThreadID() );
  
  status_code = ListApplyToEachEx( connection->client_list, 1, send_message_client, (void*) connection->write_buffer );
  
  CmtExitThreadPoolThread( status_code );
  
  return status_code;
}

// Waits for a remote connection to be added to the client list of the given TCP server connection
static int CVICALLBACK accept_tcp_client( unsigned int client_handle, int event_type, int error_code, void* connection_data )
{
  Async_Connection* connection = (Async_Connection*) connection_data;
  
  Async_Connection* client;
  
  char message_buffer[ IP_CONNECTION_MSG_LEN ];
  
  DEBUG_UPDATE( "called for connection handle %u on server %u on thread %x", client_handle, connection->handle, CmtGetCurrentThreadID() );
  
  switch( event_type )
	{
		case TCP_DISCONNECT:
			
      if( (client = find_connection( connection->client_list, client_handle, REMOVE )) != NULL )
      {
        DEBUG_EVENT( "TCP client handle %u disconnected", client->handle );
        
        (void) close_connection( 0, &client, NULL );
      }
      
			break;
      
		case TCP_CONNECT:
      
      if( connection->network_role == SERVER )
      {
  			client = add_connection( client_handle, 0, NULL, TCP | CLIENT );
        GetTCPPeerAddr( client->handle, client->address.host, HOST_LENGTH );
        client->server_ref = connection;
      
        ListInsertItem( connection->client_list, &client, END_OF_LIST );
      
        if( (error_code = CmtWriteTSQData( connection->read_queue, &client, 1, TSQ_INFINITE_TIMEOUT, NULL )) < 0 )
        {
          CmtGetErrorMessage( error_code, message_buffer );
          ERROR_EVENT( "%s", message_buffer );
          break;
        }
        
        DEBUG_EVENT( "TCP connection handle %u accepted client: %u (%s/%u)", connection->handle, client->handle, client->address.host, client->address.port );
      }
      else
        DEBUG_EVENT( "TCP connection handle %u not accepting more clients", connection->handle );
		  
      break;
      
    case TCP_DATAREADY:
      
			if( (client = find_connection( connection->client_list, client_handle, PEEK )) != NULL )
      {
        if( (error_code = ServerTCPRead( client->handle, message_buffer, IP_CONNECTION_MSG_LEN, 0 )) < 0 )
        {
          ERROR_EVENT( "%s: %s", GetTCPErrorString( error_code ), GetTCPSystemErrorString() );
          (void) close_connection( 0, &client, NULL );
          break;
        }
      
        if( (error_code = CmtWriteTSQData( client->read_queue, message_buffer, 1, TSQ_INFINITE_TIMEOUT, NULL )) < 0 )
        { 
          CmtGetErrorMessage( error_code, message_buffer );
          ERROR_EVENT( "%s", message_buffer );
          break;
        }
      }
      
      DEBUG_UPDATE( "TCP connection handle %u (server_handle %u) received message: %s", client_handle, connection->handle, message_buffer );
		  
      break;
	}
  
  return error_code;
}

// Waits for a remote connection to be added to the client list of the given UDP server connection
static int CVICALLBACK accept_udp_client( unsigned int channel, int event_type, int error_code, void* connection_data )
{
  Async_Connection* connection = (Async_Connection*) connection_data;
  
  Async_Connection* client = NULL;
  
  char message_buffer[ IP_CONNECTION_MSG_LEN ];
  
  DEBUG_UPDATE( "called for connection handle %u on server %u on thread %x", channel, connection->handle, CmtGetCurrentThreadID() );
  
  if( event_type == UDP_DATAREADY )
  {
    if( connection->network_role == SERVER )
    {
      if( (client = find_connection( connection->client_list, channel, PEEK )) == NULL )
      {
        client = add_connection( channel, client->address.port, client->address.host, UDP | CLIENT );
        ListInsertItem( connection->client_list, &client, END_OF_LIST );
      
        //DEBUG_EVENT( "client accepted !\n" );
      }
    }
    
    if( (error_code = UDPRead( channel, message_buffer, IP_CONNECTION_MSG_LEN, UDP_DO_NOT_WAIT, &(client->address.port), client->address.host )) < 0 )
    {
      ERROR_EVENT( "%s", GetUDPErrorString( error_code ) );
      (void) close_connection( 0, &client, NULL );
      return -1;
    }
    
    if( (error_code = CmtWriteTSQData( client->read_queue, message_buffer, 1, TSQ_INFINITE_TIMEOUT, NULL )) < 0 )
    {
      CmtGetErrorMessage( error_code, message_buffer );
      ERROR_EVENT( "%s", message_buffer );
      return -1;
    }

    DEBUG_UPDATE( "UDP connection handle %u (server handle %u) received message: %s", channel, connection->handle, message_buffer );
  }
  
  return 0;
}


//////////////////////////////////////////////////////////////////////////////////
/////                        SYNCRONOUS COMMUNICATION                        /////
//////////////////////////////////////////////////////////////////////////////////

// Get (and remove) message from the beginning (oldest) of the given index corresponding read queue
// Method to be called from the main thread
char* async_connection_read_message( ssize_t connection_id )
{
  static char message_buffer[ IP_CONNECTION_MSG_LEN ];
  
  static Async_Connection* connection;
  static size_t n_messages;
  static int error_code;
  
  connection = find_connection( global_connection_list, (unsigned int) connection_id, PEEK ); 

  if( connection_id < 0 || connection == NULL )
  {
    ERROR_EVENT( "no queue available for this connection: ID %d", connection_id );
    return NULL;
  }
  
  if( connection->network_role != CLIENT )
  {
    ERROR_EVENT( "not a client connection: ID %d", connection_id );  
	  return NULL;
  }

  if( (error_code = CmtGetTSQAttribute( connection->read_queue, ATTR_TSQ_ITEMS_IN_QUEUE, &n_messages )) < 0 )
  {
    CmtGetErrorMessage( error_code, message_buffer );
    ERROR_EVENT( "failed getting queue items count: %s", message_buffer );
    return NULL;
  }
  
  if( n_messages <= 0 )
  {
	  DEBUG_UPDATE( "no messages available for this connection: ID %d", connection_id );
    return NULL;
  }
  
  if( (error_code = CmtReadTSQData( connection->read_queue, message_buffer, 1, TSQ_INFINITE_TIMEOUT, 0 )) < 0 )
  {
    CmtGetErrorMessage( error_code, message_buffer );
    ERROR_EVENT( "failed reading data from queue: %s", message_buffer );
    connection->network_role = DISCONNECTED;
    return NULL;
  }
  
  DEBUG_UPDATE( "message from connection ID %d: %s", connection_id, message_buffer );  
  
  return message_buffer;
}

int async_connection_write_message( ssize_t connection_id, const char* message )
{
  static Async_Connection* connection;

  connection = find_connection( global_connection_list, (unsigned int) connection_id, PEEK ); 

  if( connection_id < 0 || connection == NULL )
  {
    ERROR_EVENT( "no queue available for this connection: ID %d", connection_id );    
    return -1;
  }
  
  strncpy( connection->write_buffer, message, IP_CONNECTION_MSG_LEN );
  
  if( CmtScheduleThreadPoolFunction( DEFAULT_THREAD_POOL_HANDLE, connection->send_message, connection, NULL ) < 0 )
    return -1;
  
  return 0;
}

int async_connection_get_client( ssize_t connection_id )
{
  static int client_id;

  static Async_Connection* connection;
  static size_t n_clients;
  static int error_code;
  
  connection = find_connection( global_connection_list, (unsigned int) connection_id, PEEK ); 

  if( connection_id < 0 || connection == NULL )
  {
    ERROR_EVENT( "no queue available for connection ID %d", connection_id );
    return -1;
  }
  
  if( connection->network_role != SERVER )
  {
    ERROR_EVENT( "connection ID %d is not a server connection ID", connection_id );  
	  return -1;
  }
  
  if( (error_code = CmtGetTSQAttribute( connection->read_queue, ATTR_TSQ_ITEMS_IN_QUEUE, &n_clients )) < 0 )
  {
    char error_message[ DEBUG_MESSAGE_LENGTH ]; 
    CmtGetErrorMessage( error_code, error_message );
    ERROR_EVENT( "%s", error_message );
    return -1;
  }
  
  if( n_clients <= 0 )
  {
	  DEBUG_UPDATE( "no new clients available for server connection ID %d", connection_id );
    return -1;
  }
  
  
  if( (error_code = CmtReadTSQData( connection->read_queue, &client_id, 1, TSQ_INFINITE_TIMEOUT, 0 )) < 0 )
  {
    char error_message[ DEBUG_MESSAGE_LENGTH ]; 
    CmtGetErrorMessage( error_code, error_message );
    ERROR_EVENT( "%s", error_message );
    return -1;
  }
  
  DEBUG_UPDATE( "new client connection ID %d from server connection ID %d", client_id , connection_id );  
  
  return client_id; 
}


//////////////////////////////////////////////////////////////////////////////////
/////                               FINALIZING                               /////
//////////////////////////////////////////////////////////////////////////////////

// Handle proper destruction of any given connection type
static int CVICALLBACK close_connection( int index, void* connection_ref, void* closing_data )
{
  Async_Connection* connection = *((Async_Connection**) connection_ref);
  
  static int status_code;
  
  if( connection != NULL )
  {
    DEBUG_EVENT( "closing connection handle %u\n", connection->handle );
    
    (void) find_connection( global_connection_list, connection->handle, REMOVE );
    
    switch( connection->protocol | connection->network_role )
    {
      case ( TCP | LISTENER ):
      case ( TCP | SERVER ):
        
        if( ListNumItems( connection->client_list ) <= 0 )
        {
          if( (status_code = UnregisterTCPServer( connection->address.port )) < 0 )
            ERROR_EVENT( "error closing TCP server: %s: %s", GetTCPErrorString( status_code ), GetTCPSystemErrorString() );
        
          ListDispose( connection->client_list );
          connection->client_list = NULL;
        }
        else
        {
          connection->network_role = LISTENER;
          return 0;
        }
      
        break;
      
      case ( TCP | CLIENT ):
      
        if( connection->server_ref == NULL )
        {
          if( (status_code = DisconnectFromTCPServer( connection->handle )) < 0 )
            ERROR_EVENT( "error closing TCP server: %s: %s", GetTCPErrorString( status_code ), GetTCPSystemErrorString() );
        }
        else
        {
          if( (status_code = DisconnectTCPClient( connection->handle )) < 0 )
            ERROR_EVENT( "%s: %s", GetTCPErrorString( status_code ), GetTCPSystemErrorString() );
          
          (void) find_connection( connection->server_ref->client_list, connection->handle, REMOVE );
          
          close_connection( 0, &(connection->server_ref), NULL );
        }
      
        break;
      
      case ( UDP | LISTENER ):
      case ( UDP | SERVER ):
      
        if( ListNumItems( connection->client_list ) <= 0 )
        {
          if( (status_code = DisposeUDPChannel( connection->handle )) < 0 )
            ERROR_EVENT( "%s", GetUDPErrorString( status_code ) );
        
          ListDispose( connection->client_list );
          connection->client_list = NULL;
        }
        else
        {
          connection->network_role = LISTENER;
          return 0;
        }
      
        break;
      
      case ( UDP | CLIENT ):
      
        if( (status_code = DisposeUDPChannel( connection->handle )) < 0 )
          ERROR_EVENT( "%s", GetUDPErrorString( status_code ) );
        
        if( connection->server_ref != NULL )
        {
          (void) find_connection( connection->server_ref->client_list, connection->handle, REMOVE );
          
          close_connection( 0, &(connection->server_ref), NULL );
        }
      
        break;
        
      default: break;
    }
    
    connection->network_role = DISCONNECTED;
    
    if( connection->callback_thread_id != -1 )
    {
      CmtWaitForThreadPoolFunctionCompletion( DEFAULT_THREAD_POOL_HANDLE, connection->callback_thread_id, OPT_TP_PROCESS_EVENTS_WHILE_WAITING );
      CmtReleaseThreadPoolFunctionID( DEFAULT_THREAD_POOL_HANDLE, connection->callback_thread_id );
      
      DEBUG_EVENT( "connection callback thread %x returned successfully", connection->callback_thread_id );
      
      connection->callback_thread_id = -1;
    }
    
    (void) CmtDiscardTSQ( connection->read_queue );
    
    free( connection );
    *((Async_Connection**) connection_ref) = NULL;
  }

  DEBUG_EVENT( "%u active connections listed", ListNumItems( global_connection_list ) );
  
  return 0;
}

void async_connection_close( ssize_t connection_id )
{
  static Async_Connection* connection;
  
  connection = find_connection( global_connection_list, (unsigned int) connection_id, PEEK ); 

  if( connection_id < 0 || connection == NULL )
  {
    ERROR_EVENT( "invalid connection ID %d", connection_id );
    return;
  }
  
  (void) close_connection( 0, &connection, NULL );
}

void inline async_connections_close_all()
{
  if( global_connection_list != NULL )
    ListApplyToEach( global_connection_list, 1, close_connection, NULL );
}

#ifdef __cplusplus
}
#endif

#endif  /* CONNECT_H */
