/////////////////////////////////////////////////////////////////////////////////////
///// Real-Time platform library for creation and handling of IP sockets        /////
///// connections as server or client, using TCP or UDP protocols               /////
/////////////////////////////////////////////////////////////////////////////////////

#ifndef CONNECT_H
#define CONNECT_H

#ifdef __cplusplus
    extern "C" {
#endif

      
#include <tcpsupp.h>
#include <udpsupp.h>
#include <utility.h>
      
#include "cvidef.h"
#include "debug.h"

  
//////////////////////////////////////////////////////////////////////////
/////                         DATA STRUCTURES                        /////
//////////////////////////////////////////////////////////////////////////

const size_t MESSAGE_BUFFER_SIZE = CMT_MAX_MESSAGE_BUF_SIZE; // 256

const int MAX_DATA = 10;
const unsigned int BASE_WAIT_TIME = 1;

const size_t HOST_LENGTH = 40;
const size_t PORT_LENGTH = 6;
const size_t ADDRESS_LENGTH = HOST_LENGTH + PORT_LENGTH;

const size_t HOST = 0;
const size_t PORT = HOST_LENGTH;

enum Property { CLIENT = 0x000f, SERVER = 0x00f0,  NETWORK_ROLE_MASK = 0x00ff, TCP = 0x0f00, UDP = 0xf000, PROTOCOL_MASK = 0xff00 };

typedef int (CVICALLBACK *connection_callback)( unsigned, int, int, void* );

typedef struct _Async_Connection Async_Connection;

// Generic structure to store methods and data of any connection type handled by the library
struct _Async_Connection
{
  unsigned handle;
  struct
  {
    unsigned port;
    char host[ HOST_LENGTH ];
    char string[ ADDRESS_LENGTH ];
  } address;
  uint16_t type;
  union {
    char* buffer;
    Connection** client_list;
  };
  CmtTSQHandle read_queue, write_queue;
  size_t* ref_connections_count;
};



/////////////////////////////////////////////////////////////////////////////
/////                         NETWORK UTILITIES                         /////
/////////////////////////////////////////////////////////////////////////////

// Utility method to request an address (host and port) string for client connections (returns default values for server connections)
char* get_address( Connection* connection )
{
  static char address_string[ ADDRESS_LENGTH ];
  
  EVENT_DEBUG( "getting address string for connection handle: %u\n", connection->handle );
  
  sprintf( address_string, "%s/%u", connection->address.host, connection->address.port );
  
  return address_string;
}

// Returns number of active clients for a connection 
size_t connections_count( Connection* connection )
{
  size_t n_connections = 0;
  
  if( (connection->type & NETWORK_ROLE_MASK) == SERVER )
    n_connections = *(connection->ref_connections_count);
  else
    n_connections = 1;
  
  return n_connections;
}


//////////////////////////////////////////////////////////////////////////////////
/////                             INITIALIZATION                             /////
//////////////////////////////////////////////////////////////////////////////////

// Forward Declaration
static int CVICALLBACK accept_tcp_client( unsigned, int, int, void* );
static int CVICALLBACK accept_udp_client( unsigned, int, int, void* );
static int CVICALLBACK receive_tcp_message( unsigned, int, int, void* );
static int CVICALLBACK receive_udp_message( unsigned, int, int, void* );
static int CVICALLBACK send_tcp_messages( void* );
static int CVICALLBACK send_udp_messages( void* );
static int CVICALLBACK send_messages_all( void* );


// Asyncronous callback for ip connection setup and event processing
static int CVICALLBACK async_connect( void* connection_data );
{
  Async_Connection* connection = (Async_Connection*) connection_data;
  
  static int status_code;
  
  switch( connection->type )
  {
    case ( TCP | SERVER ):
      
      if( (status_code = RegisterTCPServer( connection->address.port, accept_tcp_client, connection )) < 0 )
        ERROR_PRINT( "%s: %s\n", GetTCPErrorString( status_code ), GetTCPSystemErrorString() );
      else
        connection->handle = 0;
      
      break;
      
    case ( TCP | CLIENT ):
      
      if( (status_code = ConnectToTCPServer( &(connection->handle), connection->address.port, connection->address.host, receive_tcp_message, connection, 100 )) < 0 )
        ERROR_PRINT( "%s: %s\n", GetTCPErrorString( status_code ), GetTCPSystemErrorString() );
      
      break;
      
    case ( UDP | SERVER ):
      
      if( (status_code = CreateUDPChannelConfig( connection->address.port, UDP_ANY_ADDRESS, 0, accept_udp_client, &(connection->handle) )) < 0 )
        ERROR_PRINT( "%s\n", GetUDPErrorString( status_code ) );
      
      break;
      
    case ( UDP | CLIENT ):
      
      if( (status_code = CreateUDPChannelConfig( UDP_ANY_LOCAL_PORT, UDP_ANY_ADDRESS, 0, receive_udp_message, &(connection->handle) )) < 0 )
        ERROR_PRINT( "%s\n", GetUDPErrorString( status_code ) );
      
      break;
  }
  
  if( status_code < 0 ) connection->handle = (unsigned int) -1;
  
  while( (signed int) connection->handle != -1 ) ProcessSystemEvents();
  
  CmtExitThreadPoolThread( status_code );
}

// Add defined connection to the client list of the given server connection
static INLINE void add_client( Connection* server, Connection* client )
{
  static size_t n_clients, client_index = 0, client_list_size = 0;
  
  free( client->ref_connections_count );
  client->ref_connections_count = server->ref_connections_count;
  
  n_clients = *(server->ref_connections_count);
  while( client_index < n_clients )
  {
    client_list_size++;
    if( server->client_list[ client_index ] != NULL )
      client_index++;
    else
      break;
  }
  
  if( client_index == client_list_size )
    server->client_list = (Connection**) realloc( server->client_list, ( client_index + 1 ) * sizeof(Connection*) );
  server->client_list[ client_index ] = client;
  
  (*(server->ref_connections_count))++;

  return;
}

// Generic method for opening a new socket and providing a corresponding Connection structure for use
Connection* open_connection( const char* host, const char* port, uint16_t protocol )
{
  static Connection* connection;

  static unsigned int port_number;
  
  // Assure that the port number is in the Dynamic/Private range (49152-65535)
  port_number = (unsigned int) strtoul( port, NULL, 0 );
  if( port_number < 49152 || port_number > 65535 )
  {
    fprintf( stderr, "%s: invalid port number value: %s\n", __func__, port );
    return NULL;
  }
  
  if( ( protocol != TCP ) && ( protocol != UDP ) )
  {
    ERROR_PRINT( "invalid protocol option: %x\n", protocol );
    return NULL;
  }
  
  connection = (Async_Connection*) malloc( sizeof(Async_Connection) );
  
  connection->address.port = port_number;
  strncpy( connection->address.host, ( host != NULL ) ? host : "0.0.0.0", HOST_LENGTH );
  sprintf( connection->address.string, "%s/%u", connection->address.host, connection->address.port );
  
  connection->type = ( ( host == NULL ) ? SERVER : CLIENT ) | protocol;
  
  static CmtThreadFunctionID connect_thread_id;
  
  CmtScheduleThreadPoolFunction( DEFAULT_THREAD_POOL_HANDLE, async_connect, connection, &connect_thread_id );
  
  CmtWaitForThreadPoolFunctionCompletionEx( DEFAULT_THREAD_POOL_HANDLE, connect_thread_id, 0, 1000 );
  
  if( (signed int) connection->handle != -1 )
  {
    free( connection );
    return NULL;
  }
  
  EVENT_DEBUG( "connection added: address: %s\n", connection->address.string );
  
  if( (connection->type & NETWORK_ROLE_MASK) == SERVER )
  {
    connection->client_list = NULL;
    CmtScheduleThreadPoolFunction( DEFAULT_THREAD_POOL_HANDLE, send_messages_all, connection, NULL );
    
    CmtNewTSQ( MAX_DATA, sizeof(Async_Connection), 0, connection->read_queue );
  }
  else
  {
    if( (connection->type & PROTOCOL_MASK) == TCP )
      CmtScheduleThreadPoolFunction( DEFAULT_THREAD_POOL_HANDLE, send_tcp_messages, connection, NULL );
    else
      CmtScheduleThreadPoolFunction( DEFAULT_THREAD_POOL_HANDLE, send_udp_messages, connection, NULL );
    
    CmtNewTSQ( MAX_DATA, MESSAGE_BUFFER_SIZE, 0, connection->read_queue );
  }

  CmtNewTSQ( MAX_DATA, MESSAGE_BUFFER_SIZE, OPT_TSQ_AUTO_FLUSH_EXACT, connection->write_queue );
  
  connection->ref_connections_count = (size_t*) malloc( sizeof(size_t) );
  *(connection->ref_connections_count) = 0;

  return connection;
}


/////////////////////////////////////////////////////////////////////////////////
/////                             COMMUNICATION                             /////
/////////////////////////////////////////////////////////////////////////////////

// Try to receive incoming message from the given TCP client connection and store it on its buffer
static int CVICALLBACK receive_tcp_message( unsigned int connection_handle, int event_type, int error_code, void* connection_data );
{
  Async_Connection* connection = (Async_Connection*) connection_data;
  
  char message_buffer[ MESSAGE_BUFFER_SIZE ];
  
  switch( event_type )
	{
		case TCP_DISCONNECT:
			
      close_connection( connection );
			break;
      
		case TCP_DATAREADY:
        
			if( (error_code = ClientTCPRead( connection_handle, message_buffer, MESSAGE_BUFFER_SIZE, 0 )) < 0 )
      {
        ERROR_PRINT( "%s: %s\n", GetTCPErrorString( error_code ), GetTCPSystemErrorString() );
        close_connection( connection );
        break;
      }
      
      if( (error_code = CmtWriteTSQData( connection->read_queue, message_buffer, 1, TSQ_INFINITE_TIMEOUT, NULL )) < 0 )
      {
        CmtGetErrorMessage( error_code, message_buffer );
        ERROR_PRINT( "%s\n", message_buffer );
        close_connection( connection );
        break;
      }
      
      LOOP_DEBUG( "connection handle %u received message: %s\n", connection->handle, connection->buffer );
		  
      break;
	}
  
  return 0;
}

// Send given message through the given TCP connection
static int CVICALLBACK send_tcp_messages( void* connection_data )
{
  Async_Connection* connection = (Async_Connection*) connection_data;
  
  int error_code;
  
  char message_buffer[ MESSAGE_BUFFER_SIZE ];
  
  while( (signed int) connection->handle != -1 )
  {
    if( (error_code = CmtReadTSQData( connection->write_queue, message_buffer, 1, TSQ_INFINITE_TIMEOUT, 0 )) < 0 )
    {
      CmtGetErrorMessage( error_code, message_buffer );
      ERROR_PRINT( "%s\n", message_buffer );
      close_connection( connection );
      break;
    }
      
  	if( (error_code = ClientTCPWrite( connection->handle, message_buffer, MESSAGE_BUFFER_SIZE, 0 )) < 0 )
    {
      ERROR_PRINT( "%s: %s\n", GetTCPErrorString( error_code ), GetTCPSystemErrorString() );
      close_connection( connection );
      break;
    }
  }
  
  CmtExitThreadPoolThread( error_code );
}

// Try to receive incoming message from the given UDP client connection and store it on its buffer
static int CVICALLBACK receive_udp_message( unsigned int connection_handle, int event_type, int error_code, void* connection_data );
{
  Async_Connection* connection = (Async_Connection*) connection_data;
  
  char message_buffer[ MESSAGE_BUFFER_SIZE ];
  
  switch( event_type )
	{
		case TCP_DISCONNECT:
			
      close_connection( connection );
			break;
      
		case TCP_DATAREADY:
        
			if( (error_code = ClientTCPRead( connection_handle, message_buffer, MESSAGE_BUFFER_SIZE, 0 )) < 0 )
      {
        ERROR_PRINT( "%s: %s\n", GetTCPErrorString( error_code ), GetTCPSystemErrorString() );
        close_connection( connection );
        break;
      }
      
      if( (error_code = CmtWriteTSQData( connection->read_queue, message_buffer, 1, TSQ_INFINITE_TIMEOUT, NULL )) < 0 )
      {
        CmtGetErrorMessage( error_code, message_buffer );
        ERROR_PRINT( "%s\n", message_buffer );
        close_connection( connection );
        break;
      }
      
      LOOP_DEBUG( "connection handle %u received message: %s\n", connection->handle, connection->buffer );
		  
      break;
	}

  memset( connection->buffer, 0, BUFFER_SIZE );
  // Blocks until there is something to be read in the socket
  if( recvfrom( connection->sockfd, connection->buffer, BUFFER_SIZE, MSG_PEEK, (struct sockaddr *) &address, &addr_len ) == SOCKET_ERROR )
  {
    print_socket_error( "receive_udp_message: recvfrom: error reading from socket" );
    return NULL;
  }

  // Verify if incoming message is destined to this connection (and returns the message if it is)
  if( connection->address->sin6_port == address.sin6_port )
  {
    address_string[0] = connection->address->sin6_addr.s6_addr;
    address_string[1] = address.sin6_addr.s6_addr;
    if( strncmp( (const char*) address_string[0], (const char*) address_string[1], sizeof(struct in6_addr) ) == 0 )
    {
      recv( connection->sockfd, NULL, 0, 0 );
    
      #ifdef DEBUG_2
      printf( "receive_udp_message: socket %d received right message: %s\n", connection->sockfd, connection->buffer );
      #endif
    
      return connection->buffer;
    }
  }
  
  // Default return message (the received one was not destined to this connection) 
  return 0; 
}

// Send given message through the given UDP connection
static int send_udp_message( Connection* connection, const char* message )
{
  if( strlen( message ) + 1 > BUFFER_SIZE )
  {
    fprintf( stderr, "send_udp_message: message too long !\n" );
    return 0;
  }
  
  #ifdef DEBUG_2
  printf( "send_udp_message: connection socket %d sending message: %s\n", connection->sockfd, message );
  #endif
  
  if( sendto( connection->sockfd, message, BUFFER_SIZE, 0, (struct sockaddr *) connection->address, sizeof(struct sockaddr_in6) ) == SOCKET_ERROR )
  {
    print_socket_error( "send_udp_message: sendto: error writing to socket" );
    return -1;
  }
  
  return 0;
}

// Send given message to all the clients of the given server connection
static int send_message_all( Connection* connection, const char* message )
{
  static size_t client_id = 0, n_clients;
  n_clients = *(connection->ref_connections_count);
  while( client_id < n_clients )
  {
    if( connection->client_list[ client_id ] != NULL )
    {
      send_message( connection->client_list[ client_id ], message );
      client_id++;
    }
  }
  
  return 0;
}

// Waits for a remote connection to be added to the client list of the given TCP server connection
static Connection* accept_tcp_client( Connection* server )
{
  Connection* client;
  int client_sockfd;
  static struct sockaddr_storage client_address;
  static socklen_t addr_len = sizeof(client_address);
  
  client_sockfd = accept( server->sockfd, (struct sockaddr *) &client_address, &addr_len );

  if( client_sockfd == INVALID_SOCKET )
  {
    print_socket_error( "accept_tcp_client: accept: error accepting connection" );
    return NULL;
  }
  
  #ifdef DEBUG_1
  printf( "accept_tcp_client: client accepted: socket fd: %d\n", client_sockfd );
  #endif
  
  client =  add_connection( client_sockfd, (struct sockaddr *) &client_address, CLIENT | TCP );

  add_client( server, client );

  return client;
}

// Waits for a remote connection to be added to the client list of the given UDP server connection
static Connection* accept_udp_client( Connection* server )
{
  size_t client_id, n_clients;
  Connection* client;
  static struct sockaddr_storage client_address;
  static socklen_t addr_len = sizeof(client_address);
  static char buffer[ BUFFER_SIZE ];
  static uint8_t* address_string[2];
  static Connection dummy_connection;
  int bytes_received;

  if( recvfrom( server->sockfd, buffer, BUFFER_SIZE, MSG_PEEK, (struct sockaddr *) &client_address, &addr_len ) == SOCKET_ERROR )
  {
    print_socket_error( "accept_udp_client: recvfrom: error reading from socket" );
    return NULL;
  }
  
  // Verify if incoming message belongs to unregistered client (returns default value if not)
  n_clients = *(server->ref_connections_count);
  for( client_id = 0; client_id < n_clients; client_id++ )
  {
    #ifdef DEBUG_2
    printf( "accept_udp_client: comparing ports: %u - %u\n", server->client_list[ client_id ]->address->sin6_port, ((struct sockaddr_in6*) &client_address)->sin6_port );
    #endif
    if( server->client_list[ client_id ]->address->sin6_port == ((struct sockaddr_in6*) &client_address)->sin6_port )
    {
      address_string[0] = server->client_list[ client_id ]->address->sin6_addr.s6_addr;
      address_string[1] = ((struct sockaddr_in6*) &client_address)->sin6_addr.s6_addr;
      if( strncmp( (const char*) address_string[0], (const char*) address_string[1], sizeof(struct in6_addr) ) == 0 )
        return &dummy_connection;
    }
  }
  
  #ifdef DEBUG_1
  printf( "accept_udp_client: client accepted\n" );
  
  printf( "\tdatagram clients before: %u\n", *(server->ref_connections_count) );
  #endif
  
  client = add_connection( server->sockfd, (struct sockaddr*) &client_address, CLIENT | UDP );

  add_client( server, client );
  
  #ifdef DEBUG_1
  printf( "\tdatagram clients after: %u\n",  *(server->ref_connections_count) );
  #endif
  
  return client;
}


//////////////////////////////////////////////////////////////////////////////////
/////                               FINALIZING                               /////
//////////////////////////////////////////////////////////////////////////////////

// Handle proper destruction of any given connection type
void close_connection( Connection* connection )
{
  if( connection != NULL )
  {
    #ifdef DEBUG_1
    printf( "close_connection: closing connection for socket %d\n", connection->sockfd );
    printf( "close_connection: connection users count: %u\n", *(connection->ref_connections_count) );
    #endif

    // Each TCP connection has its own socket, so we can close it without problem. But UDP connections
    // from the same server share the socket, so we need to wait for all of them to be stopped to close the socket
    if( (connection->type & PROTOCOL_MASK) == TCP )
    {
      #ifdef DEBUG_1
      printf( "close_connection: closing TCP connection unused socket fd: %d\n", connection->sockfd );
      #endif
      FD_CLR( connection->sockfd, &socket_read_fds );
      shutdown( connection->sockfd, SHUT_RDWR );
    }
    // Check number of client connections of a server (also of sharers of a socket for UDP connections)
    if( *(connection->ref_connections_count) <= 0 )
    {
      if( (connection->type & PROTOCOL_MASK) == UDP )
      {
        #ifdef DEBUG_1
        printf( "close_connection: closing UDP connection unused socket fd: %d\n", connection->sockfd );
        #endif
        FD_CLR( connection->sockfd, &socket_read_fds );
        close( connection->sockfd );
      }
      free( connection->ref_connections_count );
    }
    else
      (*(connection->ref_connections_count))--;
  
    free( connection->address );
    connection->address = NULL;
  
    if( (connection->type & NETWORK_ROLE_MASK) == CLIENT )
    {
      #ifdef DEBUG_1
      printf( "close_connection: freeing client connection message buffer\n" );
      #endif
      free( connection->buffer );
      connection->buffer = NULL;
    }
    else if( (connection->type & NETWORK_ROLE_MASK) == SERVER )
    {
      #ifdef DEBUG_1
      printf( "close_connection: cleaning server connection client list\n" );
      #endif
      if( connection->client_list != NULL )
      {
        free( connection->client_list );
        connection->client_list = NULL;
      }
    }
  
    free( connection );
    connection = NULL;
  }

  return;
}

#ifdef __cplusplus
    }
#endif

#endif  /* CONNECT_H */
