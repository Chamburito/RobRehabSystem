/////////////////////////////////////////////////////////////////////////////////////
///// Multiplatform library for creation and handling of IP sockets connections /////
///// as server or client, using TCP or UDP protocols                           /////
/////////////////////////////////////////////////////////////////////////////////////

#ifndef CONNECT_H
#define CONNECT_H

#ifdef __cplusplus
extern "C"{
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
  
#ifdef WIN32
  
  #include <winsock2.h>
  #include <ws2tcpip.h>
  #include <stdint.h>
  #include <time.h>
  
  #pragma comment(lib, "Ws2_32.lib")
  
  #define SHUT_RDWR SD_BOTH
  #define close( i ) closesocket( i )

  typedef SOCKET Socket;
  
#else
  
  #include <unistd.h>
  #include <errno.h>
  #include <sys/types.h>
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <arpa/inet.h>
  #include <netdb.h>

  #define SOCKET_ERROR -1
  #define INVALID_SOCKET -1

  typedef int Socket;
  
#endif

/////////////////////////////////////////////////////////////////////////  
/////                    PREPROCESSOR DIRECTIVES                    /////
/////////////////////////////////////////////////////////////////////////

#ifndef __cplusplus
  #if __STDC_VERSION__ >= 199901L
    /* "inline" is a keyword */
    #include <stdbool.h>
  #else
    #define inline //static
    typedef uint8_t bool;
    #define true 1
    #define false 0
  #endif
#endif

#define QUEUE_SIZE 20
#define BUFFER_SIZE 256

#define HOST_LENGTH 40
#define PORT_LENGTH 6
#define ADDRESS_LENGTH (HOST_LENGTH + PORT_LENGTH)

#define HOST 0
#define PORT HOST_LENGTH

#define CLIENT 0x000f
#define SERVER 0x00f0
#define NETWORK_ROLE 0x00ff
#define TCP 0x0f00
#define UDP 0xf000
#define PROTOCOL 0xff00

//////////////////////////////////////////////////////////////////////////
/////                         DATA STRUCTURES                        /////
//////////////////////////////////////////////////////////////////////////

typedef struct _Connection Connection;

// Generic structure to store methods and data of any connection type handled by the library
struct _Connection
{
  Socket sockfd;
  union {
    char* (*receive_message)( Connection* );
    Connection* (*accept_client)( Connection* );
  };
  int (*send_message)( Connection*, const char* );
  struct sockaddr_in6* address;
  uint16_t type;
  union {
    char* buffer;
    Connection** client_list;
  };
  size_t* ref_connections_count;
};


//////////////////////////////////////////////////////////////////////////
/////                        GLOBAL VARIABLES                        /////
//////////////////////////////////////////////////////////////////////////

static fd_set socket_read_fds; // stores all the socket file descriptors in use (for performance when verifying incoming messages) 


///////////////////////////////////////////////////////////////////////////
/////                         DEBUG UTILITIES                         /////
///////////////////////////////////////////////////////////////////////////

// Platform specific error printing
inline void print_socket_error( const char* message )
{
  #ifdef WIN32
  fprintf( stderr, "%s: code: %d\n", message, WSAGetLastError() );
  #else
  perror( message );
  #endif
}


/////////////////////////////////////////////////////////////////////////////
/////                         NETWORK UTILITIES                         /////
/////////////////////////////////////////////////////////////////////////////

// Utility method to request an address (host and port) string for client connections (returns default values for server connections)
char* get_address( Connection* connection )
{
  static char address_string[ ADDRESS_LENGTH ];
  static int error = 0;
  
  #ifdef DEBUG_1
  printf( "get_address: getting address string for socket fd: %d\n", connection->sockfd );
  #endif
  
  error = getnameinfo( (struct sockaddr*) (connection->address), sizeof(struct sockaddr_in6), 
                    &address_string[ HOST ], HOST_LENGTH, &address_string[ PORT ], PORT_LENGTH, NI_NUMERICHOST | NI_NUMERICSERV );

  if( error != 0 )
  {
    #ifdef WIN32
    fprintf( stderr, "get_address: getnameinfo: error getting address string: code: %d\n", WSAGetLastError() );
    #else
    fprintf( stderr, "get_address: getnameinfo: error getting address string: %s\n", gai_strerror( error ) );
    #endif
    return NULL;
  }
  
  return address_string;
}

// More complete (but slow) method for verifying the remote address of incoming messages
static int compare_addresses( struct sockaddr* addr_1, struct sockaddr* addr_2 )
{
  static char address_strings[ 2 ][ ADDRESS_LENGTH ];
  static int error = 0;
  
  error = getnameinfo( addr_1, sizeof(struct sockaddr_in6), &address_strings[ 0 ][ HOST ], HOST_LENGTH, &address_strings[ 0 ][ PORT ], PORT_LENGTH, NI_NUMERICHOST | NI_NUMERICSERV );
  if( error != 0 )
  {
    fprintf( stderr, "compare_addresses: getnameinfo: error getting address 1 string: %s\n", gai_strerror( error ) );
    return -1;
  }
  
  error = getnameinfo( addr_2, sizeof(struct sockaddr_in6), &address_strings[ 1 ][ HOST ], HOST_LENGTH, &address_strings[ 1 ][ PORT ], PORT_LENGTH, NI_NUMERICHOST | NI_NUMERICSERV );
  if( error != 0 )
  {
    fprintf( stderr, "compare_addresses: getnameinfo: error getting address 2 string: %s\n", gai_strerror( error ) );
    return -1;
  }
  
  if( strcmp( (const char*) &address_strings[ 0 ][ PORT ], (const char*) &address_strings[ 1 ][ PORT ] ) == 0 )
    return ( strcmp( (const char*) &address_strings[ 0 ][ HOST ], (const char*) &address_strings[ 1 ][ HOST ] ) );
  else
    return -1;
}

// Returns number of active clients for a connection 
size_t connections_count( Connection* connection )
{
  size_t n_connections = 0;
  
  if( (connection->type & NETWORK_ROLE) == SERVER )
    n_connections = *(connection->ref_connections_count);
  else
    n_connections = 1;
  
  return n_connections;
}


//////////////////////////////////////////////////////////////////////////////////
/////                             INITIALIZATION                             /////
//////////////////////////////////////////////////////////////////////////////////

// Forward Declaration
static char* receive_tcp_message( Connection* );
static char* receive_udp_message( Connection* );
static int send_tcp_message( Connection*, const char* );
static int send_udp_message( Connection*, const char* );
static int send_message_all( Connection*, const char* );
static Connection* accept_tcp_client( Connection* );
static Connection* accept_udp_client( Connection* );


// Handle construction of a Connection structure with the defined properties
static Connection* add_connection( int sockfd, struct sockaddr* address, uint16_t type )
{
  int opt = BUFFER_SIZE;
  Connection* connection = (Connection*) malloc( sizeof(Connection) );
  connection->sockfd = sockfd;
  connection->type = type;

  #ifdef DEBUG_1
  printf( "add_connection: socket: %d - type: %x\n", connection->sockfd, connection->type );
  #endif
  
  connection->address = (struct sockaddr_in6*) malloc( sizeof(struct sockaddr_in6) );
  *(connection->address) = *((struct sockaddr_in6*) address);
  connection->ref_connections_count = (size_t*) malloc( sizeof(size_t) );
  *(connection->ref_connections_count) = 0;
  
  if( (connection->type & NETWORK_ROLE) == SERVER ) // Server role connection
  {
    connection->client_list = NULL;
    connection->accept_client = ( (connection->type & PROTOCOL) == TCP ) ? accept_tcp_client : accept_udp_client;
    connection->send_message = send_message_all;
  }
  else
  {
    /*if( setsockopt( connection->sockfd, SOL_SOCKET, SO_RCVLOWAT, (const char*) &opt, sizeof(opt) ) == SOCKET_ERROR )
    {
      perror( "add_connection: error setting socket option SO_RCVLOWAT" );
      return NULL;
    }*/
    // Not available
    /*if( setsockopt( connection->sockfd, SOL_SOCKET, SO_SNDLOWAT, (const char*) &opt, sizeof(opt) ) == SOCKET_ERROR )
    {
      perror( "add_connection: error setting socket option SO_SNDLOWAT" );
      return NULL;
    }*/
      
    #ifdef DEBUG_1
    printf( "add_connection: connection added:\n" );
    printf( "\tfamily: %u\n", connection->address->sin6_family );
    printf( "\tport: %u\n", connection->address->sin6_port );
    printf( "\tscope: %u\n", connection->address->sin6_scope_id );
    printf( "\tflow info: %u\n", connection->address->sin6_flowinfo );
    printf( "\taddress: " );
    for( opt = 0; opt < 16; opt++ )
      printf( "%u", connection->address->sin6_addr.s6_addr[ opt ] );
    printf( "\n" );
    #endif
    //connection->address->sin6_family = AF_INET6;
    connection->buffer = (char*) calloc( BUFFER_SIZE, sizeof(char) );
    connection->receive_message = ( (connection->type & PROTOCOL) == TCP ) ? receive_tcp_message : receive_udp_message;
    connection->send_message = ( (connection->type & PROTOCOL) == TCP ) ? send_tcp_message : send_udp_message;
  }
  
  return connection;
}

// Add defined connection to the client list of the given server connection
static inline void add_client( Connection* server, Connection* client )
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
Connection* open_connection( const char* host, const char* port, int protocol )
{
  static Connection* connection;

  static int sockfd;
  static struct addrinfo hints;
  static struct addrinfo* host_info;
  static struct addrinfo* p;

  static int rw;
  static uint16_t connection_type;
  
  #ifdef WIN32
  WSADATA wsa;
  if( WSAStartup( MAKEWORD(2,2), &wsa ) != 0 )
  {
    fprintf( stderr, "open_connection: error initialiasing windows sockets: code: %d", WSAGetLastError() );
    return NULL;
  }
  #endif

  memset( &hints, 0, sizeof(hints) );
  hints.ai_family = AF_UNSPEC; // AF_INET6 (IPv6), AF_INET (IPv4) or AF_UNSPEC

  if( protocol == TCP )
    hints.ai_socktype = SOCK_STREAM;
  else if( protocol == UDP )
    hints.ai_socktype = SOCK_DGRAM;
  else
  {
    fprintf( stderr, "open_connection: invalid protocol option: %x\n", protocol );
    return NULL;
  }

  if( host == NULL ) 
  {
    hints.ai_flags |= AI_PASSIVE; // Set address for me
    connection_type = SERVER;
  }
  else
    connection_type = CLIENT;

  connection_type |= protocol;

  if( (rw = getaddrinfo( host, port, &hints, &host_info )) != 0 )
  {
    #ifdef WIN32
    fprintf( stderr, "open_connection: getaddrinfo: error reading host info: code: %d\n", WSAGetLastError() );
    #else
    fprintf( stderr, "open_connection: getaddrinfo: error reading host info: %s\n", gai_strerror( rw ) );
    #endif
    return NULL;
  }

  // loop through all the results and bind to the first we can
  for( p = host_info; p != NULL; p = p->ai_next ) 
  {
    // Extended connection info for debug builds
    #ifdef DEBUG_1
    char address_string[ ADDRESS_LENGTH ];
    getnameinfo( p->ai_addr, sizeof(struct sockaddr), 
		 &address_string[ HOST ], HOST_LENGTH, &address_string[ PORT ], PORT_LENGTH, NI_NUMERICHOST | NI_NUMERICSERV );
    if( (connection_type & NETWORK_ROLE) == SERVER )
      printf( "open_connection: trying to bind to: host: %s - port: %s", &address_string[ HOST ], &address_string[ PORT ] );
    else
      printf( "open_connection: trying to connect to: host: %s - port: %s", &address_string[ HOST ], &address_string[ PORT ] );
    if( p->ai_protocol == IPPROTO_TCP ) printf( " - protocol: TCP" );
    else if( p->ai_protocol == IPPROTO_UDP ) printf( " - protocol: UDP" );
    else printf( " - protocol: invalid: %u", p->ai_protocol );
    if( p->ai_family == AF_INET ) printf( " - version: IPV4\n" );
    else if( p->ai_family == AF_INET6 ) printf( " - version: IPV6\n" );
    else printf( " - version: unspecified: %u\n", p->ai_family );
    #endif

    // Create IP socket
    if( (sockfd = socket( p->ai_family, p->ai_socktype, p->ai_protocol )) == INVALID_SOCKET )
    {
      print_socket_error( "open_connection: socket: error opening socket" );
      continue;
    }
    
    #ifdef WIN32
	rw = 1;
	if( ioctlsocket( sockfd, FIONBIO, (u_long*) &rw ) == SOCKET_ERROR )
	{
	  fprintf( stderr, "open_connection: ioctlsocket: error setting socket option FIONBIO: code: %d\n", WSAGetLastError() );
      close( sockfd );
      return NULL;
    }
    #else
	fcntl( sockfd, F_SETFL, O_NONBLOCK );
    #endif

    rw = 1;
    // Allow sockets to be binded to the same local port
    if( setsockopt( sockfd, SOL_SOCKET, SO_REUSEADDR, (const char*) &rw, sizeof(rw) ) == SOCKET_ERROR ) 
    {
      print_socket_error( "open_connection: setsockopt: error setting socket option SO_REUSEADDR" );
      close( sockfd );
      return NULL;
    }
    
    if( (connection_type & NETWORK_ROLE) == SERVER )
    {
      if( p->ai_family == AF_INET ) continue;
      
      rw = 0;
      // Let IPV6 servers accept IPV4 clients
      if( setsockopt( sockfd, IPPROTO_IPV6, IPV6_V6ONLY, (const char*) &rw, sizeof(rw) ) == SOCKET_ERROR )
      {
        print_socket_error( "open_connection: setsockopt: error setting socket option IPV6_V6ONLY" );
        close( sockfd );
        return NULL;
      }
      
      // Bind server socket to the given local address
      if( bind( sockfd, p->ai_addr, p->ai_addrlen ) == SOCKET_ERROR )
      {
        print_socket_error( "open_connection: bind: error on binding" );
        close( sockfd );
        continue;
      }
      
      if( p->ai_socktype == SOCK_STREAM )
      {
        // Set server socket to listen to remote connections
        if( listen( sockfd, QUEUE_SIZE ) == SOCKET_ERROR )
        {
          print_socket_error( "open_connection: listen: error on listening" );
          close( sockfd );
          continue;
        }
      }
    }
    else if( (connection_type & PROTOCOL) == TCP )
    {
      // Connect TCP client socket to given remote address
      if( connect( sockfd, p->ai_addr, p->ai_addrlen ) == SOCKET_ERROR )
      {
        print_socket_error( "open_connection: connect: error on connecting" );
        close( sockfd );
        continue;
      }
    }
    else
    {
      // Bind UDP client socket to available local address
      static struct sockaddr_storage local_address;
      local_address.ss_family = p->ai_addr->sa_family;
      if( bind( sockfd, (struct sockaddr*) &local_address, sizeof(local_address) ) == SOCKET_ERROR )
      {
        print_socket_error( "open_connection: bind: error on binding" );
        close( sockfd );
        continue;
      }
    }
    
    break;
  }
  
  if( p == NULL )  
  {
    fprintf( stderr, "open_connection: failed to create socket\n" );
    return NULL;
  }
  
  connection = add_connection( sockfd, p->ai_addr, connection_type ); // Build the Connection structure
  
  freeaddrinfo( host_info ); // Don't need this struct anymore
  
  return connection;
}


/////////////////////////////////////////////////////////////////////////////////
/////                             COMMUNICATION                             /////
/////////////////////////////////////////////////////////////////////////////////

// Wrapper functions
inline char* receive_message( Connection* c ) { return c->receive_message( c ); }
inline int send_message( Connection* c, const char* message ) { return c->send_message( c, message ); }
inline Connection* accept_client( Connection* c ) { return c->accept_client( c ); }

// Verify available incoming messages for the given connection, preventing unnecessary blocking calls (for syncronous networking)
short wait_message( Connection* connection, unsigned int milisseconds )
{
  fd_set read_fds;
  struct timeval timeout;
  int n_changes;

  FD_ZERO( &read_fds );
  FD_SET( connection->sockfd, &read_fds );

  timeout.tv_sec = milisseconds / 1000;
  timeout.tv_usec =  1000 * ( milisseconds % 1000 );
  
  n_changes = select( connection->sockfd + 1, &read_fds, NULL, NULL, &timeout );
  if( n_changes == SOCKET_ERROR )
  {
    print_socket_error( "wait_message: select: error waiting for message" );
    return -1;
  }
  else if( n_changes == 0 )
    return 0;
  
  if( FD_ISSET( connection->sockfd, &read_fds ) )
    return 1;
  else
    return 0;
}

// Try to receive incoming message from the given TCP client connection and store it on its buffer
static char* receive_tcp_message( Connection* connection )
{
  int bytes_received;
  
  memset( connection->buffer, 0, BUFFER_SIZE );
  // Blocks until there is something to be read in the socket
  bytes_received = recv( connection->sockfd, connection->buffer, BUFFER_SIZE, 0 );

  if( bytes_received == SOCKET_ERROR )
  {
    print_socket_error( "receive_tcp_message: recv: error reading from socket" );
    return NULL;
  }
  else if( bytes_received == 0 )
  {
    fprintf( stderr, "receive_tcp_message: recv: remote connection closed\n" );
    return NULL;
  }
  
  #ifdef DEBUG_2
  printf( "receive_tcp_message: socket %d received message: %s\n", connection->sockfd, connection->buffer );
  #endif
  
  return connection->buffer;
}

// Send given message through the given TCP connection
static int send_tcp_message( Connection* connection, const char* message )
{
  if( strlen( message ) + 1 > BUFFER_SIZE )
  {
    fprintf( stderr, "send_tcp_message: message too long !\n" );
    return 0;
  }
  
  if( send( connection->sockfd, message, BUFFER_SIZE, 0 ) == SOCKET_ERROR )
  {
    print_socket_error( "send_tcp_message: send: error writing to socket" );
    return -1;
  }
  
  return 0;
}

// Try to receive incoming message from the given UDP client connection and store it on its buffer
static char* receive_udp_message( Connection* connection )
{
  struct sockaddr_in6 address;
  socklen_t addr_len = sizeof(struct sockaddr_storage);
  static uint8_t* address_string[2];
  static char* empty_message = { '\0' };

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
  return empty_message;
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
    if( (connection->type & PROTOCOL) == TCP )
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
      if( (connection->type & PROTOCOL) == UDP )
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
  
    if( (connection->type & NETWORK_ROLE) == CLIENT )
    {
      #ifdef DEBUG_1
      printf( "close_connection: freeing client connection message buffer\n" );
      #endif
      free( connection->buffer );
      connection->buffer = NULL;
    }
    else if( (connection->type & NETWORK_ROLE) == SERVER )
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

#endif /* CONNECT_H */