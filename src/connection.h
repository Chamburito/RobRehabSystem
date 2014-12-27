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

  
/* Preprocessor Directives */

#define receive_message( c ) ( c )->receive_message( c )
#define send_message( c, message ) ( c )->send_message( c, message )
#define accept_client( c ) ( c )->accept_client( c )

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
  uint16_t type;
  union {
    char* buffer;
    Connection** client_list;
  };
  union {
    struct sockaddr_in6* address;
    size_t n_clients;
  };
};

// Utility method to request an address (host and port) string for client connections (returns default values for server connections)
char* get_address( Connection* connection )
{
  static char address_string[ ADDRESS_LENGTH ];
  static int error = 0;
  
  #ifdef DEBUG_1
  printf( "get_address: getting address string for socket fd: %d\n", connection->sockfd );
  #endif
  
  if( (connection->type & NETWORK_ROLE) == CLIENT )
    error = getnameinfo( (struct sockaddr*) (connection->address), sizeof(struct sockaddr_in6), 
	              &address_string[ HOST ], HOST_LENGTH, &address_string[ PORT ], PORT_LENGTH, NI_NUMERICHOST | NI_NUMERICSERV );
  else
    strcpy( address_string, "Listening (0.0.0.0 or [::])" );

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
    #ifdef WIN32
    fprintf( stderr, "compare_addresses: getnameinfo: error getting address 1 string: code: %d\n", WSAGetLastError() );
    #else
    fprintf( stderr, "compare_addresses: getnameinfo: error getting address 1 string: %s\n", gai_strerror( error ) );
    #endif
    return -1;
  }
  
  error = getnameinfo( addr_2, sizeof(struct sockaddr_in6), &address_strings[ 1 ][ HOST ], HOST_LENGTH, &address_strings[ 1 ][ PORT ], PORT_LENGTH, NI_NUMERICHOST | NI_NUMERICSERV );
  if( error != 0 )
  {
    #ifdef WIN32
    fprintf( stderr, "compare_addresses: getnameinfo: error getting address 2 string: code: %d\n", WSAGetLastError() );
    #else
    fprintf( stderr, "compare_addresses: getnameinfo: error getting address 2 string: %s\n", gai_strerror( error ) );
    #endif
    return -1;
  }
  
  if( strcmp( (const char*) &address_strings[ 0 ][ PORT ], (const char*) &address_strings[ 1 ][ PORT ] ) == 0 )
    return ( strcmp( (const char*) &address_strings[ 0 ][ HOST ], (const char*) &address_strings[ 1 ][ HOST ] ) );
  else
    return -1;
}

// Verify available incoming messages for the given connection, preventing unnecessary blocking calls (for syncronous networking)
unsigned short wait_message( Connection* connection, unsigned int milisseconds )
{
  static struct timeval timeout;
  static fd_set read_fds;
  
  timeout.tv_sec = milisseconds / 1000;
  timeout.tv_usec =  1000 * ( milisseconds % 1000 );
  
  FD_ZERO( &read_fds );
  FD_SET( connection->sockfd, &read_fds );
  
  if( select( connection->sockfd + 1, &read_fds, NULL, NULL, &timeout ) == SOCKET_ERROR )
  {
    #ifdef WIN32
    fprintf( stderr, "wait_message: select: error waiting for message: code: %d\n", WSAGetLastError() );
    #else
    perror( "wait_message: select: error waiting for message" );
    #endif
    return 0;
  }
  
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
  if( (bytes_received = recv( connection->sockfd, connection->buffer, BUFFER_SIZE, 0 )) < BUFFER_SIZE )
  {
    if( bytes_received == SOCKET_ERROR )
    {
      #ifdef WIN32
      fprintf( stderr, "receive_tcp_message: recv: error reading from socket: code: %d\n", WSAGetLastError() );
      #else
      perror( "receive_tcp_message: recv: error reading from socket" );
      #endif
      return NULL;
    }
    else if( bytes_received == 0 )
    {
      fprintf( stderr, "receive_tcp_message: recv: remote connection closed\n" );
      return NULL;
    }
  }
  
  #ifdef DEBUG_1
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
    #ifdef WIN32
    fprintf( stderr, "send_tcp_message: send: error writing to socket: code: %d\n", WSAGetLastError() );
    #else
    perror( "send_tcp_message: send: error writing to socket" );
    #endif
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
    #ifdef WIN32
    fprintf( stderr, "receive_udp_message: recvfrom: error reading from socket: code: %d\n", WSAGetLastError() );
    #else
    perror( "receive_udp_message: recvfrom: error reading from socket" );
    #endif
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
    
      #ifdef DEBUG_1
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
  
  #ifdef DEBUG_1
  printf( "send_udp_message: connection socket %d sending message: %s\n", connection->sockfd, message );
  #endif
  
  if( sendto( connection->sockfd, message, BUFFER_SIZE , 0, (struct sockaddr *) connection->address, sizeof(struct sockaddr_in6) ) == SOCKET_ERROR )
  {
    #ifdef WIN32
    fprintf( stderr, "send_udp_message: sendto: error writing to socket: code: %d\n", WSAGetLastError() );
    #else
    perror( "send_udp_message: sendto: error writing to socket" );
    #endif
    return -1;
  }
  
  return 0;
}

// Send given message to all the clients of the given server connection
static int send_message_all( Connection* connection, const char* message )
{
  static size_t client_id;
  static Connection* client;
  for( client_id = 0; client_id < connection->n_clients; client_id++ )
  {
    client = connection->client_list[ client_id ];
    send_message( client, message );
  }
  
  return 0;
}

// Forward declaration
static Connection* add_connection( int, struct sockaddr*, unsigned short );

// Add defined connection to the client list of the given server connection
void add_client( Connection* server, Connection* client )
{
  #ifdef DEBUG_1
  printf( "add_client: connection type: server: %x - client: %x\n", server->type, client->type );
  #endif
  
  if( (server->type & NETWORK_ROLE) != SERVER )
  {
    fprintf( stderr, "add_client: first argument is not a server connection\n" );
    return;
  }
  
  if( (server->type & PROTOCOL) != (client->type & PROTOCOL) )
  {
    fprintf( stderr, "add_client: unmatching protocols\n" );
    return;
  }
  
  if( (client->type & NETWORK_ROLE) != CLIENT )
    fprintf( stderr, "add_client: warning: second argument is not a client connection\n" );
  
  if( (client->type & PROTOCOL) == UDP && (client->sockfd != server->sockfd) )
  {
    close( client->sockfd );
    client->sockfd = server->sockfd;
  }
  
  server->client_list = (Connection**) realloc( server->client_list, ( server->n_clients + 1 ) * sizeof(Connection*) );
  server->client_list[ server->n_clients ] = client;
  server->n_clients++;

  return;
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
    #ifdef WIN32
    fprintf( stderr, "accept_tcp_client: accept: error accepting connection: code: %d\n", WSAGetLastError() );
    #else
    perror( "accept_tcp_client: accept: error accepting connection" );
    #endif
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
  size_t client_id;
  Connection* client;
  static struct sockaddr_storage client_address;
  static socklen_t addr_len = sizeof(client_address);
  static char buffer[ BUFFER_SIZE ];
  static uint8_t* address_string[2];
  static Connection dummy_connection;
	
  if( recvfrom( server->sockfd, buffer, BUFFER_SIZE, MSG_PEEK, (struct sockaddr *) &client_address, &addr_len ) == SOCKET_ERROR )
  {
    #ifdef WIN32
    fprintf( stderr, "accept_udp_client: recvfrom: error reading from socket: code: %d\n", WSAGetLastError() );
    #else
    perror( "accept_udp_client: recvfrom: error reading from socket" );
    #endif
    return NULL;
  }
  
  // Verify if incoming message belongs to unregistered client (returns default value if not)
  for( client_id = 0; client_id < server->n_clients; client_id++ )
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
  
  printf( "\tdatagram clients before: %d\n", server->n_clients );
  #endif
  
  client = add_connection( server->sockfd, (struct sockaddr*) &client_address, CLIENT | UDP );

  add_client( server, client );
  
  #ifdef DEBUG_1
  printf( "\tdatagram clients after: %d\n",  server->n_clients );
  #endif
  
  return client;
}

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
  
  if( (connection->type & NETWORK_ROLE) == SERVER ) // Server role connection
  {
    connection->client_list = NULL;
    connection->n_clients = 0;
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
      
    connection->address = (struct sockaddr_in6*) malloc( sizeof(struct sockaddr_in6) );
    *(connection->address) = *((struct sockaddr_in6*) address);
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
    fprintf( stderr, "open_connection: getaddrinfo: error reading host info: %s\n", gai_strerror(rw) );
    #endif
    return NULL;
  }

  // loop through all the results and bind to the first we can
  for(p = host_info; p != NULL; p = p->ai_next) 
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
      #ifdef WIN32
      fprintf( stderr, "open_connection: socket: error opening socket: code: %d\n", WSAGetLastError() );
      #else
      perror( "open_connection: socket: error opening socket" );
      #endif
      continue;
    }
    
    rw = 1;
	// Allow sockets to be binded to the same local port
    if( setsockopt( sockfd, SOL_SOCKET, SO_REUSEADDR, (const char*) &rw, sizeof(rw) ) == SOCKET_ERROR ) 
    {
      #ifdef WIN32
      fprintf( stderr, "open_connection: setsockopt: error setting socket options SO_REUSEADDR: code: %d\n", WSAGetLastError() );
      #else
      perror( "open_connection: setsockopt: error setting socket options SO_REUSEADDR" );
      #endif
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
        #ifdef WIN32
        fprintf( stderr, "open_connection: setsockopt: error setting socket options IPV6_V6ONLY: code: %d\n", WSAGetLastError() );
        #else
        perror( "open_connection: setsockopt: error setting socket options IPV6_V6ONLY" );
        #endif
        close( sockfd );
        return NULL;
      }
      
	  // Bind server socket to the given local address
      if( bind( sockfd, p->ai_addr, p->ai_addrlen ) == SOCKET_ERROR )
      {
        #ifdef WIN32
        fprintf( stderr, "open_connection: bind: error on binding: code: %d\n", WSAGetLastError() );
        #else
        perror( "open_connection: bind: error on binding" );
        #endif
        close( sockfd );
        continue;
      }
      
      if( p->ai_socktype == SOCK_STREAM )
      {
		// Set server socket to listen to remote connections
        if( listen( sockfd, QUEUE_SIZE ) == SOCKET_ERROR )
        {
          #ifdef WIN32
          fprintf( stderr, "open_connection: listen: error on listening: code: %d\n", WSAGetLastError() );
          #else
          perror( "open_connection: listen: error on listening" );
          #endif
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
        #ifdef WIN32
        fprintf( stderr, "open_connection: connect: error on connecting: code: %d\n", WSAGetLastError() );
        #else
        perror( "open_connection: connect: error on connecting" );
        #endif
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
        #ifdef WIN32
        fprintf( stderr, "open_connection: bind: error on binding: code: %d\n", WSAGetLastError() );
        #else
        perror( "open_connection: bind: error on binding" );
        #endif
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

// Handle proper destruction of any given connection type
void close_connection( Connection* connection )
{
  if( connection != NULL )
  {
    printf( "close_connection: closing connection for socket %d\n", connection->sockfd );
    if( (connection->type & PROTOCOL) == TCP ) shutdown( connection->sockfd, SHUT_RDWR );
    close( connection->sockfd );
  
    if( (connection->type & NETWORK_ROLE) == CLIENT )
    {
      free( connection->buffer );
	  connection->buffer = NULL;
      free( connection->address );
	  connection->address = NULL;
    }
    else if( (connection->type & NETWORK_ROLE) == SERVER )
    {
      free( connection->client_list );
	  connection->client_list = NULL;
    }
  
    free( connection );
  }

  return;
}

#ifdef __cplusplus
}
#endif

#endif /* CONNECT_H */