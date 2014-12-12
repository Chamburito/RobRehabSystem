#ifndef CONNECT_H
#define CONNECT_H

#ifdef __cplusplus
extern "C"{
#endif

#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#pragma comment(lib, "Ws2_32.lib")

/* Preprocessor Directives */

#define QUEUE_SIZE 20
#define BUFFER_SIZE 256

struct timeval timeout = { 1, 0 };
fd_set read_fds;

typedef struct _Connection Connection;

struct _Connection
{
  
  SOCKET sockfd;
  char buffer[ BUFFER_SIZE ];
  char* (*receive_message)( Connection* );
  void (*send_message)( Connection*, const char* );
  int (*add_client)( Connection*, const char* );
  short opened;
  //union {
    struct sockaddr* addr_info;
    int* client_list;
  //};
  //union {
    socklen_t addr_len;
    size_t n_clients;
  //};
  
};

Connection* connection_list = NULL;
size_t n_connections = 0;

char* server_receive_stream_buffer( Connection* connection )
{
  fprintf( stderr, "server_receive_stream_buffer: this connection do not receive messages !\n" );
  
  return NULL;
}

char* client_receive_stream_buffer( Connection* connection )
{
  int bytes_received, bytes_left;
  char* message;
  
  memset( connection->buffer, 0, BUFFER_SIZE );
  // Blocks until there is something to be read in the socket
  bytes_received = recv( connection->sockfd, connection->buffer, BUFFER_SIZE, 0 );
  bytes_left = (int) strtol( connection->buffer, &message, 10 );
  message++;
  printf( "bytes to be read: %d\n", bytes_left );
  printf( "buffer: %s - message: %s\n", connection->buffer, message );
  
  while( bytes_received < bytes_left )
  {
    if( bytes_received == -1 )
    {
      perror( "client_receive_stream_buffer: recv: error reading from socket" );
      return NULL;
    }
    
    bytes_received = recv( connection->sockfd, message, bytes_left, 0 );
    
    bytes_left -= bytes_received;
    message += bytes_received;
  }
  
  return connection->buffer;
}

void send_stream_buffer( Connection* connection, const char* message )
{
  if( strlen( message ) > BUFFER_SIZE )
  {
    fprintf( stderr, "send_stream_buffer: message too long !" );
    return;
  }
  
  char buffer[ BUFFER_SIZE ];
  sprintf( buffer, "%d %s", strlen( message ), message );
  printf( "%s\n", buffer );
  int bytes_left = strlen( buffer );
  int bytes_sent, total_bytes_sent = 0;
  
  while( (bytes_sent = send( connection->sockfd, buffer + total_bytes_sent, bytes_left, 0 )) < bytes_left )
  {
    if( bytes_sent == -1 )
    {
      perror( "send_stream_buffer: send: error writing to socket" );
      return;
    }
    
    bytes_left -= bytes_sent;
    total_bytes_sent += bytes_sent;
  }
  
  return;
}

char* client_receive_datagram_buffer( Connection* connection )
{
  static struct sockaddr_storage addr_info;
  static socklen_t addr_len = sizeof(addr_info);
  
  int bytes_received, bytes_left;
  char* message;
  
  memset( connection->buffer, 0, BUFFER_SIZE );
  // Blocks until there is something to be read in the socket
  bytes_received = recvfrom( connection->sockfd, connection->buffer, BUFFER_SIZE, 0, (struct sockaddr *)&addr_info, &addr_len );
  bytes_left = strtol( connection->buffer, &message, 10 );
  message++;
  
  while( bytes_received < bytes_left )
  {
    if( bytes_received == -1 )
    {
      perror( "client_receive_datagram_buffer: recvfrom: error reading from socket" );
      return NULL;
    }
    
    bytes_received = recvfrom( connection->sockfd, message, bytes_left, 0, (struct sockaddr *)&addr_info, &addr_len );
    
    bytes_left -= bytes_received;
    message += bytes_received;
  }
  
  return connection->buffer;
}

char* client_echo_datagram_buffer( Connection* connection )
{
  return connection->buffer;
}

char* server_receive_datagram_buffer( Connection* connection )
{
  int bytes_received, bytes_left;
  char* message;
  
  memset( connection->buffer, 0, BUFFER_SIZE );
  // Blocks until there is something to be read in the socket
  bytes_received = recvfrom( connection->sockfd, connection->buffer, BUFFER_SIZE, 0, connection->addr_info, &connection->addr_len );
  bytes_left = strtol( connection->buffer, &message, 10 );
  
  while( bytes_received < bytes_left )
  {
    if( bytes_received == -1 )
    {
      perror( "server_receive_datagram_buffer: recvfrom: error reading from socket" );
      return NULL;
    }
    
    bytes_received = recvfrom( connection->sockfd, message, bytes_left, 0, connection->addr_info, &connection->addr_len );
    
    bytes_left -= bytes_received;
    message += bytes_received;
  }
  
  int i;
  for( i = 0; i < connection->n_clients; i++ )
  {
    int client_id = connection->client_list[ i ];
    if( strcmp( connection_list[ client_id ].addr_info->sa_data, connection->addr_info->sa_data ) == 0 )
    {
      strcpy( connection_list[ client_id ].buffer, connection->buffer );
      break;
    }
  }
  
  return NULL;
}

void send_datagram_buffer( Connection* connection, const char* message )
{
  if( strlen( message ) > BUFFER_SIZE )
  {
    fprintf( stderr, "send_datagram_buffer: message too long !" );
    return;
  }
  
  char buffer[ BUFFER_SIZE ];
  sprintf( buffer, "%d %s", strlen( message ), message );
  int bytes_left = strlen( buffer );
  int bytes_sent, total_bytes_sent = 0;
  
  while( (bytes_sent = sendto( connection->sockfd, message + total_bytes_sent, bytes_left , 0, connection->addr_info, connection->addr_len )) < bytes_left )
  {
    if( bytes_sent == -1 )
    {
      perror( "send_datagram_buffer: sendto: error writing to socket" );
      return;
    }
    
    bytes_left -= bytes_sent;
    total_bytes_sent += bytes_sent;
  }
  
  return;
}

void server_sendall_buffer( Connection* connection, const char* message )
{
  static int client_id;
  static Connection* client;
  for( client_id = 0; client_id < connection->n_clients; client_id++ )
  {
    client = connection_list + connection->client_list[ client_id ];
    client->send_message( client, message );
  }
}

int add_connection( int, struct addrinfo* );

int server_add_stream_client( Connection* connection, const char* key )
{
  static struct addrinfo client_info;
  memset( &client_info, 0, sizeof(client_info) );
  client_info.ai_socktype = SOCK_STREAM;
  
  client_info.ai_addr = (sockaddr*) malloc( sizeof(struct sockaddr_storage) ); 
  
  static int client_sockfd; 
  
  FD_ZERO( &read_fds );
  FD_SET( connection->sockfd, &read_fds );
  
  select( connection->sockfd + 1, &read_fds, NULL, NULL, &timeout );
  
  if( FD_ISSET( connection->sockfd, &read_fds ) )
    client_sockfd = accept( connection->sockfd, (struct sockaddr *)(client_info.ai_addr), (int*) &(client_info.ai_addrlen) );

  if( client_sockfd <= 0 )
  {
    perror( "server_add_stream_client: accept: error accepting connection" );
    return -1;
  }
  else
  {    
    memset( connection->buffer, 0, BUFFER_SIZE );
    // Blocks until there is something to be read in the socket
    if( recv( client_sockfd, connection->buffer, BUFFER_SIZE, 0 ) == -1 )
    {
      perror( "recv: error reading from socket" );
      return -1;
    }
    else
    {
      printf( "buffer: %s\n", connection->buffer );
      strtok( connection->buffer, " " );
      char* message = strtok( NULL, " " );
      printf( "message: %s\n", message );
      
      if( strcmp( message, key ) == 0 )
      {
	int client_id =  add_connection( client_sockfd, &client_info );
	connection->client_list = (int*) realloc( connection->client_list, ( connection->n_clients + 1 ) * sizeof(int) );
	connection->client_list[ connection->n_clients ] = client_id;
	connection->n_clients++;
	
	return client_id;
      }
    }
  }
    
  return -1;
}

int server_add_datagram_client( Connection* connection, const char* key )
{
  static struct addrinfo client_info;
	
  if( strcmp( connection->buffer, key ) == 0 )
  {
    client_info.ai_socktype = SOCK_DGRAM;
    client_info.ai_addrlen = connection->addr_len;
    client_info.ai_addr = (struct sockaddr*) malloc( client_info.ai_addrlen );
    *(client_info.ai_addr) = *(connection->addr_info);
    
    int client_id =  add_connection( connection->sockfd, &client_info );
    connection->client_list = (int*) realloc( connection->client_list, ( connection->n_clients + 1 ) * sizeof(int) );
    connection->client_list[ connection->n_clients ] = client_id;
    connection_list[ client_id ].receive_message = client_echo_datagram_buffer;
    connection->n_clients++;
    
    return client_id;
  }
    
  return -1;
}

int client_try_add_client( Connection* connection, const char* key )
{
  fprintf( stderr, "Not a server role connection !\n" );
  
  return -1;
}

int add_connection( int sockfd, struct addrinfo* info )
{
  connection_list = (Connection*) realloc( connection_list, ( n_connections + 1 ) * sizeof(Connection) );
  connection_list[ n_connections ].sockfd = sockfd;
  connection_list[ n_connections ].addr_info = info->ai_addr;
  connection_list[ n_connections ].addr_len = info->ai_addrlen;
  connection_list[ n_connections ].client_list = NULL;
  connection_list[ n_connections ].n_clients = 0;
  
  if( info->ai_flags & AI_PASSIVE ) // Server role connection
  {
    //connection_list[ n_connections ].client_list = NULL;
    //connection_list[ n_connections ].n_clients = 0;
    connection_list[ n_connections ].add_client = ( info->ai_socktype == SOCK_STREAM ) ? server_add_stream_client : server_add_datagram_client;
    connection_list[ n_connections ].receive_message = ( info->ai_socktype == SOCK_STREAM ) ? server_receive_stream_buffer : server_receive_datagram_buffer;
    connection_list[ n_connections ].send_message = server_sendall_buffer;
  }
  else
  {
    //connection_list[ n_connections ].addr_info = (struct sockaddr_storage*)(info->ai_addr);
    //connection_list[ n_connections ].addr_len = info->ai_addrlen;
    connection_list[ n_connections ].add_client = client_try_add_client;
    connection_list[ n_connections ].receive_message = ( info->ai_socktype == SOCK_STREAM ) ? client_receive_stream_buffer : client_receive_datagram_buffer;
    connection_list[ n_connections ].send_message = ( info->ai_socktype == SOCK_STREAM ) ? send_stream_buffer : send_datagram_buffer;
  }
  
  connection_list[ n_connections ].opened = 1;
  
  return n_connections++;
}

char* receive_message( int connection_id )
{
  if( connection_id < 0 || connection_id >= n_connections )
  {
    fprintf( stderr, "connection id %d: receive_message: invalid connection index !\n", connection_id );
    return NULL;
  }
  
  Connection* c = connection_list + connection_id;
  
  return c->receive_message( c );
}

void send_message( int connection_id, const char* message )
{
  if( connection_id < 0 || connection_id >= n_connections )
  {
    //fprintf( stderr, "connection id %d: send_message: invalid connection index !\n", connection_id );
    return;
  }
  
  Connection* c = connection_list + connection_id;
  
  c->send_message( c, message );
  
  return;
}

int try_add_client( int connection_id, const char* key )
{
  if( connection_id < 0 || connection_id >= n_connections )
  {
    fprintf( stderr, "connection id %d: add_client: invalid connection index !\n", connection_id );
    return -1;
  }
  
  Connection* c = connection_list + connection_id;
  
  return c->add_client( c, key );
}

int open_connection( const char* address, const char* port, int type )
{
  static int sockfd;
  static struct addrinfo flags; // Params used to establish listening socket
  static struct addrinfo* host_info; // Resultset for localhost address info, set by getaddrinfo()

  memset( &flags, 0, sizeof(flags) );
  flags.ai_family = AF_UNSPEC; // Use IPv4 or IPv6, whichever
  flags.ai_socktype = type; // SOCK_STREAM (TCP) or SOCK_DGRAM (UDP)
  if( address == NULL ) flags.ai_flags = AI_PASSIVE; // Set address for me
  
  int rw;
  if( (rw = getaddrinfo( address, port, &flags, &host_info )) != 0 )
  {
    perror("open_socket: couldn't read host info for socket start");
    fprintf( stderr, "open_socket: getaddrinfo: %s\n", gai_strerror(rw) );
    return -1;
  }
  
  // loop through all the results and bind to the first we can
  static struct addrinfo* p;
  for(p = host_info; p != NULL; p = p->ai_next) 
  {
    if( (sockfd = socket( p->ai_family, p->ai_socktype, p->ai_protocol )) < 0 )
    {
      perror("open_socket: socket: error opening socket");
      continue;
    }
    
    if( address == NULL )
    {
      static const char yes = 1;
      if( setsockopt( sockfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int) ) == -1 ) 
      {
	perror("open_socket: setsockopt");
	return -1;
      }
  
      if( bind( sockfd, p->ai_addr, p->ai_addrlen ) == -1 )
      {
	perror("open_socket: bind: error on binding");
	continue;
      }
    }
    else if( type == SOCK_STREAM )
    {
      if( connect( sockfd, p->ai_addr, p->ai_addrlen ) == -1 )
      {
	perror("open_socket: connect: error on connecting");
	continue;
      }
    }
    
    break;
  }
  
  if( p == NULL )  
  {
    fprintf( stderr, "open_socket: failed to create socket\n" );
    return -1;
  }
  
  //freeaddrinfo( p->ai_next );
  
  // Pass in socket file descriptor and the size of the backlog queue 
  // (how many pending connections can be in queue while another request
  // is handled)  
  if( address == NULL && type == SOCK_STREAM ) listen( sockfd, QUEUE_SIZE ); 
  
  static int connection_id = add_connection( sockfd, p );
  
  //freeaddrinfo( host_info ); // Don't need this struct anymore
  
  return connection_id;
}

void close_connections()
{
  int connection_id;
  for( connection_id = 0; connection_id < n_connections; connection_id++ )
  {
    closesocket( connection_list[ connection_id ].sockfd );
    //Undefined behavior
    //free( connection_list[ connection_id ].addr_info ); // The same as free( connection_list[ connection_id ].client_list ); ???
    free( connection_list[ connection_id ].client_list );
  }
  
  free( connection_list );
    
  return;
}

#ifdef __cplusplus
}
#endif

#endif /* CONNECT_H */