/////////////////////////////////////////////////////////////////////////////////////
///// Multiplatform library for creation and handling of IP sockets connections /////
///// as server or client, using TCP or UDP protocols                           /////
/////////////////////////////////////////////////////////////////////////////////////

#ifndef IP_CONNECTION_H
#define IP_CONNECTION_H

#include <stdlib.h>
#include <string.h>
#include <fcntl.h>

#include "async_debug.h"
  
#ifdef WIN32
  
  //#include <winsock2.h>
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

  const int SOCKET_ERROR = -1;
  const int INVALID_SOCKET = -1;

  typedef int Socket;
  
#endif

/////////////////////////////////////////////////////////////////////////  
/////                    PREPROCESSOR DIRECTIVES                    /////
/////////////////////////////////////////////////////////////////////////

const int QUEUE_SIZE = 20;
const int IP_CONNECTION_MSG_LEN = 256;

const int IP_HOST_LENGTH = 40;
const int IP_PORT_LENGTH = 6;
const int IP_ADDRESS_LENGTH = IP_HOST_LENGTH + IP_PORT_LENGTH;

const int IP_HOST = 0;
const int IP_PORT = IP_HOST_LENGTH;

enum Property { CLIENT = 0x01, SERVER = 0x02, NETWORK_ROLE_MASK = 0x0f, TCP = 0x10, UDP = 0x20, PROTOCOL_MASK = 0xf0 };


//////////////////////////////////////////////////////////////////////////
/////                         DATA STRUCTURES                        /////
//////////////////////////////////////////////////////////////////////////

typedef struct _IPConnection IPConnection;

// Generic structure to store methods and data of any connection type handled by the library
struct _IPConnection
{
  Socket socketFD;
  union {
    char* (*receiveMessage)( IPConnection* );
    IPConnection* (*acceptClient)( IPConnection* );
  };
  int (*sendMessage)( IPConnection*, const char* );
  struct sockaddr_in6* address;
  uint8_t protocol, networkRole;
  union {
    char* buffer;
    IPConnection** clientsList;
  };
  size_t* ref_clientsCount;
};


//////////////////////////////////////////////////////////////////////////
/////                        GLOBAL VARIABLES                        /////
//////////////////////////////////////////////////////////////////////////

static fd_set socketReadFDs; // stores all the socket file descriptors in use (for performance when verifying incoming messages) 


/////////////////////////////////////////////////////////////////////////////
/////                         NETWORK UTILITIES                         /////
/////////////////////////////////////////////////////////////////////////////

// Utility method to request an address (host and port) string for client connections (returns default values for server connections)
char* IPConnection_GetAddress( IPConnection* connection )
{
  static char addressString[ ADDRESS_LENGTH ];
  static int error = 0;
  
  DEBUG_EVENT( "getting address string for socket fd: %d", connection->socketFD );
  
  error = getnameinfo( (struct sockaddr*) (connection->address), sizeof(struct sockaddr_in6), 
                    &addressString[ HOST ], IP_HOST_LENGTH, &addressString[ PORT ], IP_PORT_LENGTH, NI_NUMERICHOST | NI_NUMERICSERV );

  if( error != 0 )
  {
    ERROR_EVENT( "getnameinfo: failed getting address string: %s", gai_strerror( error ) );
    return NULL;
  }
  
  return addressString;
}

// More complete (but slow) method for verifying the remote address of incoming messages
static int CompareAddresses( struct sockaddr* addr_1, struct sockaddr* addr_2 )
{
  static char addressStrings[ 2 ][ ADDRESS_LENGTH ];
  static int error = 0;
  
  error = getnameinfo( addr_1, sizeof(struct sockaddr_in6), &addressStrings[ 0 ][ HOST ], IP_HOST_LENGTH, &addressStrings[ 0 ][ PORT ], IP_PORT_LENGTH, NI_NUMERICHOST | NI_NUMERICSERV );
  if( error != 0 )
  {
    ERROR_EVENT( "getnameinfo: failed getting address 1 string: %s", gai_strerror( error ) );
    return -1;
  }
  
  error = getnameinfo( addr_2, sizeof(struct sockaddr_in6), &addressStrings[ 1 ][ HOST ], IP_HOST_LENGTH, &addressStrings[ 1 ][ PORT ], IP_PORT_LENGTH, NI_NUMERICHOST | NI_NUMERICSERV );
  if( error != 0 )
  {
    ERROR_EVENT( "getnameinfo: failed getting address 2 string: %s", gai_strerror( error ) );
    return -1;
  }
  
  if( strcmp( (const char*) &addressStrings[ 0 ][ PORT ], (const char*) &addressStrings[ 1 ][ PORT ] ) == 0 )
    return ( strcmp( (const char*) &addressStrings[ 0 ][ HOST ], (const char*) &addressStrings[ 1 ][ HOST ] ) );
  else
    return -1;
}

// Returns number of active clients for a connection 
size_t IPConnection_GetClientsNumber( IPConnection* connection )
{
  size_t connectionsNumber = 0;
  
  if( connection->networkRole == SERVER )
    connectionsNumber = *(connection->ref_clientsCount);
  else
    connectionsNumber = 1;
  
  return connectionsNumber;
}


//////////////////////////////////////////////////////////////////////////////////
/////                             INITIALIZATION                             /////
//////////////////////////////////////////////////////////////////////////////////

// Forward Declaration
static char* ReceiveTCPMessage( IPConnection* );
static char* ReceiveUDPMessage( IPConnection* );
static int SendTCPMessage( IPConnection*, const char* );
static int SendUDPMessage( IPConnection*, const char* );
static int SendMessageAll( IPConnection*, const char* );
static IPConnection* AcceptTCPClient( IPConnection* );
static IPConnection* AcceptUDPClient( IPConnection* );


// Handle construction of a IPConnection structure with the defined properties
static IPConnection* AddConnection( int socketFD, struct sockaddr* address, uint8_t type )
{
  int opt;
  IPConnection* connection = (IPConnection*) malloc( sizeof(IPConnection) );
  connection->socketFD = socketFD;
  connection->protocol = type & PROTOCOL_MASK;
  connection->networkRole = type & NETWORK_ROLE_MASK;

  DEBUG_EVENT( "socket: %d - type: %x\n", connection->socketFD, ( connection->protocol | connection->networkRole ) );
  
  connection->address = (struct sockaddr_in6*) malloc( sizeof(struct sockaddr_in6) );
  *(connection->address) = *((struct sockaddr_in6*) address);
  connection->ref_clientsCount = (size_t*) malloc( sizeof(size_t) );
  *(connection->ref_clientsCount) = 0;
  
  if( connection->networkRole == SERVER ) // Server role connection
  {
    connection->clientsList = NULL;
    connection->acceptClient = ( connection->protocol == TCP ) ? AcceptTCPClient : AcceptUDPClient;
    connection->sendMessage = SendMessageAll;
  }
  else
  { 
    static char addressString[ 16 * sizeof(uint8_t) ];
    for( opt = 0; opt < 16; opt++ )
      sprintf( &(addressString[ opt * sizeof(uint8_t) ]), "%3u", connection->address->sin6_addr.s6_addr[ opt ] );
    DEBUG_EVENT( "connection added:\n\tfamily: %x\n\tport: %u\n\taddress: %s", connection->address->sin6_family,
                                                                               connection->address->sin6_port, addressString );

    //connection->address->sin6_family = AF_INET6;
    connection->buffer = (char*) calloc( IP_CONNECTION_MSG_LEN, sizeof(char) );
    connection->receiveMessage = ( connection->protocol == TCP ) ? ReceiveTCPMessage : ReceiveUDPMessage;
    connection->sendMessage = ( connection->protocol == TCP ) ? SendTCPMessage : SendUDPMessage;
  }
  
  return connection;
}

// Add defined connection to the client list of the given server connection
static INLINE void AddClient( IPConnection* server, IPConnection* client )
{
  static size_t clientsNumber, clientIndex = 0, clientsListSize = 0;
  
  free( client->ref_clientsCount );
  client->ref_clientsCount = server->ref_clientsCount;
  
  clientsNumber = *(server->ref_clientsCount);
  while( clientIndex < clientsNumber )
  {
    clientsListSize++;
    if( server->clientsList[ clientIndex ] != NULL )
      clientIndex++;
    else
      break;
  }
  
  if( clientIndex == clientsListSize )
    server->clientsList = (IPConnection**) realloc( server->clientsList, ( clientIndex + 1 ) * sizeof(IPConnection*) );
  server->clientsList[ clientIndex ] = client;
  
  (*(server->ref_clientsCount))++;

  return;
}

// Generic method for opening a new socket and providing a corresponding IPConnection structure for use
IPConnection* IPConnection_Open( const char* host, const char* port, uint8_t protocol )
{
  static IPConnection* connection;

  static int socketFD;
  static struct addrinfo hints;
  static struct addrinfo* hostsInfoList;
  static struct addrinfo* hostInfo;

  static int rw;
  static uint8_t networkRole = 0;
  
  #ifdef WIN32
  WSADATA wsa;
  if( WSAStartup( MAKEWORD(2,2), &wsa ) != 0 )
  {
    fprintf( stderr, "%s: error initialiasing windows sockets: code: %d", __func__, WSAGetLastError() );
    return NULL;
  }
  #endif
  
  // Assure that the port number is in the Dynamic/Private range (49152-65535)
  unsigned int portNumber = (unsigned int) strtoul( port, NULL, 0 );
  if( portNumber < 49152 || portNumber > 65535 )
  {
    ERROR_EVENT( "invalid port number value: %s", port );
    return NULL;
  }

  memset( &hints, 0, sizeof(hints) );
  hints.ai_family = AF_UNSPEC; // AF_INET6 (IPv6), AF_INET (IPv4) or AF_UNSPEC

  if( protocol == TCP )
    hints.ai_socktype = SOCK_STREAM;
  else if( protocol == UDP )
    hints.ai_socktype = SOCK_DGRAM;
  else
  {
    DEBUG_EVENT( "invalid protocol option: %x", protocol );
    return NULL;
  }

  if( host == NULL ) 
  {
    hints.ai_flags |= AI_PASSIVE; // Set address for me
    networkRole = SERVER;
  }
  else
    networkRole = CLIENT;

  if( (rw = getaddrinfo( host, port, &hints, &hostsInfoList )) != 0 )
  {
    ERROR_EVENT( "getaddrinfo: error reading host info: %s\n", gai_strerror( rw ) );
    return NULL;
  }

  // loop through all the results and bind to the first we can
  for( hostInfo = hostsInfoList; hostInfo != NULL; hostInfo = hostInfo->ai_next ) 
  {
    // Extended connection info for debug builds
    static char addressString[ ADDRESS_LENGTH ];
    getnameinfo( hostInfo->ai_addr, sizeof(struct sockaddr), 
                 &addressString[ HOST ], IP_HOST_LENGTH, &addressString[ PORT ], IP_PORT_LENGTH, NI_NUMERICHOST | NI_NUMERICSERV );
    
    DEBUG_EVENT( "trying to %s to: host: %s - port: %s - protocol: %s - version: %s", ( host == NULL ) ? "bind" : "connect",
                                                                                      &addressString[ HOST ], &addressString[ PORT ],
                                                                                      ( hostInfo->ai_protocol == IPPROTO_TCP ) ? "TCP" : "UDP",
                                                                                      ( hostInfo->ai_family == AF_INET6 ) ? "IPv6" : "IPv4" );

    // Create IP socket
    if( (socketFD = socket( hostInfo->ai_family, hostInfo->ai_socktype, hostInfo->ai_protocol )) == INVALID_SOCKET )
    {
      ERROR_EVENT( "socket: failed opening %s %s socket", ( hostInfo->ai_protocol == IPPROTO_TCP ) ? "TCP" : "UDP",
                                                          ( hostInfo->ai_family == AF_INET6 ) ? "IPv6" : "IPv4" );                                                              
      continue;
    }
    
    #ifdef WIN32
    rw = 1;
    if( ioctlsocket( socketFD, FIONBIO, (u_long*) &rw ) == SOCKET_ERROR )
    #else
    if( fcntl( socketFD, F_SETFL, O_NONBLOCK ) == SOCKET_ERROR )
    #endif
    {
      ERROR_EVENT( "failure setting socket %d to non-blocking state", socketFD );
      close( socketFD );
      return NULL;
    }

    rw = 1;
    // Allow sockets to be binded to the same local port
    if( setsockopt( socketFD, SOL_SOCKET, SO_REUSEADDR, (const char*) &rw, sizeof(rw) ) == SOCKET_ERROR ) 
    {
      
      ERROR_EVENT( "setsockopt: failed setting socket %d option SO_REUSEADDR", socketFD );
      close( socketFD );
      return NULL;
    }
    
    if( networkRole == SERVER )
    {
      if( hostInfo->ai_family == AF_INET ) continue;
      
      rw = 0;
      // Let IPV6 servers accept IPV4 clients
      if( setsockopt( socketFD, IPPROTO_IPV6, IPV6_V6ONLY, (const char*) &rw, sizeof(rw) ) == SOCKET_ERROR )
      {
        ERROR_EVENT( "setsockopt: failed setting socket %d option IPV6_V6ONLY", socketFD );
        close( socketFD );
        return NULL;
      }
      
      // Bind server socket to the given local address
      if( bind( socketFD, hostInfo->ai_addr, hostInfo->ai_addrlen ) == SOCKET_ERROR )
      {
        ERROR_EVENT( "bind: failed on binding socket %d to local port %u", socketFD, ((struct sockaddr_in6*) hostInfo->ai_addr)->sin6_port );
        close( socketFD );
        continue;
      }
      
      if( hostInfo->ai_socktype == SOCK_STREAM )
      {
        // Set server socket to listen to remote connections
        if( listen( socketFD, QUEUE_SIZE ) == SOCKET_ERROR )
        {
          ERROR_EVENT( "listen: failed listening on socket %d", socketFD );
          close( socketFD );
          continue;
        }
      }
    }
    else if( protocol == TCP )
    {
      // Connect TCP client socket to given remote address
      if( connect( socketFD, hostInfo->ai_addr, hostInfo->ai_addrlen ) == SOCKET_ERROR )
      {
        ERROR_EVENT( "connect: failed on connecting socket %d to remote address %s/%s", socketFD, &addressString[ HOST ], &addressString[ PORT ] );
        close( socketFD );
        continue;
      }
    }
    else
    {
      // Bind UDP client socket to available local address
      static struct sockaddr_storage local_address;
      local_address.ss_family = hostInfo->ai_addr->sa_family;
      if( bind( socketFD, (struct sockaddr*) &local_address, sizeof(local_address) ) == SOCKET_ERROR )
      {
        ERROR_EVENT( "bind: failed on binding socket %d to arbitrary local port", socketFD );
        close( socketFD );
        continue;
      }
    }
    
    break;
  }
  
  if( hostInfo == NULL )  
  {
    ERROR_EVENT( "failed to create %s %s %s socket on host %s and port %s", ( host == NULL ) ? "server" : "client",
                                                                            ( hostInfo->ai_protocol == IPPROTO_TCP ) ? "TCP" : "UDP",
                                                                            ( hostInfo->ai_family == AF_INET6 ) ? "IPv6" : "IPv4",
                                                                            ( host == NULL ) ? "localhost" : host, port );
    return NULL;
  }
  
  connection = AddConnection( socketFD, hostInfo->ai_addr, ( protocol | networkRole ) ); // Build the IPConnection structure
  
  freeaddrinfo( hostsInfoList ); // Don't need this struct anymore
  
  return connection;
}


/////////////////////////////////////////////////////////////////////////////////
/////                             COMMUNICATION                             /////
/////////////////////////////////////////////////////////////////////////////////

// Wrapper functions
extern INLINE char* IPConnection_ReceiveMessage( IPConnection* c ) { return c->receiveMessage( c ); }
extern INLINE int IPConnection_SendMessage( IPConnection* c, const char* message ) { return c->sendMessage( c, message ); }
extern INLINE IPConnection* IPConnection_AcceptClient( IPConnection* c ) { return c->acceptClient( c ); }

// Verify available incoming messages for the given connection, preventing unnecessary blocking calls (for syncronous networking)
short IPConnection_WaitEvent( IPConnection* connection, unsigned int milisseconds )
{
  fd_set readFDs;
  struct timeval timeout;
  int eventsNumber;

  FD_ZERO( &readFDs );
  FD_SET( connection->socketFD, &readFDs );

  timeout.tv_sec = milisseconds / 1000;
  timeout.tv_usec =  1000 * ( milisseconds % 1000 );
  
  eventsNumber = select( connection->socketFD + 1, &readFDs, NULL, NULL, &timeout );
  if( eventsNumber == SOCKET_ERROR )
  {
    ERROR_EVENT( "select: error waiting for message on socket %d", connection->socketFD );
    return -1;
  }
  else if( eventsNumber == 0 )
    return 0;
  
  if( FD_ISSET( connection->socketFD, &readFDs ) )
    return 1;
  else
    return 0;
}

// Try to receive incoming message from the given TCP client connection and store it on its buffer
static char* ReceiveTCPMessage( IPConnection* connection )
{
  int bytes_received;
  
  memset( connection->buffer, 0, IP_CONNECTION_MSG_LEN );
  // Blocks until there is something to be read in the socket
  bytes_received = recv( connection->socketFD, connection->buffer, IP_CONNECTION_MSG_LEN, 0 );

  if( bytes_received == SOCKET_ERROR )
  {
    ERROR_EVENT( "recv: error reading from socket %d", connection->socketFD );
    return NULL;
  }
  else if( bytes_received == 0 )
  {
    ERROR_EVENT( "recv: remote connection with socket %d closed", connection->socketFD );
    return NULL;
  }
  
  DEBUG_UPDATE( "socket %d received message: %s", connection->socketFD, connection->buffer );
  
  return connection->buffer;
}

// Send given message through the given TCP connection
static int SendTCPMessage( IPConnection* connection, const char* message )
{
  if( strlen( message ) + 1 > IP_CONNECTION_MSG_LEN )
  {
    ERROR_EVENT( "message too long (%u bytes) !", strlen( message ) );
    return 0;
  }
  
  DEBUG_UPDATE( "connection socket %d sending message: %s\n", connection->socketFD, message );
  
  if( send( connection->socketFD, message, IP_CONNECTION_MSG_LEN, 0 ) == SOCKET_ERROR )
  {
    ERROR_EVENT( "send: error writing to socket %d", connection->socketFD );
    return -1;
  }
  
  return 0;
}

// Try to receive incoming message from the given UDP client connection and store it on its buffer
static char* ReceiveUDPMessage( IPConnection* connection )
{
  struct sockaddr_in6 address;
  socklen_t addr_len = sizeof(struct sockaddr_storage);
  static uint8_t* addressString[ 2 ];
  static char* empty_message = { '\0' };

  memset( connection->buffer, 0, IP_CONNECTION_MSG_LEN );
  // Blocks until there is something to be read in the socket
  if( recvfrom( connection->socketFD, connection->buffer, IP_CONNECTION_MSG_LEN, MSG_PEEK, (struct sockaddr *) &address, &addr_len ) == SOCKET_ERROR )
  {
    ERROR_EVENT( "recvfrom: error reading from socket %d", connection->socketFD );
    return NULL;
  }

  // Verify if incoming message is destined to this connection (and returns the message if it is)
  if( connection->address->sin6_port == address.sin6_port )
  {
    addressString[0] = connection->address->sin6_addr.s6_addr;
    addressString[1] = address.sin6_addr.s6_addr;
    if( strncmp( (const char*) addressString[ 0 ], (const char*) addressString[ 1 ], sizeof(struct in6_addr) ) == 0 )
    {
      recv( connection->socketFD, NULL, 0, 0 );
    
      DEBUG_UPDATE( "socket %d received right message: %s", connection->socketFD, connection->buffer );
    
      return connection->buffer;
    }
  }
  
  // Default return message (the received one was not destined to this connection) 
  return empty_message;
}

// Send given message through the given UDP connection
static int SendUDPMessage( IPConnection* connection, const char* message )
{
  if( strlen( message ) + 1 > IP_CONNECTION_MSG_LEN )
  {
    ERROR_EVENT( "message too long (%u bytes) !", strlen( message ) );
    return 0;
  }
  
  DEBUG_UPDATE( "connection socket %d sending message: %s\n", connection->socketFD, message );
  
  if( sendto( connection->socketFD, message, IP_CONNECTION_MSG_LEN, 0, (struct sockaddr *) connection->address, sizeof(struct sockaddr_in6) ) == SOCKET_ERROR )
  {
    ERROR_EVENT( "sendto: error writing to socket %d", connection->socketFD );
    return -1;
  }
  
  return 0;
}

// Send given message to all the clients of the given server connection
static int SendMessageAll( IPConnection* connection, const char* message )
{
  static size_t clientIndex = 0, clientsNumber;
    clientsNumber = *(connection->ref_clientsCount);
  while( clientIndex < clientsNumber )
  {
    if( connection->clientsList[ clientIndex ] != NULL )
    {
      IPConnection_SendMessage( connection->clientsList[ clientIndex ], message );
      clientIndex++;
    }
  }
  
  return 0;
}

// Waits for a remote connection to be added to the client list of the given TCP server connection
static IPConnection* AcceptTCPClient( IPConnection* server )
{
  IPConnection* client;
  int clientSocketFD;
  static struct sockaddr_storage clientAddress;
  static socklen_t addr_len = sizeof(clientAddress);
  
  clientSocketFD = accept( server->socketFD, (struct sockaddr *) &clientAddress, &addr_len );

  if( clientSocketFD == INVALID_SOCKET )
  {
    ERROR_EVENT( "accept: failed accepting connection on socket %d", server->socketFD );
    return NULL;
  }
  
  DEBUG_EVENT( "client accepted: socket fd: %d\n", clientSocketFD );
  
  client =  AddConnection( clientSocketFD, (struct sockaddr *) &clientAddress, CLIENT | TCP );

  AddClient( server, client );

  return client;
}

// Waits for a remote connection to be added to the client list of the given UDP server connection
static IPConnection* AcceptUDPClient( IPConnection* server )
{
  size_t clientIndex, clientsNumber;
  IPConnection* client;
  static struct sockaddr_storage clientAddress;
  static socklen_t addr_len = sizeof(clientAddress);
  static char buffer[ IP_CONNECTION_MSG_LEN ];
  static uint8_t* addressString[2];
  static IPConnection dummy_connection;
  int bytes_received;

  if( recvfrom( server->socketFD, buffer, IP_CONNECTION_MSG_LEN, MSG_PEEK, (struct sockaddr *) &clientAddress, &addr_len ) == SOCKET_ERROR )
  {
    ERROR_EVENT( "recvfrom: error reading from socket %d", server->socketFD );
    return NULL;
  }
  
  // Verify if incoming message belongs to unregistered client (returns default value if not)
  clientsNumber = *(server->ref_clientsCount);
  for( clientIndex = 0; clientIndex < clientsNumber; clientIndex++ )
  {
    DEBUG_UPDATE( "comparing ports: %u - %u", server->clientsList[ clientIndex ]->address->sin6_port, ((struct sockaddr_in6*) &clientAddress)->sin6_port );
    
    if( server->clientsList[ clientIndex ]->address->sin6_port == ((struct sockaddr_in6*) &clientAddress)->sin6_port )
    {
      addressString[0] = server->clientsList[ clientIndex ]->address->sin6_addr.s6_addr;
      addressString[1] = ((struct sockaddr_in6*) &clientAddress)->sin6_addr.s6_addr;
      if( strncmp( (const char*) addressString[0], (const char*) addressString[1], sizeof(struct in6_addr) ) == 0 )
        return &dummy_connection;
    }
  }
  
  DEBUG_EVENT( "client accepted (clients count before: %u)", *(server->ref_clientsCount) );
  
  client = AddConnection( server->socketFD, (struct sockaddr*) &clientAddress, CLIENT | UDP );

  AddClient( server, client );
  
  DEBUG_EVENT( "client accepted (clients count after: %u)", *(server->ref_clientsCount) );
  
  return client;
}


//////////////////////////////////////////////////////////////////////////////////
/////                               FINALIZING                               /////
//////////////////////////////////////////////////////////////////////////////////

// Handle proper destruction of any given connection type
void IPConnection_Close( IPConnection* connection )
{
  if( connection != NULL )
  {
    DEBUG_EVENT( "closing connection for socket %d (users count before: %u)", connection->socketFD, *(connection->ref_clientsCount) );

    // Each TCP connection has its own socket, so we can close it without problem. But UDP connections
    // from the same server share the socket, so we need to wait for all of them to be stopped to close the socket
    if( connection->protocol == TCP )
    {
      DEBUG_UPDATE( "closing TCP connection unused socket fd: %d", connection->socketFD );
      FD_CLR( connection->socketFD, &socketReadFDs );
      shutdown( connection->socketFD, SHUT_RDWR );
    }
    // Check number of client connections of a server (also of sharers of a socket for UDP connections)
    if( *(connection->ref_clientsCount) <= 0 )
    {
      if( connection->protocol == UDP )
      {
        DEBUG_UPDATE( "closing UDP connection unused socket fd: %d", connection->socketFD );
        
        FD_CLR( connection->socketFD, &socketReadFDs );
        close( connection->socketFD );
      }
      free( connection->ref_clientsCount );
      connection->ref_clientsCount = NULL;
    }
    else
      (*(connection->ref_clientsCount))--;
  
    free( connection->address );
    connection->address = NULL;
  
    if( connection->networkRole == CLIENT )
    {
      DEBUG_UPDATE( "freeing client connection (socket %d) message buffer", connection->socketFD );

      free( connection->buffer );
      connection->buffer = NULL;
    }
    else if( connection->networkRole == SERVER )
    {
      DEBUG_UPDATE( "cleaning server connection (socket %d) client list", connection->socketFD );
      
      if( connection->clientsList != NULL )
      {
        free( connection->clientsList );
        connection->clientsList = NULL;
      }
    }
  
    DEBUG_EVENT( "closing connection for socket %d (users count after: %u)", connection->socketFD, 
                                                   ( connection->ref_clientsCount == NULL ) ? 0 : *(connection->ref_clientsCount) );
  
    free( connection );
    connection = NULL;
  }

  return;
}

#endif /* IP_CONNECTION_H */