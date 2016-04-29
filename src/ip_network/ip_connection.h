/////////////////////////////////////////////////////////////////////////////////////
///// Multiplatform library for creation and handling of IP sockets connections /////
///// as server or client, using TCP or UDP protocols                           /////
/////////////////////////////////////////////////////////////////////////////////////

#ifndef IP_CONNECTION_H
#define IP_CONNECTION_H

#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
  
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
  
  #include <fcntl.h>
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

#include "interfaces.h"
  
#include "debug/sync_debug.h"

/////////////////////////////////////////////////////////////////////////  
/////                    PREPROCESSOR DIRECTIVES                    /////
/////////////////////////////////////////////////////////////////////////

// const size_t QUEUE_SIZE = 20;
// const size_t IP_CONNECTION_MSG_LEN = 256;
// 
// const size_t IP_HOST_LENGTH = 40;
// const size_t IP_PORT_LENGTH = 6;
// const size_t IP_ADDRESS_LENGTH = IP_HOST_LENGTH + IP_PORT_LENGTH;
// 
// const size_t IP_HOST = 0;
// const size_t IP_PORT = IP_HOST_LENGTH;
  
#define QUEUE_SIZE 20
const size_t IP_MAX_MESSAGE_LENGTH = 256;

#define IP_HOST_LENGTH 40
#define IP_PORT_LENGTH 6
#define IP_ADDRESS_LENGTH IP_HOST_LENGTH + IP_PORT_LENGTH

#define IP_HOST 0
#define IP_PORT IP_HOST_LENGTH

enum Property { DISCONNECTED = 0x00, CLIENT = 0x01, SERVER = 0x02, LISTENER = 0x03, NETWORK_ROLE_MASK = 0x0f, TCP = 0x10, UDP = 0x20, PROTOCOL_MASK = 0xf0 };


//////////////////////////////////////////////////////////////////////////
/////                         DATA STRUCTURES                        /////
//////////////////////////////////////////////////////////////////////////

typedef struct _IPConnectionData IPConnectionData;
typedef IPConnectionData* IPConnection;

// Generic structure to store methods and data of any connection type handled by the library
struct _IPConnectionData
{
  Socket socketFD;
  union {
    char* (*ref_ReceiveMessage)( IPConnection );
    IPConnection (*ref_AcceptClient)( IPConnection );
  };
  int (*ref_SendMessage)( IPConnection, const char* );
  struct sockaddr_in6* address;
  uint8_t protocol, networkRole;
  union {
    char* buffer;
    IPConnection* clientsList;
  };
  size_t* ref_clientsCount;
};


//////////////////////////////////////////////////////////////////////////
/////                        GLOBAL VARIABLES                        /////
//////////////////////////////////////////////////////////////////////////

static fd_set socketReadFDs; // stores all the socket file descriptors in use (for performance when verifying incoming messages) 


///////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                            INTERFACE                                            /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

#define IP_NETWORK_FUNCTIONS( namespace, FUNCTION_INIT ) \
        FUNCTION_INIT( char*, namespace, GetAddress, IPConnection ) \
        FUNCTION_INIT( size_t, namespace, GetClientsNumber, IPConnection ) \
        FUNCTION_INIT( size_t, namespace, SetMessageLength, IPConnection, size_t ) \
        FUNCTION_INIT( IPConnection, namespace, OpenConnection, const char*, const char*, uint8_t ) \
        FUNCTION_INIT( void, namespace, CloseConnection, IPConnection ) \
        FUNCTION_INIT( bool, WaitEvent, IPConnection, unsigned int ) \
        FUNCTION_INIT( char*, namespace, ReadMessage, IPConnection ) \
        FUNCTION_INIT( int, namespace, WriteMessage, IPConnection, const char* ) \
        FUNCTION_INIT( IPConnection, namespace, AcceptClient, IPConnection )

INIT_NAMESPACE_INTERFACE( IPNetwork, IP_NETWORK_FUNCTIONS )


/////////////////////////////////////////////////////////////////////////////
/////                         NETWORK UTILITIES                         /////
/////////////////////////////////////////////////////////////////////////////

// Utility method to request an address (host and port) string for client connections (returns default values for server connections)
char* IPNetwork_GetAddress( IPConnection connection )
{
  static char addressString[ IP_ADDRESS_LENGTH ];
  static int error = 0;
  
  DEBUG_PRINT( "getting address string for socket fd: %d", connection->socketFD );
  
  error = getnameinfo( (struct sockaddr*) (connection->address), sizeof(struct sockaddr_in6), 
                    &addressString[ IP_HOST ], IP_HOST_LENGTH, &addressString[ IP_PORT ], IP_PORT_LENGTH, NI_NUMERICHOST | NI_NUMERICSERV );

  if( error != 0 )
  {
    ERROR_PRINT( "getnameinfo: failed getting address string: %s", gai_strerror( error ) );
    return NULL;
  }
  
  return addressString;
}

// More complete (but slow) method for verifying the remote address of incoming messages
static int CompareAddresses( struct sockaddr* addr_1, struct sockaddr* addr_2 )
{
  char addressStrings[ 2 ][ IP_ADDRESS_LENGTH ];
  static int error = 0;
  
  error = getnameinfo( addr_1, sizeof(struct sockaddr_in6), &addressStrings[ 0 ][ IP_HOST ], IP_HOST_LENGTH, &addressStrings[ 0 ][ IP_PORT ], IP_PORT_LENGTH, NI_NUMERICHOST | NI_NUMERICSERV );
  if( error != 0 )
  {
    ERROR_PRINT( "getnameinfo: failed getting address 1 string: %s", gai_strerror( error ) );
    return -1;
  }
  
  error = getnameinfo( addr_2, sizeof(struct sockaddr_in6), &addressStrings[ 1 ][ IP_HOST ], IP_HOST_LENGTH, &addressStrings[ 1 ][ IP_PORT ], IP_PORT_LENGTH, NI_NUMERICHOST | NI_NUMERICSERV );
  if( error != 0 )
  {
    ERROR_PRINT( "getnameinfo: failed getting address 2 string: %s", gai_strerror( error ) );
    return -1;
  }
  
  if( strcmp( (const char*) &addressStrings[ 0 ][ IP_PORT ], (const char*) &addressStrings[ 1 ][ IP_PORT ] ) == 0 )
    return ( strcmp( (const char*) &addressStrings[ 0 ][ IP_HOST ], (const char*) &addressStrings[ 1 ][ IP_HOST ] ) );
  else
    return -1;
}

// Returns number of active clients for a connection 
size_t IPNetwork_GetClientsNumber( IPConnection connection )
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
static char* ReceiveTCPMessage( IPConnection );
static char* ReceiveUDPMessage( IPConnection );
static int SendTCPMessage( IPConnection, const char* );
static int SendUDPMessage( IPConnection, const char* );
static int SendMessageAll( IPConnection, const char* );
static IPConnection AcceptTCPClient( IPConnection );
static IPConnection AcceptUDPClient( IPConnection );


// Handle construction of a IPConnection structure with the defined properties
static IPConnection AddConnection( int socketFD, struct sockaddr* address, uint8_t type )
{
  int opt;
  IPConnection connection = (IPConnection) malloc( sizeof(IPConnection) );
  connection->socketFD = socketFD;
  connection->protocol = type & PROTOCOL_MASK;
  connection->networkRole = type & NETWORK_ROLE_MASK;

  DEBUG_PRINT( "socket: %d - type: %x", connection->socketFD, ( connection->protocol | connection->networkRole ) );
  
  connection->address = (struct sockaddr_in6*) malloc( sizeof(struct sockaddr_in6) );
  *(connection->address) = *((struct sockaddr_in6*) address);
  connection->ref_clientsCount = (size_t*) malloc( sizeof(size_t) );
  *(connection->ref_clientsCount) = 0;
  
  if( connection->networkRole == SERVER ) // Server role connection
  {
    connection->clientsList = NULL;
    connection->ref_AcceptClient = ( connection->protocol == TCP ) ? AcceptTCPClient : AcceptUDPClient;
    connection->ref_SendMessage = SendMessageAll;
  }
  else
  { 
    static char addressString[ 16 * sizeof(uint8_t) ];
    for( opt = 0; opt < 16; opt++ )
      sprintf( &(addressString[ opt * sizeof(uint8_t) ]), "%3u", connection->address->sin6_addr.s6_addr[ opt ] );
    DEBUG_PRINT( "connection added:\n\tfamily: %x\n\tport: %u\n\taddress: %s", connection->address->sin6_family, connection->address->sin6_port, addressString );

    //connection->address->sin6_family = AF_INET6;
    connection->buffer = (char*) calloc( IP_MAX_MESSAGE_LENGTH, sizeof(char) );
    connection->ref_ReceiveMessage = ( connection->protocol == TCP ) ? ReceiveTCPMessage : ReceiveUDPMessage;
    connection->ref_SendMessage = ( connection->protocol == TCP ) ? SendTCPMessage : SendUDPMessage;
  }
  
  return connection;
}

// Add defined connection to the client list of the given server connection
static inline void AddClient( IPConnection server, IPConnection client )
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
    server->clientsList = (IPConnection*) realloc( server->clientsList, ( clientIndex + 1 ) * sizeof(IPConnection) );
  server->clientsList[ clientIndex ] = client;
  
  (*(server->ref_clientsCount))++;

  return;
}

// Generic method for opening a new socket and providing a corresponding IPConnection structure for use
IPConnection IPNetwork_Open( const char* host, const char* port, uint8_t protocol )
{
  static IPConnection connection;

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
    ERROR_PRINT( "invalid port number value: %s", port );
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
    DEBUG_PRINT( "invalid protocol option: %x", protocol );
    return NULL;
  }

  if( host == NULL ) 
  {
    hints.ai_flags |= AI_PASSIVE; // Set address for me
    networkRole = SERVER;
  }
  else
    networkRole = CLIENT;

  #ifndef _CVI_DLL_
  if( (rw = getaddrinfo( host, port, &hints, &hostsInfoList )) != 0 )
  {
    ERROR_PRINT( "getaddrinfo: error reading host info: %s", gai_strerror( rw ) );
    return NULL;
  }
  #else
  hostInfo = &hints;
  hostInfo->ai_family = AF_INET6;
  hostInfo->ai_addr->sa_family = AF_INET6;
  ((struct sockaddr_in6*) hostInfo->ai_addr)->sin6_port = htons( (short) portNumber );
  ((struct sockaddr_in6*) hostInfo->ai_addr)->sin6_addr = in6addr_any;
  hostInfo->ai_next = NULL;
  #endif

  // loop through all the results and bind to the first we can
  for( hostInfo = hostsInfoList; hostInfo != NULL; hostInfo = hostInfo->ai_next ) 
  {
    // Extended connection info for debug builds
    static char addressString[ IP_ADDRESS_LENGTH ];
    getnameinfo( hostInfo->ai_addr, sizeof(struct sockaddr), 
                 &addressString[ IP_HOST ], IP_HOST_LENGTH, &addressString[ IP_PORT ], IP_PORT_LENGTH, NI_NUMERICHOST | NI_NUMERICSERV );
    
    DEBUG_PRINT( "trying to %s to: host: %s - port: %s - protocol: %s - version: %s", ( host == NULL ) ? "bind" : "connect",
                                                                                      &addressString[ IP_HOST ], &addressString[ IP_PORT ],
                                                                                      ( hostInfo->ai_protocol == IPPROTO_TCP ) ? "TCP" : "UDP",
                                                                                      ( hostInfo->ai_family == AF_INET6 ) ? "IPv6" : "IPv4" );

    // Create IP socket
    if( (socketFD = socket( hostInfo->ai_family, hostInfo->ai_socktype, hostInfo->ai_protocol )) == INVALID_SOCKET )
    {
      ERROR_PRINT( "socket: failed opening %s %s socket", ( hostInfo->ai_protocol == IPPROTO_TCP ) ? "TCP" : "UDP",
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
      ERROR_PRINT( "failure setting socket %d to non-blocking state", socketFD );
      close( socketFD );
      return NULL;
    }

    rw = 1;
    // Allow sockets to be binded to the same local port
    if( setsockopt( socketFD, SOL_SOCKET, SO_REUSEADDR, (const char*) &rw, sizeof(rw) ) == SOCKET_ERROR ) 
    {
      
      ERROR_PRINT( "setsockopt: failed setting socket %d option SO_REUSEADDR", socketFD );
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
        ERROR_PRINT( "setsockopt: failed setting socket %d option IPV6_V6ONLY", socketFD );
        close( socketFD );
        return NULL;
      }
      
      // Bind server socket to the given local address
      if( bind( socketFD, hostInfo->ai_addr, hostInfo->ai_addrlen ) == SOCKET_ERROR )
      {
        ERROR_PRINT( "bind: failed on binding socket %d to local port %u", socketFD, ((struct sockaddr_in6*) hostInfo->ai_addr)->sin6_port );
        close( socketFD );
        continue;
      }
      
      if( hostInfo->ai_socktype == SOCK_STREAM )
      {
        // Set server socket to listen to remote connections
        if( listen( socketFD, QUEUE_SIZE ) == SOCKET_ERROR )
        {
          ERROR_PRINT( "listen: failed listening on socket %d", socketFD );
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
        ERROR_PRINT( "connect: failed on connecting socket %d to remote address %s/%s", socketFD, &addressString[ IP_HOST ], &addressString[ IP_PORT ] );
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
        ERROR_PRINT( "bind: failed on binding socket %d to arbitrary local port", socketFD );
        close( socketFD );
        continue;
      }
    }
    
    break;
  }
  
  if( hostInfo == NULL )  
  {
    ERROR_PRINT( "failed to create %s %s %s socket on host %s and port %s", ( host == NULL ) ? "server" : "client",
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
inline char* IPNetwork_ReceiveMessage( IPConnection c ) { return c->ref_ReceiveMessage( c ); }
inline int IPNetwork_SendMessage( IPConnection c, const char* message ) { return c->ref_SendMessage( c, message ); }
inline IPConnection IPNetwork_AcceptClient( IPConnection c ) { return c->ref_AcceptClient( c ); }

// Verify available incoming messages for the given connection, preventing unnecessary blocking calls (for syncronous networking)
bool IPNetwork_WaitEvent( IPConnection connection, unsigned int milliseconds )
{
  fd_set readFDs;
  struct timeval timeout;
  int eventsNumber;

  FD_ZERO( &readFDs );
  FD_SET( connection->socketFD, &readFDs );

  timeout.tv_sec = milliseconds / 1000;
  timeout.tv_usec =  1000 * ( milliseconds % 1000 );
  
  eventsNumber = select( connection->socketFD + 1, &readFDs, NULL, NULL, &timeout );
  if( eventsNumber == SOCKET_ERROR )
  {
    ERROR_PRINT( "select: error waiting for message on socket %d", connection->socketFD );
    return -1;
  }
  else if( eventsNumber == 0 )
    return 0;
  
  if( FD_ISSET( connection->socketFD, &readFDs ) )
    return true;
  else
    return false;
}

// Try to receive incoming message from the given TCP client connection and store it on its buffer
static char* ReceiveTCPMessage( IPConnection connection )
{
  int bytes_received;
  
  memset( connection->buffer, 0, IP_MAX_MESSAGE_LENGTH );
  // Blocks until there is something to be read in the socket
  bytes_received = recv( connection->socketFD, connection->buffer, IP_MAX_MESSAGE_LENGTH, 0 );

  if( bytes_received == SOCKET_ERROR )
  {
    ERROR_PRINT( "recv: error reading from socket %d", connection->socketFD );
    return NULL;
  }
  else if( bytes_received == 0 )
  {
    ERROR_PRINT( "recv: remote connection with socket %d closed", connection->socketFD );
    return NULL;
  }
  
  //DEBUG_PRINT( "socket %d received message: %s", connection->socketFD, connection->buffer );
  
  return connection->buffer;
}

// Send given message through the given TCP connection
static int SendTCPMessage( IPConnection connection, const char* message )
{
  if( strlen( message ) + 1 > IP_MAX_MESSAGE_LENGTH )
  {
    ERROR_PRINT( "message too long (%lu bytes) !", strlen( message ) );
    return 0;
  }
  
  DEBUG_PRINT( "connection socket %d sending message: %s", connection->socketFD, message );
  
  if( send( connection->socketFD, message, IP_MAX_MESSAGE_LENGTH, 0 ) == SOCKET_ERROR )
  {
    ERROR_PRINT( "send: error writing to socket %d", connection->socketFD );
    return -1;
  }
  
  return 0;
}

// Try to receive incoming message from the given UDP client connection and store it on its buffer
static char* ReceiveUDPMessage( IPConnection connection )
{
  struct sockaddr_in6 address;
  socklen_t addressLength = sizeof(struct sockaddr_storage);
  static uint8_t* addressString[ 2 ];
  static char* empty_message = { '\0' };

  memset( connection->buffer, 0, IP_MAX_MESSAGE_LENGTH );
  // Blocks until there is something to be read in the socket
  if( recvfrom( connection->socketFD, connection->buffer, IP_MAX_MESSAGE_LENGTH, MSG_PEEK, (struct sockaddr *) &address, &addressLength ) == SOCKET_ERROR )
  {
    ERROR_PRINT( "recvfrom: error reading from socket %d", connection->socketFD );
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
    
      DEBUG_PRINT( "socket %d received right message: %s", connection->socketFD, connection->buffer );
    
      return connection->buffer;
    }
  }
  
  // Default return message (the received one was not destined to this connection) 
  return empty_message;
}

// Send given message through the given UDP connection
static int SendUDPMessage( IPConnection connection, const char* message )
{
  if( strlen( message ) + 1 > IP_MAX_MESSAGE_LENGTH )
  {
    ERROR_PRINT( "message too long (%lu bytes) !", strlen( message ) );
    return 0;
  }
  
  DEBUG_PRINT( "connection socket %d sending message: %s", connection->socketFD, message );
  
  if( sendto( connection->socketFD, message, IP_MAX_MESSAGE_LENGTH, 0, (struct sockaddr *) connection->address, sizeof(struct sockaddr_in6) ) == SOCKET_ERROR )
  {
    ERROR_PRINT( "sendto: error writing to socket %d", connection->socketFD );
    return -1;
  }
  
  return 0;
}

// Send given message to all the clients of the given server connection
static int SendMessageAll( IPConnection connection, const char* message )
{
  static size_t clientIndex = 0, clientsNumber;
    clientsNumber = *(connection->ref_clientsCount);
  while( clientIndex < clientsNumber )
  {
    if( connection->clientsList[ clientIndex ] != NULL )
    {
      IPNetwork_SendMessage( connection->clientsList[ clientIndex ], message );
      clientIndex++;
    }
  }
  
  return 0;
}

// Waits for a remote connection to be added to the client list of the given TCP server connection
static IPConnection AcceptTCPClient( IPConnection server )
{
  IPConnection client;
  int clientSocketFD;
  static struct sockaddr_storage clientAddress;
  static socklen_t addressLength = sizeof(clientAddress);
  
  clientSocketFD = accept( server->socketFD, (struct sockaddr *) &clientAddress, &addressLength );

  if( clientSocketFD == INVALID_SOCKET )
  {
    ERROR_PRINT( "accept: failed accepting connection on socket %d", server->socketFD );
    return NULL;
  }
  
  DEBUG_PRINT( 0, "client accepted: socket fd: %d\n", clientSocketFD );
  
  client =  AddConnection( clientSocketFD, (struct sockaddr *) &clientAddress, CLIENT | TCP );

  AddClient( server, client );

  return client;
}

// Waits for a remote connection to be added to the client list of the given UDP server connection
static IPConnection AcceptUDPClient( IPConnection server )
{
  size_t clientIndex, clientsNumber;
  IPConnection client;
  static struct sockaddr_storage clientAddress;
  static socklen_t addressLength = sizeof(clientAddress);
  static char buffer[ IP_MAX_MESSAGE_LENGTH ];
  static uint8_t* addressString[2];
  static IPConnection dummyConnection;

  if( recvfrom( server->socketFD, buffer, IP_MAX_MESSAGE_LENGTH, MSG_PEEK, (struct sockaddr *) &clientAddress, &addressLength ) == SOCKET_ERROR )
  {
    ERROR_PRINT( "recvfrom: error reading from socket %d", server->socketFD );
    return NULL;
  }
  
  // Verify if incoming message belongs to unregistered client (returns default value if not)
  clientsNumber = *(server->ref_clientsCount);
  for( clientIndex = 0; clientIndex < clientsNumber; clientIndex++ )
  {
    //DEBUG_PRINT( "comparing ports: %u - %u", server->clientsList[ clientIndex ]->address->sin6_port, ((struct sockaddr_in6*) &clientAddress)->sin6_port );
    
    if( server->clientsList[ clientIndex ]->address->sin6_port == ((struct sockaddr_in6*) &clientAddress)->sin6_port )
    {
      addressString[0] = server->clientsList[ clientIndex ]->address->sin6_addr.s6_addr;
      addressString[1] = ((struct sockaddr_in6*) &clientAddress)->sin6_addr.s6_addr;
      if( strncmp( (const char*) addressString[0], (const char*) addressString[1], sizeof(struct in6_addr) ) == 0 )
        return dummyConnection;
    }
  }
  
  DEBUG_PRINT( "client accepted (clients count before: %lu)", *(server->ref_clientsCount) );
  
  client = AddConnection( server->socketFD, (struct sockaddr*) &clientAddress, CLIENT | UDP );

  AddClient( server, client );
  
  //DEBUG_PRINT( "client accepted (clients count after: %lu)", *(server->ref_clientsCount) );
  
  return client;
}


//////////////////////////////////////////////////////////////////////////////////
/////                               FINALIZING                               /////
//////////////////////////////////////////////////////////////////////////////////

// Handle proper destruction of any given connection type
void IPNetwork_Close( IPConnection connection )
{
  if( connection != NULL )
  {
    //DEBUG_PRINT( "closing connection for socket %d (users count before: %u)", connection->socketFD, *(connection->ref_clientsCount) );

    // Each TCP connection has its own socket, so we can close it without problem. But UDP connections
    // from the same server share the socket, so we need to wait for all of them to be stopped to close the socket
    if( connection->protocol == TCP )
    {
      //DEBUG_PRINT( "closing TCP connection unused socket fd: %d", connection->socketFD );
      FD_CLR( connection->socketFD, &socketReadFDs );
      shutdown( connection->socketFD, SHUT_RDWR );
    }
    // Check number of client connections of a server (also of sharers of a socket for UDP connections)
    if( *(connection->ref_clientsCount) <= 0 )
    {
      if( connection->protocol == UDP )
      {
        //DEBUG_PRINT( "closing UDP connection unused socket fd: %d", connection->socketFD );
        
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
      //DEBUG_PRINT( "freeing client connection (socket %d) message buffer", connection->socketFD );

      free( connection->buffer );
      connection->buffer = NULL;
    }
    else if( connection->networkRole == SERVER )
    {
      //DEBUG_PRINT( "cleaning server connection (socket %d) client list", connection->socketFD );
      
      if( connection->clientsList != NULL )
      {
        free( connection->clientsList );
        connection->clientsList = NULL;
      }
    }
  
    //DEBUG_PRINT( 0, "closing connection for socket %d (users count after: %u)", connection->socketFD, 
                                                   ( connection->ref_clientsCount == NULL ) ? 0 : *(connection->ref_clientsCount) );
  
    free( connection );
    connection = NULL;
  }

  return;
}

#endif /* IP_CONNECTION_H */
