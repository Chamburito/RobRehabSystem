/////////////////////////////////////////////////////////////////////////////////////
///// Multiplatform library for creation and handling of IP sockets connections /////
///// as server or client, using TCP or UDP protocols                           /////
/////////////////////////////////////////////////////////////////////////////////////

#ifndef IP_NETWORK_H
#define IP_NETWORK_H

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
  #define poll WSAPoll
  
  typedef SOCKET Socket;
  typedef WSAPOLLFD PollSocket;
#else
  #include <fcntl.h>
  #include <unistd.h>
  #include <errno.h>
  #include <sys/types.h>
  #include <sys/socket.h>
  #include <stropts.h>
  #include <poll.h>
  #include <netinet/in.h>
  #include <arpa/inet.h>
  #include <netdb.h>

  const int SOCKET_ERROR = -1;
  const int INVALID_SOCKET = -1;

  typedef int Socket;
  typedef struct pollfd PollSocket;
#endif
  
#ifndef IP_NETWORK_LEGACY
typedef struct sockaddr_in6 IPAddressData;          // IPv6 structure can store both IPv4 and IPv6 data
#else
typedef struct sockaddr_in IPAddressData;           // Legacy mode only works with IPv4 addresses
#endif
typedef struct sockaddr* IPAddress;                 // Opaque IP address type

#include "interfaces.h"
  
#include "debug/sync_debug.h"

/////////////////////////////////////////////////////////////////////////  
/////                    PREPROCESSOR DIRECTIVES                    /////
/////////////////////////////////////////////////////////////////////////
  
#define IP_MAX_MESSAGE_LENGTH 512
#define IP_CONNECTIONS_MAX 1024
#define QUEUE_SIZE 20

const int IP_CONNECTION_INVALID_ID = -1;

#define IP_ADDRESS_LENGTH INET6_ADDRSTRLEN                    // Maximum length of IPv6 address (host+port) string
#define IP_PORT_LENGTH 6                                      // Maximum length of short integer string representation
#define IP_HOST_LENGTH IP_ADDRESS_LENGTH - IP_PORT_LENGTH     // Maximum length of IPv6 host string

#define IP_HOST 0
#define IP_PORT IP_HOST_LENGTH

#define IP_TCP 0x10
#define IP_UDP 0x20


//////////////////////////////////////////////////////////////////////////
/////                         DATA STRUCTURES                        /////
//////////////////////////////////////////////////////////////////////////

typedef struct _IPConnectionData IPConnectionData;
typedef IPConnectionData* IPConnection;

// Generic structure to store methods and data of any connection type handled by the library
struct _IPConnectionData
{
  //Socket socketFD;
  PollSocket* socket;
  union {
    char* (*ref_ReceiveMessage)( IPConnection );
    IPConnection (*ref_AcceptClient)( IPConnection );
  };
  int (*ref_SendMessage)( IPConnection, const char* );
  void (*ref_Close)( IPConnection );
  IPAddressData addressData;
  size_t messageLength;
  union {
    char* buffer;
    IPConnection* clientsList;
  };
  union {
    size_t* ref_clientsCount;
    IPConnection server;
  };
};


///////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                        GLOBAL VARIABLES                                         /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

static PollSocket availableSocketsList[ IP_CONNECTIONS_MAX ];
static size_t availableSocketsNumber = 0;


///////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                            INTERFACE                                            /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

#define IP_NETWORK_INTERFACE( Namespace, INIT_FUNCTION ) \
        INIT_FUNCTION( char*, Namespace, GetAddress, IPConnection ) \
        INIT_FUNCTION( size_t, Namespace, GetClientsNumber, IPConnection ) \
        INIT_FUNCTION( size_t, Namespace, SetMessageLength, IPConnection, size_t ) \
        INIT_FUNCTION( bool, Namespace, IsServer, IPConnection ) \
        INIT_FUNCTION( IPConnection, Namespace, OpenConnection, const char*, const char*, uint8_t ) \
        INIT_FUNCTION( void, Namespace, CloseConnection, IPConnection ) \
        INIT_FUNCTION( char*, Namespace, ReceiveMessage, IPConnection ) \
        INIT_FUNCTION( int, Namespace, SendMessage, IPConnection, const char* ) \
        INIT_FUNCTION( IPConnection, Namespace, AcceptClient, IPConnection ) \
        INIT_FUNCTION( int, Namespace, WaitEvent, unsigned int ) \
        INIT_FUNCTION( bool, Namespace, IsDataAvailable, IPConnection )

INIT_NAMESPACE_INTERFACE( IPNetwork, IP_NETWORK_INTERFACE )

// Forward Declaration
static char* ReceiveTCPMessage( IPConnection );
static char* ReceiveUDPMessage( IPConnection );
static int SendTCPMessage( IPConnection, const char* );
static int SendUDPMessage( IPConnection, const char* );
static int SendMessageAll( IPConnection, const char* );
static IPConnection AcceptTCPClient( IPConnection );
static IPConnection AcceptUDPClient( IPConnection );
static void CloseTCPServer( IPConnection );
static void CloseUDPServer( IPConnection );
static void CloseTCPClient( IPConnection );
static void CloseUDPClient( IPConnection );

/////////////////////////////////////////////////////////////////////////////
/////                         NETWORK UTILITIES                         /////
/////////////////////////////////////////////////////////////////////////////

// System calls for getting IP address strings
char* GetAddressString( IPAddress address )
{
  static char addressString[ IP_ADDRESS_LENGTH ];
  
  DEBUG_PRINT( "getting address string for address pointer %p", address );
  
  char* hostString = &addressString[ IP_HOST ];
  char* portString = &addressString[ IP_PORT ];
  #ifndef IP_NETWORK_LEGACY
  int flags = NI_NUMERICHOST | NI_NUMERICSERV;
  int error = getnameinfo( address, sizeof(IPAddressData), hostString, IP_HOST_LENGTH, portString, IP_PORT_LENGTH, flags );
  if( error != 0 )
  {
    ERROR_PRINT( "getnameinfo: failed getting address string: %s", gai_strerror( error ) );
    return NULL;
  }
  #else
  sprintf( hostString, "%s", inet_ntoa( ((IPAddressData*) address)->sin_addr ) );
  sprintf( portString, "%d", ((IPAddressData*) address)->sin_port ) );
  #endif
  
  return addressString;
}
// Utility method to request an address (host and port) string for client connections (returns default values for server connections)
char* IPNetwork_GetAddress( IPConnection connection )
{
  if( connection == NULL ) return NULL;
  
  return GetAddressString( (IPAddress) &(connection->addressData) );
}

// Returns number of active clients for a connection 
size_t IPNetwork_GetClientsNumber( IPConnection connection )
{
  if( connection == NULL ) return 0;
  
  if( IPNetwork_IsServer( connection ) )
    return *(connection->ref_clientsCount);
  
  return 1;
}

bool IPNetwork_IsServer( IPConnection connection )
{
  if( connection == NULL ) return false;
  
  return ( connection->ref_SendMessage == SendMessageAll ) ? true : false;
}


//////////////////////////////////////////////////////////////////////////////////
/////                             INITIALIZATION                             /////
//////////////////////////////////////////////////////////////////////////////////

static int CompareSockets( const void* ref_socket_1, const void* ref_socket_2 )
{
  return ( ((struct pollfd*) ref_socket_1)->fd - ((struct pollfd*) ref_socket_2)->fd );
}

// Handle construction of a IPConnection structure with the defined properties
static IPConnection AddConnection( Socket socketFD, IPAddress address, uint8_t transportProtocol, bool isServer )
{
  IPConnection connection = (IPConnection) malloc( sizeof(IPConnectionData) );
  memset( connection, 0, sizeof(IPConnectionData) );
  
  PollSocket cmpSocket = { .fd = socketFD };
  connection->socket = (PollSocket*) bsearch( &cmpSocket, availableSocketsList, availableSocketsNumber, sizeof(PollSocket), CompareSockets );
  if( connection->socket == NULL )
  {
    connection->socket = &(availableSocketsList[ availableSocketsNumber ]);
    connection->socket->fd = socketFD;
    connection->socket->events = POLLRDNORM | POLLRDBAND;
    availableSocketsNumber++;
  }
  
  connection->messageLength = IP_MAX_MESSAGE_LENGTH;

  DEBUG_PRINT( "socket: %d - type: %x", connection->socket->fd, ( transportProtocol | isServer ) );
  
  memcpy( &(connection->addressData), address, sizeof(IPAddressData) );
  connection->ref_clientsCount = malloc( sizeof(size_t) );
  
  if( isServer ) // Server role connection
  {
    connection->clientsList = NULL;
    connection->ref_AcceptClient = ( transportProtocol == IP_TCP ) ? AcceptTCPClient : AcceptUDPClient;
    connection->ref_SendMessage = SendMessageAll;
    connection->ref_Close = ( transportProtocol == IP_TCP ) ? CloseTCPServer : CloseUDPServer;
    *(connection->ref_clientsCount) = 0;
  }
  else
  { 
    char* addressString = GetAddressString( (IPAddress) &(connection->addressData) );
    DEBUG_PRINT( "connection added:\n\tfamily: %x\n\tport: %s\n\thost: %s", transportProtocol, &addressString[ IP_PORT ], &addressString[ IP_HOST ] );

    //connection->address->sin6_family = AF_INET6;
    connection->buffer = (char*) calloc( IP_MAX_MESSAGE_LENGTH, sizeof(char) );
    connection->ref_ReceiveMessage = ( transportProtocol == IP_TCP ) ? ReceiveTCPMessage : ReceiveUDPMessage;
    connection->ref_SendMessage = ( transportProtocol == IP_TCP ) ? SendTCPMessage : SendUDPMessage;
    connection->ref_Close = ( transportProtocol == IP_TCP ) ? CloseTCPClient : CloseUDPClient;
    connection->server = NULL;
  }
  
  return connection;
}

// Add defined connection to the client list of the given server connection
static inline void AddClient( IPConnection server, IPConnection client )
{
  size_t clientIndex = 0, clientsListSize = 0;
  
  client->server = server;
  
  size_t clientsNumber = *((size_t*) server->ref_clientsCount);
  while( clientIndex < clientsNumber )
  {
    clientsListSize++;
    if( server->clientsList[ clientIndex ] == NULL ) break;
    clientIndex++;      
  }
  
  if( clientIndex == clientsListSize ) server->clientsList = (IPConnection*) realloc( server->clientsList, ( clientIndex + 1 ) * sizeof(IPConnection) );
  
  server->clientsList[ clientIndex ] = client;
  
  (*(server->ref_clientsCount))++;

  return;
}

IPAddress LoadAddressInfo( const char* host, const char* port )
{
  static IPAddressData addressData;
  
  struct addrinfo hints = { 0 };
  struct addrinfo* hostsInfoList;
  struct addrinfo* hostInfo = NULL;
  
  #ifdef WIN32
  static WSADATA wsa;
  if( wsa.wVersion == 0 )
  {
    if( WSAStartup( MAKEWORD( 2, 2 ), &wsa ) != 0 )
    {
      fprintf( stderr, "%s: error initialiasing windows sockets: code: %d", __func__, WSAGetLastError() );
      return NULL;
    }
  }
  #endif
  
  #ifndef IP_NETWORK_LEGACY
  if( host == NULL )
  {
    hints.ai_flags |= AI_PASSIVE; // Set address for me
    hints.ai_family = AF_INET6;
  }
  else
    hints.ai_family = AF_UNSPEC; // AF_INET6 (IPv6), AF_INET (IPv4) or AF_UNSPEC
  
  int errorCode = 0;
  if( (errorCode = getaddrinfo( host, port, &hints, &hostsInfoList )) != 0 )
  {
    ERROR_PRINT( "getaddrinfo: error reading host info: %s", gai_strerror( errorCode ) );
    return NULL;
  }
  
  // loop through all the results and bind to the first we can
  for( hostInfo = hostsInfoList; hostInfo != NULL; hostInfo = hostInfo->ai_next ) 
  {
    // Extended connection info for debug builds
    char* addressString = GetAddressString( hostInfo->ai_addr );
    if( addressString == NULL ) continue;
    
    DEBUG_PRINT( "trying to %s to: host: %s - port: %s - protocol: %s - version: %s", ( host == NULL ) ? "bind" : "connect", 
                                                                                      &addressString[ IP_HOST ], &addressString[ IP_PORT ],
                                                                                      ( hostInfo->ai_protocol == IPPROTO_TCP ) ? "TCP" : "UDP",
                                                                                      ( hostInfo->ai_family == AF_INET6 ) ? "IPv6" : "IPv4" );
    
    memcpy( &addressData, hostInfo->ai_addr, hostInfo->ai_addrlen );
    break;
  }
  
  freeaddrinfo( hostsInfoList ); // Don't need this struct anymore
  
  if( hostInfo == NULL ) return NULL;
  #else
  addressData.sin_family = AF_INET;
  addressData.sin_port = htons( (short) portNumber );
  if( host == NULL ) addressData.sin_addr.s_addr = htonl( INADDR_ANY );
  else if( inet_aton( host, &(addressData.sin_addr) ) == 0 ) 
    return NULL;
  #endif
  
  return (IPAddress) &addressData;
}

int CreateSocket( uint8_t protocol, IPAddress address )
{
  int socketType, transportProtocol;
  
  if( protocol == IP_TCP ) 
  {
    socketType = SOCK_STREAM;
    transportProtocol = IPPROTO_TCP;
  }
  else if( protocol == IP_UDP ) 
  {
    socketType = SOCK_DGRAM;
    transportProtocol = IPPROTO_UDP;
  }
  else
  {
    DEBUG_PRINT( "invalid protocol option: %x", protocol );
    return INVALID_SOCKET;
  }
  
  // Create IP socket
  int socketFD = socket( address->sa_family, socketType, transportProtocol );
  if( socketFD == INVALID_SOCKET )
    ERROR_PRINT( "socket: failed opening %s %s socket", ( protocol == IP_TCP ) ? "TCP" : "UDP", ( address->sa_family == AF_INET6 ) ? "IPv6" : "IPv4" );                                                              
  
  return socketFD;
}

bool SetSocketConfig( int socketFD )
{
  #ifdef WIN32
  u_long nonBlocking = 1;
  if( ioctlsocket( socketFD, FIONBIO, &nonBlocking ) == SOCKET_ERROR )
  #else
  if( fcntl( socketFD, F_SETFL, O_NONBLOCK ) == SOCKET_ERROR )
  #endif
  {
    ERROR_PRINT( "failure setting socket %d to non-blocking state", socketFD );
    close( socketFD );
    return false;
  }
  
  int reuseAddress = 1;
  // Allow sockets to be binded to the same local port
  if( setsockopt( socketFD, SOL_SOCKET, SO_REUSEADDR, (const char*) &reuseAddress, sizeof(reuseAddress) ) == SOCKET_ERROR ) 
  {
    ERROR_PRINT( "setsockopt: failed setting socket %d option SO_REUSEADDR", socketFD );
    close( socketFD );
    return NULL;
  }
  
  return true;
}

bool BindUDPServerSocket( int socketFD, IPAddress address )
{
  if( address->sa_family == AF_INET6 )
  {
    int ipv6Only = 0;
    // Let IPV6 servers accept IPV4 clients
    if( setsockopt( socketFD, IPPROTO_IPV6, IPV6_V6ONLY, (const char*) &ipv6Only, sizeof(ipv6Only) ) == SOCKET_ERROR )
    {
      ERROR_PRINT( "setsockopt: failed setting socket %d option IPV6_V6ONLY", socketFD );
      close( socketFD );
      return false;
    }
  }
  
  // Bind server socket to the given local address
  size_t addressLength = ( address->sa_family == AF_INET6 ) ? sizeof(struct sockaddr_in6) : sizeof(struct sockaddr_in);
  if( bind( socketFD, address, addressLength ) == SOCKET_ERROR )
  {
    ERROR_PRINT( "bind: failed on binding socket %d", socketFD );
    close( socketFD );
    return false;
  }
  
  return true;
}

bool BindTCPServerSocket( int socketFD, IPAddress address )
{
  if( !BindUDPServerSocket( socketFD, address ) ) return false;
  
  // Set server socket to listen to remote connections
  if( listen( socketFD, QUEUE_SIZE ) == SOCKET_ERROR )
  {
    ERROR_PRINT( "listen: failed listening on socket %d", socketFD );
    close( socketFD );
    return false;
  }
  
  return true;
}

bool ConnectTCPClientSocket( int socketFD, IPAddress address )
{
  // Connect TCP client socket to given remote address
  size_t addressLength = ( address->sa_family == AF_INET6 ) ? sizeof(struct sockaddr_in6) : sizeof(struct sockaddr_in);
  if( connect( socketFD, address, addressLength ) == SOCKET_ERROR )
  {
    ERROR_PRINT( "connect: failed on connecting socket %d to remote address", socketFD );
    close( socketFD );
    return false;
  }
  
  return true;
}

bool ConnectUDPClientSocket( int socketFD, IPAddress address )
{
  // Bind UDP client socket to available local address
  static struct sockaddr_storage localAddress;
  localAddress.ss_family = address->sa_family;
  if( bind( socketFD, (struct sockaddr*) &localAddress, sizeof(localAddress) ) == SOCKET_ERROR )
  {
    ERROR_PRINT( "bind: failed on binding socket %d to arbitrary local port", socketFD );
    close( socketFD );
    return false;
  }
  
  return true;
}

// Generic method for opening a new socket and providing a corresponding IPConnection structure for use
IPConnection IPNetwork_OpenConnection( const char* host, const char* port, uint8_t transportProtocol )
{
  // Assure that the port number is in the Dynamic/Private range (49152-65535)
  unsigned int portNumber = (unsigned int) strtoul( port, NULL, 0 );
  if( portNumber < 49152 || portNumber > 65535 )
  {
    ERROR_PRINT( "invalid port number value: %s", port );
    return NULL;
  }
  
  IPAddress address = LoadAddressInfo( host, port );
  if( address == NULL ) return NULL;
  
  Socket socketFD = CreateSocket( transportProtocol, address );
  if( socketFD == INVALID_SOCKET ) return NULL;
  
  bool isServer = ( host == NULL ) ? true : false;
  
  if( !SetSocketConfig( socketFD ) ) return NULL;
  
  if( isServer )
  {
    if( transportProtocol == IP_TCP )
    {
      if( !BindTCPServerSocket( socketFD, address ) ) return NULL;
    }
    else //if( transportProtocol == IP_UDP )
    {
      if( !BindUDPServerSocket( socketFD, address ) ) return NULL;
    }
  }
  else
  {
    if( transportProtocol == IP_TCP )
    {
      if( !ConnectTCPClientSocket( socketFD, address ) ) return NULL;
    }
    else //if( transportProtocol == IP_UDP )
    {
      if( !ConnectUDPClientSocket( socketFD, address ) ) return NULL;
    }
  }  
  
  return AddConnection( socketFD, address, transportProtocol, isServer ); // Build the IPConnection structure
}

size_t IPNetwork_SetMessageLength( IPConnection connection, size_t messageLength )
{
  if( connection == NULL ) return 0;
  
  connection->messageLength = ( messageLength > IP_MAX_MESSAGE_LENGTH ) ? IP_MAX_MESSAGE_LENGTH : (uint16_t) messageLength;
  
  return (size_t) connection->messageLength;
}


/////////////////////////////////////////////////////////////////////////////////////////
/////                             GENERIC COMMUNICATION                             /////
/////////////////////////////////////////////////////////////////////////////////////////

inline char* IPNetwork_ReceiveMessage( IPConnection connection ) 
{ 
  memset( connection->buffer, 0, IP_MAX_MESSAGE_LENGTH );
  
  return connection->ref_ReceiveMessage( connection ); 
}

inline int IPNetwork_SendMessage( IPConnection connection, const char* message ) 
{ 
  if( strlen( message ) + 1 > connection->messageLength )
  {
    ERROR_PRINT( "message too long (%lu bytes for %lu max) !", strlen( message ), connection->messageLength );
    return 0;
  }
  
  //DEBUG_PRINT( "connection socket %d sending message: %s", connection->socket->fd, message );
  
  return connection->ref_SendMessage( connection, message ); 
}

inline IPConnection IPNetwork_AcceptClient( IPConnection connection ) { return connection->ref_AcceptClient( connection ); }

// Verify available incoming messages for the given connection, preventing unnecessary blocking calls (for syncronous networking)
int IPNetwork_WaitEvent( unsigned int milliseconds )
{
  int eventsNumber = poll( availableSocketsList, availableSocketsNumber, milliseconds );
  
  if( eventsNumber == SOCKET_ERROR ) ERROR_PRINT( "select: error waiting for events on %lu FDs", availableSocketsNumber );
    
  return eventsNumber;
}

bool IPNetwork_IsDataAvailable( IPConnection connection )
{
  if( connection == NULL ) return false;
  
  if( connection->socket->revents & POLLRDNORM ) return true;
  else if( connection->socket->revents & POLLRDBAND ) return true;
  
  return false;
}

/////////////////////////////////////////////////////////////////////////////////////////
/////                      SPECIFIC TRANSPORT/ROLE COMMUNICATION                    /////
/////////////////////////////////////////////////////////////////////////////////////////

// Try to receive incoming message from the given TCP client connection and store it on its buffer
static char* ReceiveTCPMessage( IPConnection connection )
{
  int bytesReceived;
  
  // Blocks until there is something to be read in the socket
  bytesReceived = recv( connection->socket->fd, connection->buffer, connection->messageLength, 0 );

  if( bytesReceived == SOCKET_ERROR )
  {
    ERROR_PRINT( "recv: error reading from socket %d", connection->socket->fd );
    return NULL;
  }
  else if( bytesReceived == 0 )
  {
    ERROR_PRINT( "recv: remote connection with socket %d closed", connection->socket->fd );
    return NULL;
  }
  
  //DEBUG_PRINT( "socket %d received message: %s", connection->socketFD, connection->buffer );
  
  return connection->buffer;
}

// Send given message through the given TCP connection
static int SendTCPMessage( IPConnection connection, const char* message )
{
  if( send( connection->socket->fd, message, connection->messageLength, 0 ) == SOCKET_ERROR )
  {
    ERROR_PRINT( "send: error writing to socket %d", connection->socket->fd );
    return -1;
  }
  
  return 0;
}

// Try to receive incoming message from the given UDP client connection and store it on its buffer
static char* ReceiveUDPMessage( IPConnection connection )
{
  IPAddressData address = { 0 };
  socklen_t addressLength;
  static char* emptyMessage = "";//{ '\0' };

  // Blocks until there is something to be read in the socket
  if( recvfrom( connection->socket->fd, connection->buffer, connection->messageLength, MSG_PEEK, (IPAddress) &address, &addressLength ) == SOCKET_ERROR )
  {
    ERROR_PRINT( "recvfrom: error reading from socket %d", connection->socket->fd );
    return NULL;
  }

  // Verify if incoming message is destined to this connection (and returns the message if it is)
  if( memcmp( (void*) &(connection->addressData), (void*) &address, addressLength ) == 0 )
  {
    recv( connection->socket->fd, NULL, 0, 0 );
    DEBUG_PRINT( "socket %d received right message: %s", connection->socket->fd, connection->buffer );
    return connection->buffer;
  }
  
  // Default return message (the received one was not destined to this connection) 
  return emptyMessage;
}

// Send given message through the given UDP connection
static int SendUDPMessage( IPConnection connection, const char* message )
{
  if( sendto( connection->socket->fd, message, connection->messageLength, 0, (IPAddress) &(connection->addressData), sizeof(IPAddressData) ) == SOCKET_ERROR )
  {
    ERROR_PRINT( "sendto: error writing to socket %d", connection->socket->fd );
    return -1;
  }
  
  return 0;
}

// Send given message to all the clients of the given server connection
static int SendMessageAll( IPConnection connection, const char* message )
{
  size_t clientIndex = 0;
  size_t clientsNumber = *((size_t*) connection->ref_clientsCount);
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
  
  clientSocketFD = accept( server->socket->fd, (struct sockaddr *) &clientAddress, &addressLength );

  if( clientSocketFD == INVALID_SOCKET )
  {
    ERROR_PRINT( "accept: failed accepting connection on socket %d", server->socket->fd );
    return NULL;
  }
  
  DEBUG_PRINT( "client accepted: socket fd: %d\n", clientSocketFD );
  
  client =  AddConnection( clientSocketFD, (IPAddress) &clientAddress, IP_TCP, false );

  AddClient( server, client );

  return client;
}

// Waits for a remote connection to be added to the client list of the given UDP server connection
static IPConnection AcceptUDPClient( IPConnection server )
{
  static char buffer[ IP_MAX_MESSAGE_LENGTH ];
  static IPConnectionData dummyConnection;

  struct sockaddr_storage clientAddress = { 0 };
  socklen_t addressLength = sizeof(clientAddress);
  if( recvfrom( server->socket->fd, buffer, IP_MAX_MESSAGE_LENGTH, MSG_PEEK, (IPAddress) &clientAddress, &addressLength ) == SOCKET_ERROR )
  {
    ERROR_PRINT( "recvfrom: error reading from socket %d", server->socket->fd );
    return NULL;
  }
  
  // Verify if incoming message belongs to unregistered client (returns default value if not)
  size_t clientsNumber = *(server->ref_clientsCount);
  for( size_t clientIndex = 0; clientIndex < clientsNumber; clientIndex++ )
  {
    //DEBUG_PRINT( "comparing ports: %u - %u", server->clientsList[ clientIndex ]->address->sin6_port, ((struct sockaddr_in6*) &clientAddress)->sin6_port );
    if( memcmp( (void*) &(server->clientsList[ clientIndex ]->addressData), (void*) &clientAddress, addressLength ) == 0 )
      return &dummyConnection;
  }
  
  DEBUG_PRINT( "client accepted (clients count before: %lu)", *(server->ref_clientsCount) );
  
  IPConnection client = AddConnection( server->socket->fd, (IPAddress) &clientAddress, IP_UDP, false );

  AddClient( server, client );
  
  //DEBUG_PRINT( "client accepted (clients count after: %lu)", *(server->ref_clientsCount) );
  
  return client;
}


//////////////////////////////////////////////////////////////////////////////////
/////                               FINALIZING                               /////
//////////////////////////////////////////////////////////////////////////////////

// Handle proper destruction of any given connection type
void CloseTCPServer( IPConnection server )
{
  DEBUG_PRINT( "closing TCP server unused socket fd: %d", server->socket->fd );
  shutdown( server->socket->fd, SHUT_RDWR );
  free( server->ref_clientsCount );
  if( server->clientsList != NULL ) free( server->clientsList );
  free( server );
}

void CloseUDPServer( IPConnection server )
{
  // Check number of client connections of a server (also of sharers of a socket for UDP connections)
  if( *(server->ref_clientsCount) == 0 )
  {
    DEBUG_PRINT( "closing UDP server unused socket fd: %d", server->socket->fd );
    close( server->socket->fd );
    free( server->ref_clientsCount );
    if( server->clientsList != NULL ) free( server->clientsList );
    free( server );
  }
}

static inline void RemoveClient( IPConnection server, IPConnection client )
{
  if( server == NULL ) return;
  
  size_t clientsNumber = *(server->ref_clientsCount);
  for( size_t clientIndex = 0; clientIndex < clientsNumber; clientIndex++ )
  {
    if( server->clientsList[ clientIndex ] == client )
    {
      server->clientsList[ clientIndex ] = NULL;
      (*(server->ref_clientsCount))--;
      break;
    }
  }
}

void CloseTCPClient( IPConnection client )
{
  DEBUG_PRINT( "closing TCP client unused socket fd: %d", client->socket->fd );
  RemoveClient( client->server, client );
  shutdown( client->socket->fd, SHUT_RDWR );
  if( client->buffer != NULL ) free( client->buffer );
  free( client );
}

void CloseUDPClient( IPConnection client )
{
  DEBUG_PRINT( "closing UDP client unused socket fd: %d", client->socket->fd );
  RemoveClient( client->server, client );
  
  if( client->server == NULL ) close( client->socket->fd );
  else if( *(client->server->ref_clientsCount) == 0 ) CloseUDPServer( client->server );

  if( client->buffer != NULL ) free( client->buffer );
  free( client );
}

void IPNetwork_CloseConnection( IPConnection connection )
{
  if( connection == NULL ) return;
  
  DEBUG_PRINT( "closing connection for socket %d", connection->socket->fd );

  // Each TCP connection has its own socket, so we can close it without problem. But UDP connections
  // from the same server share the socket, so we need to wait for all of them to be stopped to close the socket
  connection->ref_Close( connection );
}

#endif /* IP_NETWORK_H */
