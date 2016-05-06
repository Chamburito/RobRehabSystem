/////////////////////////////////////////////////////////////////////////////////////
///// Multiplatform library for creation and handling of IP sockets connections /////
///// as server or client, using TCP or UDP protocols                           /////
/////////////////////////////////////////////////////////////////////////////////////

#ifndef IP_NETWORK_H
#define IP_NETWORK_H

#ifdef WIN32
  //#include <winsock2.h>
  #include <ws2tcpip.h>
#else
  #include <netinet/in.h>
#endif

#include "interfaces.h"
  
#include "debug/sync_debug.h"

/////////////////////////////////////////////////////////////////////////  
/////                    PREPROCESSOR DIRECTIVES                    /////
/////////////////////////////////////////////////////////////////////////
  
#define IP_MAX_MESSAGE_LENGTH 512
#define IP_CONNECTIONS_MAX 1024

#ifndef IP_NETWORK_LEGACY
  #define IP_ADDRESS_LENGTH INET6_ADDRSTRLEN                  // Maximum length of IPv6 address (host+port) string
#else
  #define IP_ADDRESS_LENGTH INET_ADDRSTRLEN                   // Maximum length of IPv4 address (host+port) string
#endif
#define IP_PORT_LENGTH 6                                      // Maximum length of short integer string representation
#define IP_HOST_LENGTH IP_ADDRESS_LENGTH - IP_PORT_LENGTH     // Maximum length of host string

#define IP_HOST 0
#define IP_PORT IP_HOST_LENGTH

#define IP_SERVER 0x01
#define IP_CLIENT 0x02

#define IP_TCP 0x10
#define IP_UDP 0x20


//////////////////////////////////////////////////////////////////////////
/////                         DATA STRUCTURES                        /////
//////////////////////////////////////////////////////////////////////////

typedef struct _IPConnectionData IPConnectionData;
typedef IPConnectionData* IPConnection;


///////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                            INTERFACE                                            /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

// #define IP_NETWORK_INTERFACE( Namespace, INIT_FUNCTION ) \
//         INIT_FUNCTION( char*, Namespace, GetAddress, IPConnection ) \
//         INIT_FUNCTION( size_t, Namespace, GetClientsNumber, IPConnection ) \
//         INIT_FUNCTION( size_t, Namespace, SetMessageLength, IPConnection, size_t ) \
//         INIT_FUNCTION( bool, Namespace, IsServer, IPConnection ) \
//         INIT_FUNCTION( IPConnection, Namespace, OpenConnection, uint8_t, const char*, uint16_t ) \
//         INIT_FUNCTION( void, Namespace, CloseConnection, IPConnection ) \
//         INIT_FUNCTION( char*, Namespace, ReceiveMessage, IPConnection ) \
//         INIT_FUNCTION( int, Namespace, SendMessage, IPConnection, const char* ) \
//         INIT_FUNCTION( IPConnection, Namespace, AcceptClient, IPConnection ) \
//         INIT_FUNCTION( int, Namespace, WaitEvent, unsigned int ) \
//         INIT_FUNCTION( bool, Namespace, IsDataAvailable, IPConnection )
// 
// INIT_NAMESPACE_INTERFACE( IPNetwork, IP_NETWORK_INTERFACE )
        
char* IPNetwork_GetAddress( IPConnection ); size_t IPNetwork_GetClientsNumber( IPConnection ); size_t IPNetwork_SetMessageLength( IPConnection, size_t ); _Bool IPNetwork_IsServer( IPConnection ); IPConnection IPNetwork_OpenConnection( uint8_t, const char*, uint16_t ); void IPNetwork_CloseConnection( IPConnection ); char* IPNetwork_ReceiveMessage( IPConnection ); int IPNetwork_SendMessage( IPConnection, const char* ); IPConnection IPNetwork_AcceptClient( IPConnection ); int IPNetwork_WaitEvent( unsigned int ); _Bool IPNetwork_IsDataAvailable( IPConnection ); 

const struct { char* (*GetAddress)( IPConnection ); size_t (*GetClientsNumber)( IPConnection ); size_t (*SetMessageLength)( IPConnection, size_t ); _Bool (*IsServer)( IPConnection ); IPConnection (*OpenConnection)( uint8_t, const char*, uint16_t ); void (*CloseConnection)( IPConnection ); char* (*ReceiveMessage)( IPConnection ); int (*SendMessage)( IPConnection, const char* ); IPConnection (*AcceptClient)( IPConnection ); int (*WaitEvent)( unsigned int ); _Bool (*IsDataAvailable)( IPConnection ); } IPNetwork = { .GetAddress = IPNetwork_GetAddress, .GetClientsNumber = IPNetwork_GetClientsNumber, .SetMessageLength = IPNetwork_SetMessageLength, .IsServer = IPNetwork_IsServer, .OpenConnection = IPNetwork_OpenConnection, .CloseConnection = IPNetwork_CloseConnection, .ReceiveMessage = IPNetwork_ReceiveMessage, .SendMessage = IPNetwork_SendMessage, .AcceptClient = IPNetwork_AcceptClient, .WaitEvent = IPNetwork_WaitEvent, .IsDataAvailable = IPNetwork_IsDataAvailable, };


#endif /* IP_NETWORK_H */
