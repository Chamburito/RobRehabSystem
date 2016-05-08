/////////////////////////////////////////////////////////////////////////////////////
///// Multiplatform library for creation and handling of IP sockets connections /////
///// as server or client, using TCP or UDP protocols                           /////
/////////////////////////////////////////////////////////////////////////////////////

#ifndef IP_NETWORK_H
#define IP_NETWORK_H

#include <stdbool.h>
#include <stdint.h>

#include "namespaces.h"
  
#include "debug/sync_debug.h"

/////////////////////////////////////////////////////////////////////////  
/////                    PREPROCESSOR DIRECTIVES                    /////
/////////////////////////////////////////////////////////////////////////
  
#define IP_MAX_MESSAGE_LENGTH 512

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

#define IP_NETWORK_INTERFACE( Namespace, INIT_FUNCTION ) \
        INIT_FUNCTION( char*, Namespace, GetAddress, IPConnection ) \
        INIT_FUNCTION( size_t, Namespace, GetClientsNumber, IPConnection ) \
        INIT_FUNCTION( size_t, Namespace, SetMessageLength, IPConnection, size_t ) \
        INIT_FUNCTION( bool, Namespace, IsServer, IPConnection ) \
        INIT_FUNCTION( IPConnection, Namespace, OpenConnection, uint8_t, const char*, uint16_t ) \
        INIT_FUNCTION( void, Namespace, CloseConnection, IPConnection ) \
        INIT_FUNCTION( char*, Namespace, ReceiveMessage, IPConnection ) \
        INIT_FUNCTION( int, Namespace, SendMessage, IPConnection, const char* ) \
        INIT_FUNCTION( IPConnection, Namespace, AcceptClient, IPConnection ) \
        INIT_FUNCTION( int, Namespace, WaitEvent, unsigned int ) \
        INIT_FUNCTION( bool, Namespace, IsDataAvailable, IPConnection )

DECLARE_NAMESPACE_INTERFACE( IPNetwork, IP_NETWORK_INTERFACE )


#endif /* IP_NETWORK_H */
