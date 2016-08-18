////////////////////////////////////////////////////////////////////////////////////////
///// Library that combines threading utilities with the socket connection library /////
///// to provide asyncronous network communication methods                         /////
////////////////////////////////////////////////////////////////////////////////////////

#ifndef ASYNC_IP_CONNECTION_H
#define ASYNC_IP_CONNECTION_H

#include "namespaces.h"

#include "ip_network/ip_network.h"

#define IP_CONNECTION_INVALID_ID -1
  
///////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                      DATA STRUCTURES                                            /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
  
// Structure that stores read and write message queues for a IPConnection struct used asyncronously
typedef struct _AsyncIPConnectionData AsyncIPConnectionData;
typedef AsyncIPConnectionData* AsyncIPConnection;


///////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                            INTERFACE                                            /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

#define ASYNC_IP_NETWORK_INTERFACE( Namespace, INIT_FUNCTION ) \
        INIT_FUNCTION( char*, Namespace, GetAddress, unsigned long ) \
        INIT_FUNCTION( size_t, Namespace, GetActivesNumber, void ) \
        INIT_FUNCTION( size_t, Namespace, GetClientsNumber, unsigned long ) \
        INIT_FUNCTION( size_t, Namespace, SetMessageLength, unsigned long, size_t ) \
        INIT_FUNCTION( unsigned long, Namespace, OpenConnection, uint8_t, const char*, uint16_t ) \
        INIT_FUNCTION( void, Namespace, CloseConnection, unsigned long ) \
        INIT_FUNCTION( char*, Namespace, ReadMessage, unsigned long ) \
        INIT_FUNCTION( bool, Namespace, WriteMessage, unsigned long, const char* ) \
        INIT_FUNCTION( unsigned long, Namespace, GetClient, unsigned long )

DECLARE_NAMESPACE_INTERFACE( AsyncIPNetwork, ASYNC_IP_NETWORK_INTERFACE )


#endif /* ASYNC_IP_CONNECTION_H */
