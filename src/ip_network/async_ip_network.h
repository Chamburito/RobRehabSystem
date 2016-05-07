////////////////////////////////////////////////////////////////////////////////////////
///// Library that combines threading utilities with the socket connection library /////
///// to provide asyncronous network communication methods                         /////
////////////////////////////////////////////////////////////////////////////////////////

#ifndef ASYNC_IP_CONNECTION_H
#define ASYNC_IP_CONNECTION_H

#include "interfaces.h"

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

#define ASYNC_IP_NETWORK_INTERFACE( namespace, function_init ) \
        function_init( char*, namespace, GetAddress, int ) \
        function_init( size_t, namespace, GetActivesNumber, void ) \
        function_init( size_t, namespace, GetClientsNumber, int ) \
        function_init( size_t, namespace, SetMessageLength, int, size_t ) \
        function_init( int, namespace, OpenConnection, uint8_t, const char*, uint16_t ) \
        function_init( void, namespace, CloseConnection, int ) \
        function_init( char*, namespace, ReadMessage, int ) \
        function_init( int, namespace, WriteMessage, int, const char* ) \
        function_init( int, namespace, GetClient, int )

DECLARE_NAMESPACE_INTERFACE( AsyncIPNetwork, ASYNC_IP_NETWORK_INTERFACE )


#endif /* ASYNC_IP_CONNECTION_H */
