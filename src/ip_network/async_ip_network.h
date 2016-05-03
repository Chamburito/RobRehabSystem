////////////////////////////////////////////////////////////////////////////////////////
///// Library that combines threading utilities with the socket connection library /////
///// to provide asyncronous network communication methods                         /////
////////////////////////////////////////////////////////////////////////////////////////

#ifndef ASYNC_IP_CONNECTION_H
#define ASYNC_IP_CONNECTION_H

#include <stdbool.h>

#include "ip_network.h"
#include "debug/async_debug.h"

#include "threads/thread_safe_data.h"

  
///////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                      DATA STRUCTURES                                            /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
  
const size_t QUEUE_MAX_ITEMS = 10;
  
// Structure that stores read and write message queues for a IPConnection struct used asyncronously
typedef struct _AsyncIPConnectionData
{
  IPConnection baseConnection;
  ThreadSafeQueue readQueue;
  ThreadSafeQueue writeQueue;
  char address[ IP_ADDRESS_LENGTH ];
}
AsyncIPConnectionData;

typedef AsyncIPConnectionData* AsyncIPConnection;

// Thread for asyncronous connections update
Thread globalReadThread = NULL;
Thread globalWriteThread = NULL;
bool isNetworkRunning = false;

// Internal (private) list of asyncronous connections created (accessible only by index)
static ThreadSafeMap globalConnectionsList = NULL;


///////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                            INTERFACE                                            /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

#define ASYNC_IP_NETWORK_FUNCTIONS( namespace, function_init ) \
        function_init( char*, namespace, GetAddress, int ) \
        function_init( size_t, namespace, GetActivesNumber, void ) \
        function_init( size_t, namespace, GetClientsNumber, int ) \
        function_init( size_t, namespace, SetMessageLength, int, size_t ) \
        function_init( int, namespace, OpenConnection, const char*, const char*, uint8_t ) \
        function_init( void, namespace, CloseConnection, int ) \
        function_init( char*, namespace, ReadMessage, int ) \
        function_init( int, namespace, WriteMessage, int, const char* ) \
        function_init( int, namespace, GetClient, int )

INIT_NAMESPACE_INTERFACE( AsyncIPNetwork, ASYNC_IP_NETWORK_FUNCTIONS )

///////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                      INFORMATION UTILITIES                                      /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

// Returns the number of asyncronous connections created (method for encapsulation purposes)
size_t AsyncIPNetwork_GetActivesNumber()
{
  static size_t activeConnectionsNumber = 0;
  
  size_t globalConnectionsListSize = ThreadSafeMaps.GetItemsCount( globalConnectionsList );
  for( size_t connectionIndex = 0; connectionIndex < globalConnectionsListSize; connectionIndex++ )
  {
    if( ThreadSafeMaps.GetItem( globalConnectionsList, connectionIndex, NULL ) ) activeConnectionsNumber++;
  }
  
  return activeConnectionsNumber;
}

// Returns number of clients for the server connection of given index
size_t AsyncIPNetwork_GetClientsNumber( int serverIndex )
{
  AsyncIPConnection connection = (AsyncIPConnection) ThreadSafeMaps.AquireItem( globalConnectionsList, serverIndex );
  if( connection == NULL ) return 0;
  
  if( connection->baseConnection->networkRole != SERVER )
  {
    /*ERROR_EVENT*/ERROR_PRINT( "not a server connection index: %d", serverIndex );
    return 0;
  }
  
  size_t clientsNumber = IPNetwork_GetClientsNumber( connection->baseConnection );
  
  ThreadSafeMaps.ReleaseItem( globalConnectionsList );

  return clientsNumber;
}

// Returns address string (host and port) for the connection of given index
inline char* AsyncIPNetwork_GetAddress( int connectionIndex )
{
  AsyncIPConnection connection = (AsyncIPConnection) ThreadSafeMaps.AquireItem( globalConnectionsList, connectionIndex );
  if( connection == NULL ) return 0;
  
  ThreadSafeMaps.ReleaseItem( globalConnectionsList );
  
  return connection->address;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                       INITIALIZATION                                            /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

// Forward definition
static void* AsyncReadQueues( void* );
static void* AsyncWriteQueues( void* );

// Create new AsyncIPConnection structure (from a given IPConnection structure) and add it to the internal list
static int AddAsyncConnection( IPConnection baseConnection )
{
  static AsyncIPConnection connection;
  static char* addressString;
  
  if( globalConnectionsList == NULL ) 
  {
    globalConnectionsList = ThreadSafeMaps.Create( TSMAP_INT, sizeof(AsyncIPConnectionData) );
    globalReadThread = Threading.StartThread( AsyncReadQueues, (void*) globalConnectionsList, THREAD_JOINABLE );
    globalWriteThread = Threading.StartThread( AsyncWriteQueues, (void*) globalConnectionsList, THREAD_JOINABLE );
  }
  
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "socket index: %d", baseConnection->socket->fd );
  
  int connectionID = (int) ThreadSafeMaps.SetItem( globalConnectionsList, baseConnection, NULL );  
  connection = ThreadSafeMaps.AquireItem( globalConnectionsList, connectionID );
  
  DEBUG_PRINT( "connection handle %p aquired", connection );
  
  connection->baseConnection = baseConnection;
  
  addressString = IPNetwork_GetAddress( baseConnection );
  strcpy( &(connection->address[ IP_HOST ]), &addressString[ IP_HOST ] );
  strcpy( &(connection->address[ IP_PORT ]), &addressString[ IP_PORT ] );
  
  size_t readQueueItemSize = ( baseConnection->networkRole == CLIENT ) ? IP_MAX_MESSAGE_LENGTH : sizeof(int);
  connection->readQueue = ThreadSafeQueues.Create( QUEUE_MAX_ITEMS, readQueueItemSize );  
  connection->writeQueue = ThreadSafeQueues.Create( QUEUE_MAX_ITEMS, IP_MAX_MESSAGE_LENGTH );
  
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "last connection index: %lu - socket fd: %d", ThreadSafeMaps.GetItemsCount( globalConnectionsList ), baseConnection->socket->fd );
  
  ThreadSafeMaps.ReleaseItem( globalConnectionsList );
  
  DEBUG_PRINT( "connection handle %p released", connection );
  
  return connectionID;
}

// Creates a new IPConnection structure (from the defined properties) and add it to the asyncronous connection list
int AsyncIPNetwork_OpenConnection( const char* host, const char* port, uint8_t protocol )
{
  DEBUG_PRINT( "Trying to create %s %s connection on port %s", ( protocol == TCP ) ? "TCP" : "UDP", ( host == NULL ) ? "Server" : "Client", port );
  IPConnection baseConnection = IPNetwork_OpenConnection( host, port, protocol );
  if( baseConnection == NULL )
  {
    /*ERROR_EVENT*/ERROR_PRINT( "failed to create %s %s connection on port %s", ( protocol == TCP ) ? "TCP" : "UDP", ( host == NULL ) ? "Server" : "Client", port );
    return -1;
  } 
  
  return AddAsyncConnection( baseConnection );
}

size_t AsyncIPNetwork_SetMessageLength( int connectionID, size_t messageLength )
{
  AsyncIPConnection connection = ThreadSafeMaps.AquireItem( globalConnectionsList, connectionID );
  if( connection == NULL ) return 0;
  
  messageLength = IPNetwork_SetMessageLength( connection->baseConnection, messageLength );
  
  ThreadSafeMaps.ReleaseItem( globalConnectionsList );
  
  return messageLength;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                     ASYNCRONOUS UPDATE                                          /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

static void ReadToQueue( void* ref_connection )
{
  AsyncIPConnection connection = (AsyncIPConnection) ref_connection;
  
  // Do not proceed if queue is full
  if( ThreadSafeQueues.GetItemsCount( connection->readQueue ) >= QUEUE_MAX_ITEMS )
  {
    //DEBUG_UPDATE( "connection socket %d read cache full\n", server->baseConnection->socket->fd );
    return;
  }
  
  if( IPNetwork_IsDataAvailable( connection->baseConnection ) )
  {
    if( connection->baseConnection->networkRole == SERVER )
    {
      IPConnection newClient = IPNetwork_AcceptClient( connection->baseConnection );
      if( newClient != NULL )
      {
        if( newClient->socket->fd != 0 )
        {
          DEBUG_PRINT( "client accepted: socket: %d - type: %x\n\thost: %s - port: %s", newClient->socket->fd, 
                                                                                        (newClient->protocol | newClient->networkRole),
                                                                                        &( IPNetwork_GetAddress( newClient ) )[ IP_HOST ],
                                                                                        &( IPNetwork_GetAddress( newClient ) )[ IP_PORT ] );
          ThreadSafeQueues.Enqueue( connection->readQueue, newClient, TSQUEUE_WAIT );
          AddAsyncConnection( newClient );
        }
      }
    }
    else
    {
      char* lastMessage = IPNetwork_ReceiveMessage( connection->baseConnection );
      if( lastMessage != NULL )
      {
        if( lastMessage[ 0 ] != '\0' )
        {
          //DEBUG_UPDATE( "connection socket %d received message: %s", reader->socket->fd, reader->buffer );
          ThreadSafeQueues.Enqueue( connection->readQueue, (void*) lastMessage, TSQUEUE_WAIT );
        }
      }
    }
  }
}

// Loop of message reading (storing in queue) to be called asyncronously for client/server connections
static void* AsyncReadQueues( void* args )
{
  isNetworkRunning = true;
  
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "reading client/message queues on thread %lx", THREAD_ID );
  
  while( isNetworkRunning )
  {    
    // Blocking call
    if( IPNetwork_WaitEvent( 5000 ) > 0 ) 
      ThreadSafeMaps.RunForAll( globalConnectionsList, ReadToQueue );
  }
  
  return NULL;
}

static void WriteFromQueue( void* ref_connection )
{
  AsyncIPConnection connection = (AsyncIPConnection) ref_connection;
  
  char firstMessage[ IP_MAX_MESSAGE_LENGTH ];
  
  // Do not proceed if queue is empty
  if( ThreadSafeQueues.GetItemsCount( connection->writeQueue ) == 0 )
  {
    //DEBUG_UPDATE( "connection socket %d write cache empty", writer->baseConnection->socket->fd );
    return;
  }
  
  ThreadSafeQueues.Dequeue( connection->writeQueue, (void*) firstMessage, TSQUEUE_WAIT );
  
  //DEBUG_UPDATE( "connection socket %d sending message: %s", writer->baseConnection->socket->fd, firstMessage );
  
  if( IPNetwork_SendMessage( connection->baseConnection, firstMessage ) == -1 )
    return;
}

// Loop of message writing (removing in order from queue) to be called asyncronously for client connections
static void* AsyncWriteQueues( void* args )
{
  isNetworkRunning = true;

  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "sending messages on thread %lx", THREAD_ID );
  
  while( isNetworkRunning )
    ThreadSafeMaps.RunForAll( globalConnectionsList, WriteFromQueue );
  
  return NULL;//(void*) 1;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                      SYNCRONOUS UPDATE                                          /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

// Get (and remove) message from the beginning (oldest) of the given index corresponding read queue
// Method to be called from the main thread
char* AsyncIPNetwork_ReadMessage( int clientID )
{
  static char firstMessage[ IP_MAX_MESSAGE_LENGTH ];

  AsyncIPConnection client = ThreadSafeMaps.AquireItem( globalConnectionsList, clientID );
  if( client != NULL )
  {
    if( client->baseConnection->networkRole == CLIENT )
    {
      if( ThreadSafeQueues.GetItemsCount( client->readQueue ) > 0 )
      {
        //DEBUG_UPDATE( "message from connection index %d: %s", clientID, firstMessage );
        ThreadSafeQueues.Dequeue( client->readQueue, firstMessage, TSQUEUE_WAIT );
      }
      else
      {
        //DEBUG_UPDATE( "no messages available for connection index %d", clientID );
        return NULL;
      }
    }
    else
    {
      /*ERROR_EVENT*/ERROR_PRINT( "connection index %d is not of a client connection", clientID );
      return NULL;
    }

    ThreadSafeMaps.ReleaseItem( globalConnectionsList );  
  }
  else
    return NULL;
  
  return firstMessage;
}

int AsyncIPNetwork_WriteMessage( int connectionID, const char* message )
{
  AsyncIPConnection connection = ThreadSafeMaps.AquireItem( globalConnectionsList, connectionID );
  if( connection == NULL ) return -1;
  
  if( ThreadSafeQueues.GetItemsCount( connection->writeQueue ) >= QUEUE_MAX_ITEMS )
    /*DEBUG_UPDATE*/DEBUG_PRINT( "connection index %d write queue is full", connectionID );
  
  ThreadSafeQueues.Enqueue( connection->writeQueue, (void*) message, TSQUEUE_NOWAIT );
  
  ThreadSafeMaps.ReleaseItem( globalConnectionsList );
  
  return 0;
}

int AsyncIPNetwork_GetClient( int serverID )
{
  int firstClient = IP_CONNECTION_INVALID_ID;

  AsyncIPConnection server = ThreadSafeMaps.AquireItem( globalConnectionsList, serverID );
  if( server != NULL )
  {
    if( server->baseConnection->networkRole == SERVER )
    {
      if( ThreadSafeQueues.GetItemsCount( server->readQueue ) > 0 )
      {
        ThreadSafeQueues.Dequeue( server->readQueue, &firstClient, TSQUEUE_WAIT );
    
        /*DEBUG_UPDATE*/DEBUG_PRINT( "new client index from connection index %d: %d", serverID, firstClient ); 
      }
    }
    //else
    //  /*ERROR_EVENT*/ERROR_PRINT( "connection index %d is not a server index", serverID );
    
    ThreadSafeMaps.ReleaseItem( globalConnectionsList );
  }
  
  return firstClient; 
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                           ENDING                                                /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

// Handle socket closing and structures destruction for the given index corresponding connection
void AsyncIPNetwork_CloseConnection( int connectionID )
{
  AsyncIPConnection connection = ThreadSafeMaps.AquireItem( globalConnectionsList, connectionID );
  if( connection == NULL ) return;
  
  IPNetwork_CloseConnection( connection->baseConnection );
  connection->baseConnection = NULL;
  
  ThreadSafeQueues.Discard( connection->readQueue );
  ThreadSafeQueues.Discard( connection->writeQueue );
  
  ThreadSafeMaps.ReleaseItem( globalConnectionsList );
  
  ThreadSafeMaps.RemoveItem( globalConnectionsList, connectionID );
  
  if( ThreadSafeMaps.GetItemsCount( globalConnectionsList ) == 0 )
  {
    isNetworkRunning = false;
    /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "waiting update threads %p and %p for exit", globalReadThread, globalWriteThread );
    (void) Threading.WaitExit( globalReadThread, 5000 );
    /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "read thread for connection id %u returned", connectionID );     
    (void) Threading.WaitExit( globalWriteThread, 5000 );
    /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "write thread for connection id %u returned", connectionID ); 
    
    ThreadSafeMaps.Discard( globalConnectionsList );
    globalConnectionsList = NULL;
  }
  
  return;
}

#endif /* ASYNC_IP_CONNECTION_H */
