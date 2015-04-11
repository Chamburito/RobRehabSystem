////////////////////////////////////////////////////////////////////////////////////////
///// Library that combines threading utilities with the socket connection library /////
///// to provide asyncronous network communication methods                         /////
////////////////////////////////////////////////////////////////////////////////////////

#ifndef ASYNC_IP_CONNECTION_H
#define ASYNC_IP_CONNECTION_H

#ifdef __cplusplus
extern "C"{
#endif

#include "ip_connection.h"
#include "debug.h"  
#include "threads_data_structures.h"

  
///////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                      DATA STRUCTURES                                            /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
  
const size_t QUEUE_MAX_ITEMS = 10;
  
// Structure that stores read and write message queues for a IPConnection struct used asyncronously
typedef struct _AsyncIPConnection
{
  IPConnection* baseConnection;
  DataQueue* readQueue, writeQueue;
  Thread_Handle readThread, writeThread;
  char address[ IP_ADDRESS_LENGTH ];
}
AsyncIPConnection;

// Internal (private) list of asyncronous connections created (accessible only by index)
static AsyncIPConnection** globalConnectionsList = NULL;
static size_t globalConnectionsListSize = 0;

///////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                      INFORMATION UTILITIES                                      /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

// Verifies if connection index is valid or not already closed
static INLINE bool IsValidConnectionIndex( int connectionIndex )
{
  if( connectionIndex < 0 || connectionIndex >= (int) globalConnectionsListSize )
  {
    ERROR_EVENT( "invalid connection index: %d", connectionIndex );
    return false;
  }
  
  if( globalConnectionsList[ connectionIndex ] == NULL )
  {
    ERROR_EVENT( "connection index %d already closed", connectionIndex );
    return false;
  }
  
  return true;
}

// Returns the number of asyncronous connections created (method for encapsulation purposes)
size_t AsyncIPConnection_GetActivesNumber()
{
  static size_t activeConnectionsNumber = 0;
  
  for( size_t connectionIndex = 0; connectionIndex < globalConnectionsListSize; connectionIndex++ )
  {
    if( globalConnectionsList[ connectionIndex ] != NULL ) activeConnectionsNumber++;
  }
  
  return activeConnectionsNumber;
}

// Returns number of clients for the server connection of given index
size_t AsyncIPConnection_GetClientsNumber( int serverIndex )
{
  if( !IsValidConnectionIndex( serverIndex ) ) return 0;
  
  if( globalConnectionsList[ serverIndex ]->baseConnection->networkRole != SERVER )
  {
    ERROR_EVENT( "not a server connection index: %d", serverIndex );
    return 0;
  }

  return IPConnection_GetClientsNumber( globalConnectionsList[ serverIndex ]->baseConnection );
}

// Returns address string (host and port) for the connection of given index
extern INLINE char* AsyncIPConnection_GetAddress( int connectionIndex )
{
  if( !IsValidConnectionIndex( connectionIndex ) ) return NULL;
  
  return globalConnectionsList[ connectionIndex ]->address;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                       INITIALIZATION                                            /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

// Forward definition
static void* AsyncReadQueue( void* );
static void* AsyncWriteQueue( void* );
static void* AsyncAcceptClients( void* );

// Create new AsyncIPConnection structure (from a given IPConnection structure) and add it to the internal list
static int AddAsyncConnection( IPConnection* baseConnection )
{
  static AsyncIPConnection* connection;
  static char* addressString;
  
  DEBUG_EVENT( 0, "socket index: %d",  baseConnection->socketFD );
  
  globalConnectionsList = (AsyncIPConnection**) realloc( globalConnectionsList, ( globalConnectionsListSize + 1 ) * sizeof(AsyncIPConnection*) );
  globalConnectionsList[ globalConnectionsListSize ] = (AsyncIPConnection*) malloc( sizeof(AsyncIPConnection) );
  
  connection = globalConnectionsList[ globalConnectionsListSize ];
  
  connection->baseConnection = baseConnection;
  
  addressString = AsyncIPConnection_GetAddress( baseConnection );
  strcpy( &(connection->address[ IP_HOST ]), &addressString[ IP_HOST ] );
  strcpy( &(connection->address[ IP_PORT ]), &addressString[ IP_PORT ] );
  
  if( baseConnection->networkRole == CLIENT )
  {
    connection->readQueue = DataQueue_Init( QUEUE_MAX_ITEMS, IP_CONNECTION_MSG_LEN );  
    connection->readThread = Thread_Start( AsyncReadQueue, (void*) connection, JOINABLE );
  }
  else if( baseConnection->networkRole == SERVER )
  {
    connection->readQueue = DataQueue_Init( QUEUE_MAX_ITEMS, sizeof(int) );
    connection->readThread = Thread_Start( AsyncAcceptClients, (void*) connection, JOINABLE );
  }

  connection->writeQueue = DataQueue_Init( QUEUE_MAX_ITEMS, IP_CONNECTION_MSG_LEN );
  connection->writeThread = Thread_Start( AsyncWriteQueue, (void*) connection, JOINABLE );
  
  DEBUG_EVENT( 0, "last connection index: %d - socket fd: %d", globalConnectionsListSize,  baseConnection->socketFD );
  
  return globalConnectionsListSize++;
}

// Creates a new IPConnection structure (from the defined properties) and add it to the asyncronous connection list
int AsyncIPConnection_Open( const char* host, const char* port, int protocol )
{
  IPConnection* baseConnection = IPConnection_Open( host, port, protocol );
  if( baseConnection == NULL )
  {
    ERROR_EVENT( "failed to create %s %s connection on port %s", ( protocol == TCP ) ? "TCP" : "UDP", 
                                                                 ( host == NULL ) ? "Server" : "Client", port );
    return -1;
  } 
  
  return AddAsyncConnection( baseConnection );
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                     ASYNCRONOUS UPDATE                                          /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

// Loop of message reading (storing in queue) to be called asyncronously for client connections
static void* AsyncReadQueue( void* args )
{
  AsyncIPConnection* reader_buffer = (AsyncIPConnection*) args;
  IPConnection* reader = reader_buffer->baseConnection;
  DataQueue* readQueue = reader_buffer->readQueue;
  
  char* lastMessage;
  
  DEBUG_EVENT( 0, "connection socket %d caching available messages on thread %x", reader_buffer-> baseConnection->socketFD, THREAD_ID );
  
  while( reader_buffer != NULL )
  {
    DEBUG_UPDATE( "message list count: %u", DataQueue_ItemCount( readQueue ) );
    
    if( reader_buffer->baseConnection == NULL )
    {
      //DEBUG_EVENT( 0, "connection closed" );
      break;
    }
    
    if( DataQueue_ItemCount( readQueue ) >= QUEUE_MAX_ITEMS )
      DEBUG_UPDATE( "connection socket %d read cache full", reader_buffer-> baseConnection->socketFD );
    
    // Blocking call
    if( IPConnection_WaitEvent( reader, 2000 ) != 0 ) 
    {
      DEBUG_UPDATE( "message available on client socket %d", reader->socketFD );
      
      if( (lastMessage = IPConnection_ReceiveMessage( reader )) != NULL )
      {
        if( lastMessage[0] == '\0' ) continue;
        
        DEBUG_UPDATE( "connection socket %d received message: %s", reader->socketFD, reader->buffer );

        DataQueue_Push( readQueue, (void*) lastMessage, WAIT );
      }
      else
        break;
    }
  }
  
  Thread_Exit( 0 );
  return NULL;
}

// Loop of message writing (removing in order from queue) to be called asyncronously
static void* AsyncWriteQueue( void* args )
{
  AsyncIPConnection* writer = (AsyncIPConnection*) args;
  IPConnection* baseWriter = writer->baseConnection;
  DataQueue* writeQueue = writer->writeQueue;
  
  char firstMessage[ IP_CONNECTION_MSG_LEN ];

  DEBUG_EVENT( 0, "connection socket %d sending messages in cache on thread %x", writer-> baseConnection->socketFD, THREAD_ID );
  
  while( writer != NULL )
  {
    DEBUG_UPDATE( "connection socket %d message count: %u", baseWriter->socketFD, DataQueue_ItemCount( writeQueue ) );
    
    if( writer->baseConnection == NULL )
    {
      //DEBUG_EVENT( 0, "connection closed" );
      break;
    }
    
    // Do not proceed if queue is empty
    if( DataQueue_ItemCount( writeQueue ) <= 0 )
      DEBUG_UPDATE( "connection socket %d write cache empty", writer->baseConnection->socketFD );
    
    DataQueue_Pop( writeQueue, (void*) firstMessage );
    
    DEBUG_UPDATE( "connection socket %d sending message: %s", baseWriter->socketFD, firstMessage );
    
    if( IPConnection_SendMessage( baseWriter, firstMessage ) == -1 )
      break;
  }
  
  Thread_Exit( 1 );
  return NULL;
}

// Loop of client accepting (storing in client list) to be called asyncronously for server connections
// The message queue stores the index and address of the detected remote connection in string format
static void* AsyncAcceptClients( void* args )
{  
  AsyncIPConnection* server = (AsyncIPConnection*) args;
  IPConnection* baseServer = server->baseConnection;
  IPConnection* client;
  DataQueue* clientQueue = server->readQueue;

  int lastClient;
  
  DEBUG_EVENT( 0, "connection socket %d trying to add clients on thread %x\n", server-> baseConnection->socketFD, THREAD_ID );
  
  while( server != NULL ) 
  {
    DEBUG_UPDATE( "client list count: %u", DataQueue_ItemCount( clientQueue ) );
    
    if( server->baseConnection == NULL )
    {
      //DEBUG_EVENT( 0, "connection closed\n" ); 
      break;
    }
    
    // Do not proceed if queue is full
    if( DataQueue_ItemCount( clientQueue ) >= QUEUE_MAX_ITEMS )
    {
      DEBUG_UPDATE( "connection socket %d read cache full\n", server->baseConnection->socketFD );
      continue;
    }
    
    // Blocking call
    if( IPConnection_WaitEvent( baseServer, 5000 ) != 0 ) 
    {
      DEBUG_UPDATE( "client available on server socket %d",  baseServer->socketFD );
      
      if( (client = IPConnection_AcceptClient( baseServer )) != NULL )
      {
        if( client->socketFD == 0 ) continue;
        
        DEBUG_EVENT( 0, "client accepted: socket: %d - type: %x\n\thost: %s - port: %s", &( AsyncIPConnection_GetAddress( client ) )[ IP_HOST ], 
                     &( AsyncIPConnection_GetAddress( client ) )[ IP_PORT ], client->socketFD, (client->protocol | client->networkRole) );
        
        lastClient = globalConnectionsListSize;
        
        DataQueue_Push( clientQueue, &lastClient, WAIT );
        
        DEBUG_UPDATE( "clients number before: %d\n", globalConnectionsListSize );
        
        AddAsyncConnection( client );
        
        DEBUG_UPDATE( "clients number after: %d\n", globalConnectionsListSize );
      }
      else
        break;
    }
  }
  
  Thread_Exit( 2 );
  return NULL;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                      SYNCRONOUS UPDATE                                          /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

// Get (and remove) message from the beginning (oldest) of the given index corresponding read queue
// Method to be called from the main thread
char* AsyncIPConnection_ReadMessage( int clientIndex )
{
  static DataQueue* readQueue;
  static char firstMessage[ IP_CONNECTION_MSG_LEN ];

  if( !IsValidConnectionIndex( clientIndex ) ) return NULL;
  
  if( globalConnectionsList[ clientIndex ]->baseConnection->networkRole != CLIENT )
  {
    ERROR_EVENT( "connection index %d is not of a client connection", clientIndex );
    return NULL;
  }
  
  readQueue = globalConnectionsList[ clientIndex ]->readQueue;
  
  if( DataQueue_ItemCount( readQueue ) <= 0 )
  {
    DEBUG_UPDATE( "no messages available for connection index %d", clientIndex );
    return NULL;
  }
  
  DataQueue_Pop( readQueue, firstMessage );
  
  DEBUG_UPDATE( "message from connection index %d: %s", clientIndex, firstMessage );  
  
  return firstMessage;
}

int AsyncIPConnection_WriteMessage( int connectionIndex, const char* message )
{
  static DataQueue* writeQueue;
  static char* last_message;

  if( !IsValidConnectionIndex( connectionIndex ) ) return -1;
  
  writeQueue = globalConnectionsList[ connectionIndex ]->writeQueue;
  
  if( DataQueue_ItemCount( writeQueue ) >= QUEUE_MAX_ITEMS )
    DEBUG_UPDATE( "connection index %d write queue is full", connectionIndex );
  
  DataQueue_Push( writeQueue, message, REPLACE );
  
  return 0;
}

int AsyncIPConnection_GetClient( int serverIndex )
{
  static DataQueue* clientQueue;
  static int firstClient;

  if( !IsValidConnectionIndex( serverIndex ) ) return -1;
  
  if( globalConnectionsList[ serverIndex ]->baseConnection->networkRole != SERVER )
  {
    ERROR_EVENT( "connection index %d is not a server index", serverIndex );
    return -1;
  }
  
  clientQueue = globalConnectionsList[ serverIndex ]->readQueue;
  
  if( DataQueue_ItemCount( clientQueue ) <= 0 )
  {
    DEBUG_UPDATE( "no new clients available for connection index %d", serverIndex );
    return -1;
  }
  
  DataQueue_Pop( clientQueue, &firstClient );
  
  DEBUG_UPDATE( "new client index from connection index %d: %d", serverIndex, firstClient );  
  
  return firstClient; 
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                           ENDING                                                /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

// Handle socket closing and structures destruction for the given index corresponding connection
void AsyncIPConnection_Close( int connectionIndex )
{
  if( !IsValidConnectionIndex( connectionIndex ) ) return;
  
  IPConnection_Close( globalConnectionsList[ connectionIndex ]->baseConnection );
  globalConnectionsList[ connectionIndex ]->baseConnection = NULL;
  
  DEBUG_EVENT( 0, "waiting threads for connection id %u", connectionIndex );
  
  (void) Thread_WaitExit( globalConnectionsList[ connectionIndex ]->readThread, 5000 );
  
  DEBUG_EVENT( 0, "read thread for connection id %u returned", connectionIndex );     
  
  (void) Thread_WaitExit( globalConnectionsList[ connectionIndex ]->writeThread, 5000 );
  
  DEBUG_EVENT( 0, "write thread for connection id %u returned", connectionIndex ); 
  
  DataQueue_End( globalConnectionsList[ connectionIndex ]->readQueue );
  DataQueue_End( globalConnectionsList[ connectionIndex ]->writeQueue );
  
  free( globalConnectionsList[ connectionIndex ] );
  globalConnectionsList[ connectionIndex ] = NULL;
  
  if( AsyncIPConnection_GetActivesNumber() <= 0 )
  {
    globalConnectionsListSize = 0;
    free( globalConnectionsList );
    globalConnectionsList = NULL;
  }
  
  return;
}

#ifdef __cplusplus
}
#endif

#endif /* ASYNC_IP_CONNECTION_H */
