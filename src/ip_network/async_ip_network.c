////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  This file is part of RobRehabSystem.                                      //
//                                                                            //
//  RobRehabSystem is free software: you can redistribute it and/or modify    //
//  it under the terms of the GNU Lesser General Public License as published  //
//  by the Free Software Foundation, either version 3 of the License, or      //
//  (at your option) any later version.                                       //
//                                                                            //
//  RobRehabSystem is distributed in the hope that it will be useful,         //
//  but WITHOUT ANY WARRANTY; without even the implied warranty of            //
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the              //
//  GNU Lesser General Public License for more details.                       //
//                                                                            //
//  You should have received a copy of the GNU Lesser General Public License  //
//  along with Foobar. If not, see <http://www.gnu.org/licenses/>.            //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////


#include <stdbool.h>

#include "ip_network/async_ip_network.h"

#include "debug/async_debug.h"

#include "time/timing.h"
#include "threads/thread_safe_data.h"

  
///////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                      DATA STRUCTURES                                            /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
  
const size_t QUEUE_MAX_ITEMS = 10;
  
// Structure that stores read and write message queues for a IPConnection struct used asyncronously
struct _AsyncIPConnectionData
{
  IPConnection baseConnection;
  ThreadSafeQueue readQueue;
  ThreadSafeQueue writeQueue;
};

// Thread for asyncronous connections update
static Thread globalReadThread = THREAD_INVALID_HANDLE;
static Thread globalWriteThread = THREAD_INVALID_HANDLE;
static bool isNetworkRunning = false;

// Internal (private) list of asyncronous connections created (accessible only by index)
static ThreadSafeMap globalConnectionsList = NULL;

DEFINE_NAMESPACE_INTERFACE( AsyncIPNetwork, ASYNC_IP_NETWORK_INTERFACE )


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
  
  size_t clientsNumber = IPNetwork.GetClientsNumber( connection->baseConnection );
  
  ThreadSafeMaps.ReleaseItem( globalConnectionsList );

  return clientsNumber;
}

// Returns address string (host and port) for the connection of given index
char* AsyncIPNetwork_GetAddress( int connectionIndex )
{
  AsyncIPConnection connection = (AsyncIPConnection) ThreadSafeMaps.AquireItem( globalConnectionsList, connectionIndex );
  if( connection == NULL ) return 0;
  
  ThreadSafeMaps.ReleaseItem( globalConnectionsList );
  
  return IPNetwork.GetAddress( connection->baseConnection );
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
  if( globalConnectionsList == NULL ) 
  {
    globalConnectionsList = ThreadSafeMaps.Create( TSMAP_INT, sizeof(AsyncIPConnectionData) );
    globalReadThread = Threading.StartThread( AsyncReadQueues, (void*) globalConnectionsList, THREAD_JOINABLE );
    globalWriteThread = Threading.StartThread( AsyncWriteQueues, (void*) globalConnectionsList, THREAD_JOINABLE );
  }
  
  ///*DEBUG_EVENT( 0,*/DEBUG_PRINT( "socket index: %d", baseConnection->socket->fd );
  
  AsyncIPConnectionData connectionData = { .baseConnection = baseConnection };
  
  size_t readQueueItemSize = ( !IPNetwork.IsServer( baseConnection ) ) ? IP_MAX_MESSAGE_LENGTH : sizeof(int);
  connectionData.readQueue = ThreadSafeQueues.Create( QUEUE_MAX_ITEMS, readQueueItemSize );  
  connectionData.writeQueue = ThreadSafeQueues.Create( QUEUE_MAX_ITEMS, IP_MAX_MESSAGE_LENGTH );
  
  int connectionID = (int) ThreadSafeMaps.SetItem( globalConnectionsList, baseConnection, &connectionData );  
  
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "last connection index: %lu - connection: %p", ThreadSafeMaps.GetItemsCount( globalConnectionsList ), baseConnection );
  
  return connectionID;
}

// Creates a new IPConnection structure (from the defined properties) and add it to the asyncronous connection list
int AsyncIPNetwork_OpenConnection( uint8_t connectionType, const char* host, uint16_t port )
{
  DEBUG_PRINT( "Trying to create connection type %x on host %s and port %u", connectionType, ( host == NULL ) ? "(ANY)" : host, port );
  IPConnection baseConnection = IPNetwork.OpenConnection( connectionType, host, port );
  if( baseConnection == NULL )
  {
    /*ERROR_EVENT*/ERROR_PRINT( "failed to create connection type %x on host %s and port %u", connectionType, ( host == NULL ) ? "(ANY)" : host, port );
    return -1;
  } 
  
  return AddAsyncConnection( baseConnection );
}

size_t AsyncIPNetwork_SetMessageLength( int connectionID, size_t messageLength )
{
  AsyncIPConnection connection = ThreadSafeMaps.AquireItem( globalConnectionsList, connectionID );
  if( connection == NULL ) return 0;
  
  messageLength = IPNetwork.SetMessageLength( connection->baseConnection, messageLength );
  
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
  
  if( IPNetwork.IsDataAvailable( connection->baseConnection ) )
  {
    if( IPNetwork.IsServer( connection->baseConnection ) )
    {
      IPConnection newClient = IPNetwork.AcceptClient( connection->baseConnection );
      if( newClient != NULL )
      {
        char* addressString = IPNetwork.GetAddress( newClient );
        if( addressString != NULL )
        {
          DEBUG_PRINT( "client accepted: connection: %p - address: %s", newClient, addressString );
          ThreadSafeQueues.Enqueue( connection->readQueue, newClient, TSQUEUE_WAIT );
          AddAsyncConnection( newClient );
        }
      }
    }
    else
    {
      char* lastMessage = IPNetwork.ReceiveMessage( connection->baseConnection );
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
    if( IPNetwork.WaitEvent( 5000 ) > 0 ) 
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
  
  if( IPNetwork.SendMessage( connection->baseConnection, firstMessage ) == -1 )
    return;
}

// Loop of message writing (removing in order from queue) to be called asyncronously for client connections
static void* AsyncWriteQueues( void* args )
{
  isNetworkRunning = true;

  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "sending messages on thread %lx", THREAD_ID );
  
  while( isNetworkRunning )
  {
    ThreadSafeMaps.RunForAll( globalConnectionsList, WriteFromQueue );
    
    Timing.Delay( 1 );
  }
  
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
    if( !IPNetwork.IsServer( client->baseConnection ) )
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
    if( IPNetwork.IsServer( server->baseConnection ) )
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
  
  IPNetwork.CloseConnection( connection->baseConnection );
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
