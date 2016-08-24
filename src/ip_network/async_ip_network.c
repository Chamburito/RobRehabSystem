////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  Copyright (c) 2016 Leonardo José Consoni                                  //
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
//  along with RobRehabSystem. If not, see <http://www.gnu.org/licenses/>.    //
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
static volatile bool isNetworkRunning = false;

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
size_t AsyncIPNetwork_GetClientsNumber( unsigned long serverID )
{
  AsyncIPConnection connection = (AsyncIPConnection) ThreadSafeMaps.AquireItem( globalConnectionsList, serverID );
  if( connection == NULL ) return 0;
  
  size_t clientsNumber = IPNetwork.GetClientsNumber( connection->baseConnection );
  
  ThreadSafeMaps.ReleaseItem( globalConnectionsList, serverID );

  return clientsNumber;
}

// Returns address string (host and port) for the connection of given index
char* AsyncIPNetwork_GetAddress( unsigned long connectionID )
{
  AsyncIPConnection connection = (AsyncIPConnection) ThreadSafeMaps.AquireItem( globalConnectionsList, connectionID );
  if( connection == NULL ) return 0;
  
  ThreadSafeMaps.ReleaseItem( globalConnectionsList, connectionID );
  
  return IPNetwork.GetAddress( connection->baseConnection );
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                       INITIALIZATION                                            /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

// Forward definition
static void* AsyncReadQueues( void* );
static void* AsyncWriteQueues( void* );

// Create new AsyncIPConnection structure (from a given IPConnection structure) and add it to the internal list
static unsigned long AddAsyncConnection( IPConnection baseConnection )
{
  if( globalConnectionsList == NULL ) 
  {
    globalConnectionsList = ThreadSafeMaps.Create( TSMAP_INT, sizeof(AsyncIPConnectionData) );
    globalReadThread = Threading.StartThread( AsyncReadQueues, (void*) globalConnectionsList, THREAD_JOINABLE );
    globalWriteThread = Threading.StartThread( AsyncWriteQueues, (void*) globalConnectionsList, THREAD_JOINABLE );
  }
  
  ///*DEBUG_EVENT( 0,*/DEBUG_PRINT( "socket index: %d", baseConnection->socket->fd );
  
  AsyncIPConnectionData connectionData = { .baseConnection = baseConnection };
  
  size_t readQueueItemSize = ( !IPNetwork.IsServer( baseConnection ) ) ? IP_MAX_MESSAGE_LENGTH : sizeof(unsigned long);
  connectionData.readQueue = ThreadSafeQueues.Create( QUEUE_MAX_ITEMS, readQueueItemSize );  
  connectionData.writeQueue = ThreadSafeQueues.Create( QUEUE_MAX_ITEMS, IP_MAX_MESSAGE_LENGTH );
  
  unsigned long connectionID = ThreadSafeMaps.SetItem( globalConnectionsList, baseConnection, &connectionData );  
  
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "last connection: %p (ID %lu)", baseConnection, connectionID );
  
  return connectionID;
}

// Creates a new IPConnection structure (from the defined properties) and add it to the asyncronous connection list
unsigned long AsyncIPNetwork_OpenConnection( uint8_t connectionType, const char* host, uint16_t port )
{
  DEBUG_PRINT( "Trying to create connection type %x on host %s and port %u", connectionType, ( host == NULL ) ? "(ANY)" : host, port );
  IPConnection baseConnection = IPNetwork.OpenConnection( connectionType, host, port );
  if( baseConnection == NULL )
  {
    /*ERROR_EVENT*/ERROR_PRINT( "failed to create connection type %x on host %s and port %u", connectionType, ( host == NULL ) ? "(ANY)" : host, port );
    return (unsigned long) IP_CONNECTION_INVALID_ID;
  } 
  
  return AddAsyncConnection( baseConnection );
}

size_t AsyncIPNetwork_SetMessageLength( unsigned long connectionID, size_t messageLength )
{
  AsyncIPConnection connection = ThreadSafeMaps.AquireItem( globalConnectionsList, connectionID );
  if( connection == NULL ) return 0;
  
  messageLength = IPNetwork.SetMessageLength( connection->baseConnection, messageLength );
  
  ThreadSafeMaps.ReleaseItem( globalConnectionsList, connectionID );
  
  return messageLength;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                     ASYNCRONOUS UPDATE                                          /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

static void ReadToQueue( unsigned long connectionID )
{
  AsyncIPConnection connection = ThreadSafeMaps.AquireItem( globalConnectionsList, connectionID );
  if( connection == NULL ) return;
  
  // Do not proceed if queue is full
  if( ThreadSafeQueues.GetItemsCount( connection->readQueue ) >= QUEUE_MAX_ITEMS ) 
  {
    ThreadSafeMaps.ReleaseItem( globalConnectionsList, connectionID );
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
          /*DEBUG_UPDATE*/DEBUG_PRINT( "client accepted: server: %p - client: %p - address: %s", connection, newClient, addressString );
          ThreadSafeMaps.ReleaseItem( globalConnectionsList, connectionID );
          unsigned long newClientID = AddAsyncConnection( newClient );
          ThreadSafeQueues.Enqueue( connection->readQueue, &newClientID, TSQUEUE_WAIT );
          return;
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
          DEBUG_UPDATE( "message received: client %p received message: %s", connection, lastMessage );
          ThreadSafeQueues.Enqueue( connection->readQueue, (void*) lastMessage, TSQUEUE_WAIT );
        }
      }
    }
  }
  
  ThreadSafeMaps.ReleaseItem( globalConnectionsList, connectionID );
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
      ThreadSafeMaps.RunForAllKeys( globalConnectionsList, ReadToQueue );
  }
  
  return NULL;
}

static void WriteFromQueue( unsigned long connectionID )
{
  AsyncIPConnection connection = ThreadSafeMaps.AquireItem( globalConnectionsList, connectionID );
  if( connection == NULL ) return;
  
  char firstMessage[ IP_MAX_MESSAGE_LENGTH ];
  
  // Do not proceed if queue is empty
  if( ThreadSafeQueues.GetItemsCount( connection->writeQueue ) == 0 )
  {
    //DEBUG_UPDATE( "connection socket %d write cache empty", writer->baseConnection->socket->fd );
    ThreadSafeMaps.ReleaseItem( globalConnectionsList, connectionID );
    return;
  }
  
  ThreadSafeQueues.Dequeue( connection->writeQueue, (void*) firstMessage, TSQUEUE_WAIT );
  
  //DEBUG_UPDATE( "connection socket %d sending message: %s", writer->baseConnection->socket->fd, firstMessage );
  
  if( IPNetwork.SendMessage( connection->baseConnection, firstMessage ) == -1 )
  {
    ThreadSafeMaps.ReleaseItem( globalConnectionsList, connectionID );
    ThreadSafeMaps.RemoveItem( globalConnectionsList, connectionID );
    return;
  }
  
  ThreadSafeMaps.ReleaseItem( globalConnectionsList, connectionID );
}

// Loop of message writing (removing in order from queue) to be called asyncronously for client connections
static void* AsyncWriteQueues( void* args )
{
  isNetworkRunning = true;

  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "sending messages on thread %lx", THREAD_ID );
  
  while( isNetworkRunning )
  {
    ThreadSafeMaps.RunForAllKeys( globalConnectionsList, WriteFromQueue );
    
    Timing.Delay( 1 );
  }
  
  return NULL;//(void*) 1;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                      SYNCRONOUS UPDATE                                          /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

// Get (and remove) message from the beginning (oldest) of the given index corresponding read queue
// Method to be called from the main thread
char* AsyncIPNetwork_ReadMessage( unsigned long clientID )
{
  static char messageData[ IP_MAX_MESSAGE_LENGTH ];
  char* firstMessage = NULL;
  
  AsyncIPConnection client = ThreadSafeMaps.AquireItem( globalConnectionsList, clientID );
  if( client != NULL )
  {
    //DEBUG_PRINT( "is connection %p index %lu a client: %s", client, clientID, IPNetwork.IsServer( client->baseConnection ) ? "no" : "yes" );
    if( !IPNetwork.IsServer( client->baseConnection ) )
    {
      //DEBUG_PRINT( "messages available for connection %p index %lu: %lu", client, clientID, ThreadSafeQueues.GetItemsCount( client->readQueue ) );
      if( ThreadSafeQueues.GetItemsCount( client->readQueue ) > 0 )
      {
        ///*DEBUG_UPDATE*/DEBUG_PRINT( "message from connection index %lu: %s", clientID, firstMessage );
        firstMessage = (char*) &messageData;
        ThreadSafeQueues.Dequeue( client->readQueue, firstMessage, TSQUEUE_WAIT );
      }
    }
    else
      ERROR_PRINT( "connection index %lu is not of a client connection", clientID );
  }
  ThreadSafeMaps.ReleaseItem( globalConnectionsList, clientID );
  
  return firstMessage;
}

bool AsyncIPNetwork_WriteMessage( unsigned long connectionID, const char* message )
{
  AsyncIPConnection connection = ThreadSafeMaps.AquireItem( globalConnectionsList, connectionID );
  if( connection == NULL ) return false;
  
  if( ThreadSafeQueues.GetItemsCount( connection->writeQueue ) >= QUEUE_MAX_ITEMS )
    /*DEBUG_UPDATE*/DEBUG_PRINT( "connection index %lu write queue is full", connectionID );
  
  ThreadSafeQueues.Enqueue( connection->writeQueue, (void*) message, TSQUEUE_NOWAIT );
  
  ThreadSafeMaps.ReleaseItem( globalConnectionsList, connectionID );
  
  return true;
}

unsigned long AsyncIPNetwork_GetClient( unsigned long serverID )
{
  unsigned long firstClient = (unsigned long) IP_CONNECTION_INVALID_ID;

  AsyncIPConnection server = ThreadSafeMaps.AquireItem( globalConnectionsList, serverID );
  if( server != NULL )
  {
    if( IPNetwork.IsServer( server->baseConnection ) )
    {
      if( ThreadSafeQueues.GetItemsCount( server->readQueue ) > 0 )
      {
        ThreadSafeQueues.Dequeue( server->readQueue, &firstClient, TSQUEUE_WAIT );
    
        /*DEBUG_UPDATE*/DEBUG_PRINT( "new client index from connection index %lu: %lu", serverID, firstClient ); 
      }
    }
    else
      /*ERROR_EVENT*/ERROR_PRINT( "connection index %d is not a server index", serverID );
    
    ThreadSafeMaps.ReleaseItem( globalConnectionsList, serverID );
  }
  
  return firstClient; 
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                           ENDING                                                /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

// Handle socket closing and structures destruction for the given index corresponding connection
void AsyncIPNetwork_CloseConnection( unsigned long connectionID )
{
  AsyncIPConnection connection = ThreadSafeMaps.AquireItem( globalConnectionsList, connectionID );
  if( connection == NULL ) return;
  
  IPNetwork.CloseConnection( connection->baseConnection );
  connection->baseConnection = NULL;
  
  ThreadSafeQueues.Discard( connection->readQueue );
  ThreadSafeQueues.Discard( connection->writeQueue );
  
  ThreadSafeMaps.ReleaseItem( globalConnectionsList, connectionID );
  
  ThreadSafeMaps.RemoveItem( globalConnectionsList, connectionID );
  
  if( ThreadSafeMaps.GetItemsCount( globalConnectionsList ) == 0 )
  {
    isNetworkRunning = false;
    /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "waiting update threads %p and %p for exit", globalReadThread, globalWriteThread );
    (void) Threading.WaitExit( globalReadThread, 5000 );
    /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "read thread for connection id %lu returned", connectionID );     
    (void) Threading.WaitExit( globalWriteThread, 5000 );
    /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "write thread for connection id %lu returned", connectionID ); 
    
    ThreadSafeMaps.Discard( globalConnectionsList );
    globalConnectionsList = NULL;
  }
  
  return;
}
