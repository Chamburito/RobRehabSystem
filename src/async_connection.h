////////////////////////////////////////////////////////////////////////////////////////
///// Library that combines threading utilities with the socket connection library /////
///// to provide asyncronous network communication methods                         /////
////////////////////////////////////////////////////////////////////////////////////////

#ifndef ASYNC_CONNECT_H
#define ASYNC_CONNECT_H

#ifdef __cplusplus
extern "C"{
#endif

#include "connection.h"
#include "debug.h"

#ifdef WIN32
  #include "timing_windows.h"
  #include "threads_windows.h"
#else
  #include "timing_unix.h"
  #include "threads_unix.h"
#endif

  
///////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                      DATA STRUCTURES                                            /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
  
const size_t MAX_MESSAGES = 10;
const unsigned int BASE_WAIT_TIME = 1;
  
// Structure that stores read and write message queues for a Connection struct used asyncronously
typedef struct _Async_Connection
{
  Connection* connection;
  Data_Queue* read_queue, write_queue;
  Thread_Handle read_thread, write_thread;
  char address[ ADDRESS_LENGTH ];
}
Async_Connection;

// Internal (private) list of asyncronous connections created (accessible only by index)
static Async_Connection** buffer_list = NULL;
static size_t n_connections = 0;

///////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                      INFORMATION UTILITIES                                      /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

// Returns the number of asyncronous connections created (method for encapsulation purposes)
size_t async_connections_get_count()
{
  return n_connections;
}

// Returns number of clients for the server connection of given index
size_t async_connection_get_clients_count( int server_id )
{
  if( server_id < 0 || server_id >= (int) n_connections )
  {
    ERROR_EVENT( "invalid connection index: %d\n", server_id );
    return 0;
  }
  
  if( !(buffer_list[ server_id ]->connection->type & SERVER) )
  {
    ERROR_EVENT( "not a server connection index: %d\n", server_id );
    return 0;
  }

  return connections_count( buffer_list[ server_id ]->connection );
}

// Returns address string (host and port) for the connection of given index
char* async_connection_get_address( int connection_id )
{
  if( connection_id < 0 || connection_id >= (int) n_connections )
  {
    ERROR_EVENT( "invalid connection index: %d\n", connection_id );
    return NULL;
  }
  
  return buffer_list[ connection_id ]->address;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                       INITIALIZATION                                            /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

// Forward definition
static void* async_read_queue( void* );
static void* async_write_queue( void* );
static void* async_accept_clients( void* );

// Create new Async_Connection structure (from a given Connection structure) and add it to the internal list
static int add_async_connection( Connection* connection )
{
  static Async_Connection* connection_buffer;
  static char* address_string;
  
  DEBUG_EVENT( "socket index: %d\n", connection->sockfd );
  
  buffer_list = (Async_Connection**) realloc( buffer_list, ( n_connections + 1 ) * sizeof(Async_Connection*) );
  buffer_list[ n_connections ] = (Async_Connection*) malloc( sizeof(Async_Connection) );
  
  connection_buffer = buffer_list[ n_connections ];
  
  connection_buffer->connection = connection;
  
  address_string = get_address( connection );
  strcpy( &(connection_buffer->address[ HOST ]), &address_string[ HOST ] );
  strcpy( &(connection_buffer->address[ PORT ]), &address_string[ PORT ] );
  
  if( (connection->type & NETWORK_ROLE_MASK) == CLIENT )
  {
    connection_buffer->read_queue = data_queue_init( MAX_MESSAGES, BUFFER_SIZE );  
    connection_buffer->read_thread = thread_start( async_read_queue, (void*) connection_buffer, JOINABLE );
  }
  else if( (connection->type & NETWORK_ROLE_MASK) == SERVER )
  {
    connection_buffer->read_queue = data_queue_init( MAX_MESSAGES, sizeof(int) );
    connection_buffer->read_thread = thread_start( async_accept_clients, (void*) connection_buffer, JOINABLE );
  }

  connection_buffer->write_queue = data_queue_init( MAX_MESSAGES, BUFFER_SIZE );
  connection_buffer->write_thread = thread_start( async_write_queue, (void*) connection_buffer, JOINABLE );
  
  DEBUG_EVENT( "last connection index: %d - socket fd: %d\n", n_connections, connection->sockfd );
  
  return n_connections++;
}

// Creates a new Connection structure (from the defined properties) and add it to the asyncronous connection list
int async_connection_open( const char* host, const char* port, int protocol )
{
  Connection* connection = open_connection( host, port, protocol );
  if( connection == NULL )
  {
    ERROR_EVENT( "failed to create %s %s connection on port %s", ( protocol == TCP ) ? "TCP" : "UDP", 
                                                                 ( host == NULL ) ? "Server" : "Client", port );
    return -1;
  } 
  
  return add_async_connection( connection );
}

// Add async connection client to the client list of the given index corresponding (if server) connection 
int add_async_client( int server_id, const char* address, const char* port )
{
  Connection* server;
  static int client_id;
  static uint16_t type;
  
  if( server_id < 0 || server_id >= (int) n_connections )
  {
    fprintf( stderr, "add_async_client: invalid server connection index: %d\n", server_id );
    return -1;
  }
  
  server = buffer_list[ server_id ]->connection;
  if( (client_id = async_connection_open( address, port, (server->type & PROTOCOL_MASK) | CLIENT )) > 0 )
    add_client( server, buffer_list[ client_id ]->connection );
  
  return client_id;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                     ASYNCRONOUS UPDATE                                          /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

// Loop of message reading (storing in queue) to be called asyncronously for client connections
static void* async_read_queue( void* args )
{
  Async_Connection* reader_buffer = (Async_Connection*) args;
  Connection* reader = reader_buffer->connection;
  Data_Queue* read_queue = reader_buffer->read_queue;
  
  char* last_message;
  size_t message_count;
  
  DEBUG_EVENT( "connection socket %d caching available messages on thread %x\n", reader_buffer->connection->sockfd, THREAD_ID );
  
  while( reader_buffer != NULL )
  {
	  message_count = data_queue_count( read_queue );
	
	  DEBUG_UPDATE( "message list count: %u", message_count );
    
    // Give CPU time to the other read/write threads based on how much of our queue is filled
    //if( message_count > 0 ) delay( BASE_WAIT_TIME * message_count );
    
    if( reader_buffer->connection == NULL )
    {
	    //DEBUG_EVENT( "connection closed" );
      break;
    }
    
    // Do not proceed if queue is full
    if( message_count >= MAX_DATA )
    {
	    DEBUG_UPDATE( "connection socket %d read cache full", reader_buffer->connection->sockfd );
      continue;
    }
    
    // Blocking call
    if( wait_message( reader, 2000 ) != 0 ) 
    {
      DEBUG_UPDATE( "message available on client socket %d", reader->sockfd );
      
      if( (last_message = receive_message( reader )) != NULL )
      {
        if( message_buffer[0] == '\0' ) continue;
        
    		DEBUG_UPDATE( "connection socket %d received message: %s", reader->sockfd, reader->buffer );
		
    		data_queue_write( read_queue, (void*) last_message, WAIT );
      }
      else
        break;
    }
  }
  
  thread_exit( 0 );
  return NULL;
}

// Loop of message writing (removing in order from queue) to be called asyncronously
static void* async_write_queue( void* args )
{
  Async_Connection* writer_buffer = (Async_Connection*) args;
  Connection* writer = writer_buffer->connection;
  Data_Queue* write_queue = writer_buffer->write_queue;
  
  char first_message[ BUFFER_SIZE ];
  size_t message_count;

  DEBUG_EVENT( "connection socket %d sending messages in cache on thread %x\n", writer_buffer->connection->sockfd, THREAD_ID );
  
  while( writer_buffer != NULL )
  {
  	message_count = data_queue_count( write_queue );
	  
  	DEBUG_UPDATE( "connection socket %d message count: %u", writer->sockfd, message_count );
    
    if( writer_buffer->connection == NULL )
    {
	    //DEBUG_EVENT( "connection closed" );
      break;
    }
    
    // Do not proceed if queue is empty
    if( message_count <= 0 )
    {
	    DEBUG_UPDATE( "connection socket %d write cache empty\n", writer_buffer->connection->sockfd );
      delay( BASE_WAIT_TIME ); // Wait a little for messages to be sent
      continue;
    }
    
	  data_queue_read( write_queue, (void*) first_message );
    
	  DEBUG_UPDATE( "connection socket %d sending message: %s\n", writer->sockfd, message_buffer );
    
    if( send_message( writer, first_message ) == -1 )
      break;
  }
  
  thread_exit( 1 );
  return NULL;
}

// Loop of client accepting (storing in client list) to be called asyncronously for server connections
// The message queue stores the index and address of the detected remote connection in string format
static void* async_accept_clients( void* args )
{  
  Async_Connection* server_buffer = (Async_Connection*) args;
  Connection* server = server_buffer->connection;
  Connection* client;
  Data_Queue* client_queue = server_buffer->read_queue;

  int last_client;
  size_t message_count;
  
  DEBUG_EVENT( "connection socket %d trying to add clients on thread %x\n", server_buffer->connection->sockfd, THREAD_ID );
  
  while( server_buffer != NULL ) 
  {
    message_count = data_queue_count( client_queue );
    
    // Give CPU time to the other read/write threads based on how much of our queue is filled
    //if( message_count > 0 ) delay( BASE_WAIT_TIME * message_count );
    
    if( server_buffer->connection == NULL )
    {
      DEBUG_EVENT( "connection closed\n" ); 
      break;
    }
    
    // Do not proceed if queue is full
    if( message_count >= MAX_DATA )
    {
	    DEBUG_UPDATE( "connection socket %d read cache full\n", server_buffer->connection->sockfd );
      continue;
    }
    
    // Blocking call
    if( wait_message( server, 5000 ) != 0 ) 
    {
      DEBUG_UPDATE( "client available on server socket %d", server->sockfd );
      
      if( (client = accept_client( server )) != NULL )
      {
        if( client->sockfd == 0 ) continue;
        
		    DEBUG_EVENT( "client accepted: socket: %d - type: %x\n\thost: %s - port: %s", 
                     &( get_address( client ) )[ HOST ], &( get_address( client ) )[ PORT ], client->sockfd, client->type );
        
        last_client = n_connections;
        
        data_queue_write( client_queue, &last_client, WAIT );
        
		    DEBUG_UPDATE( "clients number before: %d\n", n_connections );
        
        add_async_connection( client );
        
		    DEBUG_UPDATE( "clients number after: %d\n", n_connections );
      }
      else
        break;
    }
  }
  
  thread_exit( 2 );
  return NULL;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                      SYNCRONOUS UPDATE                                          /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

// Get (and remove) message from the beginning (oldest) of the given index corresponding read queue
// Method to be called from the main thread
char* async_connection_read_message( int connection_id )
{
  static Data_Queue* read_queue;
  static char first_message[ BUFFER_SIZE ];

  if( connection_id < 0 || connection_id >= (int) n_connections )
  {
    ERROR_EVENT( "invalid connection ID: %d", connection_id );
    return NULL;
  }
  
  if( (buffer_list[ connection_id ]->connection->type & NETWORK_ROLE_MASK) != CLIENT )
  {
    ERROR_EVENT( "connection ID %d is not a client connection ID", connection_id );
	  return NULL;
  }
  
  read_queue = buffer_list[ connection_id ]->read_queue;
  
  if( data_queue_count( read_queue ) <= 0 )
  {
	  DEBUG_UPDATE( "no messages available for connection ID %d", connection_id );
    return NULL;
  }
  
  data_queue_read( read_queue, first_message );
  
  DEBUG_UPDATE( "message from connection ID %d: %s", connection_id, first_message );  
  
  return first_message;
}

int async_connection_write_message( int connection_id, const char* message )
{
  static Data_Queue* write_queue;
  static char* last_message;

  if( connection_id < 0 || connection_id >= (int) n_connections )
  {
    ERROR_EVENT( "invalid connection ID: %d", connection_id );
    return -1;
  }
  
  write_queue = buffer_list[ connection_id ]->write_queue;
  
  if( data_queue_count( write_queue ) >= MAX_DATA )
	  DEBUG_UPDATE( "connection ID %d write queue is full", connection_id );
  
  data_queue_write( write_queue, message, REPLACE );
  
  return 0;
}

int async_connection_get_client( int connection_id )
{
  static Data_Queue* client_queue;
  static int first_client;

  if( connection_id < 0 || connection_id >= (int) n_connections )
  {
    ERROR_EVENT( "invalid connection ID: %d", connection_id );
    return -1;
  }
  
  if( (buffer_list[ connection_id ]->connection->type & NETWORK_ROLE_MASK) != SERVER )
  {
    ERROR_EVENT( "connection ID %d is not a server ID", connection_id );
	  return -1;
  }
  
  client_queue = buffer_list[ connection_id ]->read_queue;
  
  if( data_queue_count( read_queue ) <= 0 )
  {
	  DEBUG_UPDATE( "no new clients available for this connection: index %d", connection_id );
    return -1;
  }
  
  data_queue_read( read_queue, &first_client );
  
  DEBUG_UPDATE( "new client index from connection index %d: %d", connection_id, first_client );  
  
  return first_client; 
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                           ENDING                                                /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

// Handle socket closing and structures destruction for the given index corresponding connection
void async_connection_close( int connection_id )
{
  if( connection_id < 0 || connection_id >= (int) n_connections )
  {
    ERROR_EVENT( "invalid connection index: %d", connection_id );
    return;
  }
  
  if( buffer_list[ connection_id ] != NULL )
  {
    close_connection( buffer_list[ connection_id ]->connection );
    buffer_list[ connection_id ]->connection = NULL;
	
	  DEBUG_EVENT( "waiting threads for connection id %u", connection_id );
    
    (void) thread_wait_exit( buffer_list[ connection_id ]->read_thread, 5000 );
    
    DEBUG_EVENT( "read thread for connection id %u returned", connection_id );     
    
    (void) thread_wait_exit( buffer_list[ connection_id ]->write_thread, 5000 );
    
	  DEBUG_EVENT( "write thread for connection id %u returned !", connection_id ); 
    
	  data_queue_end( buffer_list[ connection_id ]->read_queue );
	  data_queue_end( buffer_list[ connection_id ]->write_queue );
	
    free( buffer_list[ connection_id ] );
    buffer_list[ connection_id ] = NULL;
  }
  
  return;
}

// Close all async connections created
void async_connections_close_all()
{
  size_t connection_id;
  for( connection_id = 0; connection_id < n_connections; connection_id++ )
  {
	  DEBUG_UPDATE( "closing connection id %u", connection_id );
    
    async_connection_close( connection_id );
  }
  
  if( n_connections > 0 ) free( buffer_list );
}

#ifdef __cplusplus
}
#endif

#endif /* ASYNC_CONNECT_H */
