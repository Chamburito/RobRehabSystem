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

#ifdef _CVI_DLL_
  #include "timing_realtime.h"
  #include "threads_realtime.h"
#elif WIN32
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
typedef struct _Connection_Buffer
{
  Connection* connection;
  Data_Queue* read_queue, write_queue;
  Thread_Handle read_thread, write_thread;
  char address[ ADDRESS_LENGTH ];
}
Connection_Buffer;

// Internal (private) list of asyncronous connections created (accessible only by index)
static Connection_Buffer** buffer_list = NULL;
static size_t n_connections = 0;

///////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                      INFORMATION UTILITIES                                      /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

// Returns the number of asyncronous connections created (method for encapsulation purposes)
size_t connections_number()
{
  return n_connections;
}

// Returns number of clients for the server connection of given index
size_t clients_number( int server_id )
{
  if( server_id < 0 || server_id >= (int) n_connections )
  {
    fprintf( stderr, "clients_number: invalid connection index: %d\n", server_id );
    return 0;
  }
  
  if( !(buffer_list[ server_id ]->connection->type & SERVER) )
  {
    fprintf( stderr, "clients_number: not a server connection index: %d\n", server_id );
    return 0;
  }

  return connections_count( buffer_list[ server_id ]->connection );
}

// Returns address string (host and port) for the connection of given index
char* connection_address( int connection_id )
{
  if( connection_id < 0 || connection_id >= (int) n_connections )
  {
    fprintf( stderr, "connection_address: invalid connection index: %d\n", connection_id );
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

// Create new Connection_Buffer structure (from a given Connection structure) and add it to the internal list
static int add_async_connection( Connection* connection )
{
  static Connection_Buffer* connection_buffer;
  static char* address_string;
  
  EVENT_DEBUG( "socket index: %d\n", connection->sockfd );
  
  buffer_list = (Connection_Buffer**) realloc( buffer_list, ( n_connections + 1 ) * sizeof(Connection_Buffer*) );
  buffer_list[ n_connections ] = (Connection_Buffer*) malloc( sizeof(Connection_Buffer) );
  
  connection_buffer = buffer_list[ n_connections ];
  
  connection_buffer->connection = connection;
  
  address_string = get_address( connection );
  strcpy( &(connection_buffer->address[ HOST ]), &address_string[ HOST ] );
  strcpy( &(connection_buffer->address[ PORT ]), &address_string[ PORT ] );
  
  if( (connection->type & NETWORK_ROLE_MASK) == CLIENT )
  {
    connection_buffer->read_queue = data_queue_init( MAX_MESSAGES, BUFFER_SIZE );  
    connection_buffer->read_thread = run_thread( async_read_queue, (void*) connection_buffer, JOINABLE );
  }
  else if( (connection->type & NETWORK_ROLE_MASK) == SERVER )
  {
    connection_buffer->read_queue = data_queue_init( MAX_MESSAGES, sizeof(int) );
    connection_buffer->read_thread = run_thread( async_accept_clients, (void*) connection_buffer, JOINABLE );
  }

  connection_buffer->write_queue = data_queue_init( MAX_MESSAGES, BUFFER_SIZE );
  connection_buffer->write_thread = run_thread( async_write_queue, (void*) connection_buffer, JOINABLE );
  
  EVENT_DEBUG( "last connection index: %d - socket fd: %d\n", n_connections, connection->sockfd );
  
  return n_connections++;
}

// Creates a new Connection structure (from the defined properties) and add it to the asyncronous connection list
int open_async_connection( const char* host, const char* port, short connected )
{
  Connection* connection = open_connection( host, port, ( connected == 0 ) ? UDP : TCP );
  if( connection == NULL )
  {
    fprintf( stderr, "open_async_connection: failed to create connection\n" );
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
  if( (client_id = open_async_connection( address, port, (server->type & PROTOCOL_MASK) | CLIENT )) > 0 )
    add_client( server, buffer_list[ client_id ]->connection );
  
  return client_id;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                     ASYNCRONOUS UPDATE                                          /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

// Loop of message reading (storing in queue) to be called asyncronously for client connections
static void* async_read_queue( void* args )
{
  Connection_Buffer* reader_buffer = (Connection_Buffer*) args;
  Connection* reader = reader_buffer->connection;
  Data_Queue* read_queue = reader_buffer->read_queue;
  
  char* last_message;
  size_t message_count;
  
  EVENT_DEBUG( "connection socket %d caching available messages on thread %x\n", reader_buffer->connection->sockfd, THREAD_ID );
  
  while( reader_buffer != NULL )
  {
	message_count = data_queue_count( read_queue );
	
	LOOP_DEBUG( "message list count: %u\n", message_count );
    
    // Give CPU time to the other read/write threads based on how much of our queue is filled
    if( message_count > 0 ) delay( BASE_WAIT_TIME * message_count );
    
    if( reader_buffer->connection == NULL )
    {
	  EVENT_DEBUG( "connection closed\n" );
      break;
    }
    
    // Do not proceed if queue is full
    if( message_count >= MAX_MESSAGES )
    {
	  LOOP_DEBUG( "connection socket %d read cache full\n", reader_buffer->connection->sockfd );
      continue;
    }
    
    // Blocking call
    if( wait_message( reader, 2000 ) != 0 ) 
    {
      printf( "async_read_queue: message available\n" );
      if( (last_message = receive_message( reader )) != NULL )
      {
        if( message_buffer[0] == '\0' ) continue;
        
		LOOP_DEBUG( "connection socket %d received message: %s\n", reader->sockfd, reader->buffer );
		
		data_queue_write( read_queue, (void*) last_message );
      }
      else
        break;
    }
  }
  
  exit_thread( 0 );
  return NULL;
}

// Loop of message writing (removing in order from queue) to be called asyncronously
static void* async_write_queue( void* args )
{
  Connection_Buffer* writer_buffer = (Connection_Buffer*) args;
  Connection* writer = writer_buffer->connection;
  Data_Queue* write_queue = writer_buffer->write_queue;
  
  char first_message[ BUFFER_SIZE ];
  size_t message_count;

  EVENT_DEBUG( "connection socket %d sending messages in cache on thread %x\n", writer_buffer->connection->sockfd, THREAD_ID );
  
  while( writer_buffer != NULL )
  {
	message_count = data_queue_count( write_queue );
	  
	LOOP_DEBUG( "connection socket %d message count: %u\n", writer->sockfd, message_count );
    
    if( writer_buffer->connection == NULL )
    {
	  EVENT_DEBUG( "connection closed\n" );
      break;
    }
    
    // Do not proceed if queue is empty
    if( message_count <= 0 )
    {
	  LOOP_DEBUG( "connection socket %d write cache empty\n", writer_buffer->connection->sockfd );
      delay( BASE_WAIT_TIME ); // Wait a little for messages to be sent
      continue;
    }
    
	data_queue_read( write_queue, (void*) first_message );
    
	LOOP_DEBUG( "connection socket %d sending message: %s\n", writer->sockfd, message_buffer );
    
    if( send_message( writer, first_message ) == -1 )
      break;
  }
  
  exit_thread( 1 );
  return NULL;
}

// Loop of client accepting (storing in client list) to be called asyncronously for server connections
// The message queue stores the index and address of the detected remote connection in string format
static void* async_accept_clients( void* args )
{  
  Connection_Buffer* server_buffer = (Connection_Buffer*) args;
  Connection* server = server_buffer->connection;
  Connection* client;
  Data_Queue* client_queue = server_buffer->read_queue;

  int last_client;
  size_t message_count;
  
  EVENT_DEBUG( "connection socket %d trying to add clients on thread %x\n", server_buffer->connection->sockfd, THREAD_ID );
  
  while( server_buffer != NULL ) 
  {
    message_count = data_queue_count( client_queue );
    
    // Give CPU time to the other read/write threads based on how much of our queue is filled
    if( message_count > 0 ) delay( BASE_WAIT_TIME * message_count );
    
    if( server_buffer->connection == NULL )
    {
      EVENT_DEBUG( "connection closed\n" ); 
      break;
    }
    
    // Do not proceed if queue is full
    if( message_count >= MAX_MESSAGES )
    {
	  LOOP_DEBUG( "connection socket %d read cache full\n", server_buffer->connection->sockfd );
      continue;
    }
    
    // Blocking call
    if( wait_message( server, 5000 ) != 0 ) 
    {
      printf( "async_accept_clients: client available\n" );
      if( (client = accept_client( server )) != NULL )
      {
        if( client->sockfd == 0 ) continue;
        
		EVENT_DEBUG( "client accepted: socket: %d - type: %x\n", client->sockfd, client->type );
		EVENT_DEBUG( "host: %s - port: %s\n", &( get_address( client ) )[ HOST ], &( get_address( client ) )[ PORT ] );
        
        last_client = n_connections;
        
        data_queue_write( client_queue, &last_client );
        
		LOOP_DEBUG( "clients number before: %d\n", n_connections );
        
        add_async_connection( client );
        
		LOOP_DEBUG( "clients number after: %d\n", n_connections );
      }
      else
        break;
    }
  }
  
  exit_thread( 2 );
  return NULL;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                      SYNCRONOUS UPDATE                                          /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

// Get (and remove) message from the beginning (oldest) of the given index corresponding read queue
// Method to be called from the main thread
char* dequeue_message( int connection_id )
{
  static Data_Queue* read_queue;
  static char first_message[ BUFFER_SIZE ];

  if( connection_id < 0 || connection_id >= (int) n_connections )
  {
    fprintf( stderr, "%s: no queue available for this connection: index %d\n", __func__, connection_id );
    return NULL;
  }
  
  if( (buffer_list[ connection_id ]->connection->type & NETWORK_ROLE_MASK) != CLIENT )
  {
    fprintf( stderr, "%s: not a client connection: index %d\n", __func__, connection_id );  
	return NULL;
  }
  
  read_queue = buffer_list[ connection_id ]->read_queue;
  
  if( data_queue_count( read_queue ) <= 0 )
  {
	LOOP_DEBUG( "no messages available for this connection: index %d\n", connection_id );
    return NULL;
  }
  
  data_queue_read( read_queue, first_message );
  
  LOOP_DEBUG( "message from connection index %d: %s\n", connection_id, first_message );  
  
  return first_message;
}

int enqueue_message( int connection_id, const char* message )
{
  static Data_Queue* write_queue;
  static char* last_message;

  if( connection_id < 0 || connection_id >= (int) n_connections )
  {
    fprintf( stderr, "%s: no queue available for this connection: index %d\n", __func__, connection_id );    
    return -1;
  }
  
  write_queue = buffer_list[ connection_id ]->write_queue;
  
  if( data_queue_count( read_queue ) >= MAX_MESSAGES )
	LOOP_DEBUG( "write queue is full for this connection: index %d\n", connection_id );
  
  data_queue_write( write_queue, message );
  
  return 0;
}

int get_last_client( int connection_id )
{
  static Data_Queue* client_queue;
  static int first_client;

  if( connection_id < 0 || connection_id >= (int) n_connections )
  {
    fprintf( stderr, "%s: no queue available for this connection: index %d\n", __func__, connection_id );
    return -1;
  }
  
  if( (buffer_list[ connection_id ]->connection->type & NETWORK_ROLE_MASK) != SERVER )
  {
    fprintf( stderr, "%s: not a server connection: index %d\n", __func__, connection_id );  
	return -1;
  }
  
  client_queue = buffer_list[ connection_id ]->read_queue;
  
  if( data_queue_count( read_queue ) <= 0 )
  {
	LOOP_DEBUG( "no new clients available for this connection: index %d\n", connection_id );
    return -1;
  }
  
  data_queue_read( read_queue, &first_client );
  
  LOOP_DEBUG( "new client index from connection index %d: %d\n", connection_id, first_client );  
  
  return first_client; 
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                                           ENDING                                                /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

// Handle socket closing and structures destruction for the given index corresponding connection
void close_async_connection( int connection_id )
{
  if( connection_id < 0 || connection_id >= (int) n_connections )
  {
    fprintf( stderr, "close_async_connection: invalid connection index: %d\n", connection_id );
    return;
  }
  
  if( buffer_list[ connection_id ] != NULL )
  {
    close_connection( buffer_list[ connection_id ]->connection );
    buffer_list[ connection_id ]->connection = NULL;
	
	EVENT_DEBUG( "waiting threads for connection id %u\n", connection_id );
    
    (void) wait_thread_end( buffer_list[ connection_id ]->read_thread, 5000 );
    
    EVENT_DEBUG( "read thread for connection id %u returned\n", connection_id );     
    
    (void) wait_thread_end( buffer_list[ connection_id ]->write_thread, 5000 );
    
	EVENT_DEBUG( "write thread for connection id %u returned !\n", connection_id ); 
    
	data_queue_end( buffer_list[ connection_id ]->read_queue );
	data_queue_end( buffer_list[ connection_id ]->write_queue );
	
    free( buffer_list[ connection_id ] );
    buffer_list[ connection_id ] = NULL;
  }
  
  return;
}

// Close all async connections created
void close_all_connections()
{
  size_t connection_id;
  for( connection_id = 0; connection_id < n_connections; connection_id++ )
  {
	LOOP_DEBUG( "closing connection id %u\n", connection_id );
    
    close_async_connection( connection_id );
  }
  
  if( n_connections > 0 ) free( buffer_list );
}

#ifdef __cplusplus
}
#endif

#endif /* ASYNC_CONNECT_H */
