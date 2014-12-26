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
  
#define MAX_MESSAGES 10
#define BASE_WAIT_TIME 1
  
typedef struct _Connection_Buffer Connection_Buffer;
typedef char Message[ BUFFER_SIZE ];

// Structure that stores read and write message queues for a Connection struct used asyncronously
struct _Connection_Buffer
{
  Connection* connection;
  struct Message_Queue
  {
    Message cache[ MAX_MESSAGES ];
    size_t first, last;
    Thread_Handle handle;
    Thread_Lock lock;
  } read_queue, write_queue;
  char address[ ADDRESS_LENGTH ];
};

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

  return buffer_list[ server_id ]->connection->n_clients;
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
  
  #ifdef DEBUG_1
  printf( "add_async_connection: socket index: %d\n", connection->sockfd ); 
  #endif
  
  buffer_list = (Connection_Buffer**) realloc( buffer_list, ( n_connections + 1 ) * sizeof(Connection_Buffer*) );
  buffer_list[ n_connections ] = (Connection_Buffer*) malloc( sizeof(Connection_Buffer) );
  
  connection_buffer = buffer_list[ n_connections ];
  
  connection_buffer->connection = connection;
  
  address_string = get_address( connection );
  strcpy( &(connection_buffer->address[ HOST ]), &address_string[ HOST ] );
  strcpy( &(connection_buffer->address[ PORT ]), &address_string[ PORT ] );
  
  connection_buffer->write_queue.first = connection_buffer->write_queue.last = 0;
  connection_buffer->write_queue.lock = get_new_thread_lock();

  connection_buffer->read_queue.first = connection_buffer->read_queue.last = 0;
  connection_buffer->read_queue.lock = get_new_thread_lock();
  
  if( (connection->type & NETWORK_ROLE) == CLIENT )
    connection_buffer->read_queue.handle = run_thread( async_read_queue, (void*) connection_buffer );
  else if( (connection->type & NETWORK_ROLE) == SERVER )
    connection_buffer->read_queue.handle = run_thread( async_accept_clients, (void*) connection_buffer );

  connection_buffer->write_queue.handle = run_thread( async_write_queue, (void*) connection_buffer );
  
  #ifdef DEBUG_1
  printf( "add_async_connection: last connection index: %d - socket fd: %d\n", n_connections, connection->sockfd );
  #endif
  
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
  if( (client_id = open_async_connection( address, port, (server->type & PROTOCOL) | CLIENT )) > 0 )
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
  Message* read_cache = reader_buffer->read_queue.cache;
  Thread_Lock read_lock = reader_buffer->read_queue.lock;
  
  char* message_buffer;
  char* last_message;
  size_t first_message_index, last_message_index;
  
  #ifdef DEBUG_1
  printf( "async_read_queue: connection socket %d caching available messages on thread %x\n", reader_buffer->connection->sockfd, THREAD_ID );
  #endif
  
  while( reader_buffer != NULL )
  {
    first_message_index = reader_buffer->read_queue.first;
    last_message_index = reader_buffer->read_queue.last;
    
    #ifdef DEBUG_2
    printf( "async_read_queue: message list: first: %d - last: %d\n", first_message_index, last_message_index );
    #endif
    
	// Give CPU time to the other read/write threads based on how much of our queue is filled
	if( last_message_index - first_message_index > 0 )
      delay( BASE_WAIT_TIME * ( last_message_index - first_message_index ) );
    
    if( reader_buffer->connection == NULL )
    {
      #ifdef DEBUG_1
      printf( "async_read_queue: connection closed\n" );
      #endif
      break;
    }
    
	// Do not proceed if queue is full
    if( last_message_index - first_message_index >= MAX_MESSAGES )
    {
      #ifdef DEBUG_2
      printf( "async_read_queue: connection socket %d read cache full\n", reader_buffer->connection->sockfd );
      #endif
      continue;
    }
    
	// Blocking call
    if( (message_buffer = receive_message( reader )) != NULL )
    {
      if( message_buffer[0] == '\0' ) continue;
      
      #ifdef DEBUG_1
      printf( "async_read_queue: connection socket %d received message: %s\n", reader->sockfd, reader->buffer );
      #endif
      
      last_message = read_cache[ last_message_index % MAX_MESSAGES ]; // Always keep access index between 0 and MAX_MESSAGES 
      
      strcpy( last_message, message_buffer );
      
	  // Mutex prevent index of the last (newest) message to be read before being incremented
      //LOCK_THREAD( read_lock );
      reader_buffer->read_queue.last++;
      //UNLOCK_THREAD( read_lock );
    }
    else
      break;
  }
  
  exit_thread( 0 );
  return NULL;
}

// Loop of message writing (removing in order from queue) to be called asyncronously
static void* async_write_queue( void* args )
{
  Connection_Buffer* writer_buffer = (Connection_Buffer*) args;
  Connection* writer = writer_buffer->connection;
  Message* write_cache = writer_buffer->write_queue.cache;
  Thread_Lock write_lock = writer_buffer->write_queue.lock;
  
  char* first_message;
  size_t first_message_index, last_message_index;

  #ifdef DEBUG_1
  printf( "async_write_queue: connection socket %d sending messages in cache on thread %x\n", writer_buffer->connection->sockfd, THREAD_ID );
  #endif
  
  while( writer_buffer != NULL )
  {
    first_message_index = writer_buffer->write_queue.first;
    last_message_index = writer_buffer->write_queue.last;
    
    #ifdef DEBUG_2
    printf( "async_write_queue: connection socket %d message list: first: %d - last: %d\n", writer->sockfd, first_message_index, last_message_index );
    #endif
     
    if( writer_buffer->connection == NULL )
    {
      #ifdef DEBUG_1
      printf( "async_write_queue: connection closed\n" );
      #endif
      break;
    }

	// Do not proceed if queue is empty
	if( last_message_index - first_message_index <= 0 )
	{
	  #ifdef DEBUG_2
      printf( "async_write_queue: connection socket %d write cache empty\n", writer_buffer->connection->sockfd );
      #endif
	  delay( BASE_WAIT_TIME ); // Wait a little for messages to be sent
      continue;
	}
    
    first_message = write_cache[ first_message_index % MAX_MESSAGES ]; // Always keep access index between 0 and MAX_MESSAGES 
    
    #ifdef DEBUG_2
    printf( "async_write_queue: connection socket %d sending message: %s\n", writer->sockfd, first_message );
    #endif
    
    if( send_message( writer, first_message ) == -1 )
      break;
    
	// Mutex prevent index of the first (oldest) message to be read before being incremented
    //LOCK_THREAD( write_lock );
    writer_buffer->write_queue.first++;
    //UNLOCK_THREAD( write_lock );
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
  Message* server_cache = server_buffer->read_queue.cache;
  Thread_Lock accept_lock = server_buffer->read_queue.lock;

  char* client_address;
  char* last_message;
  size_t first_message_index, last_message_index;
  
  #ifdef DEBUG_1
  printf( "async_accept_clients: connection socket %d trying to add clients on thread %x\n", server_buffer->connection->sockfd, THREAD_ID );
  #endif
  
  while( server_buffer != NULL ) 
  {
    first_message_index = server_buffer->read_queue.first;
    last_message_index = server_buffer->read_queue.last;
    
	// Give CPU time to the other read/write threads based on how much of our queue is filled
	if( last_message_index - first_message_index > 0 )
      delay( BASE_WAIT_TIME * ( last_message_index - first_message_index ) );
    
    if( server_buffer->connection == NULL )
    {
	  #ifdef DEBUG_1
      printf( "async_accept_clients: connection closed\n" );
	  #endif
      break;
    }
    
	// Do not proceed if queue is full
    if( last_message_index - first_message_index >= MAX_MESSAGES )
    {
      #ifdef DEBUG_2
      printf( "async_accept_clients: connection socket %d read cache full\n", server_buffer->connection->sockfd );
      #endif
      continue;
    }
    
	// Blocking call
    if( (client = accept_client( server )) != NULL )
    {
      if( client->sockfd == 0 ) continue;

      client_address = get_address( client );
      
      #ifdef DEBUG_1
      printf( "async_accept_clients: client accepted: socket: %d - type: %x\n", client->sockfd, client->type );
      printf( "async_accept_clients: host: %s - port: %s\n", &client_address[ HOST ], &client_address[ PORT ] );
      #endif
      
      last_message = server_cache[ last_message_index % MAX_MESSAGES ]; // Always keep access index between 0 and MAX_MESSAGES    
      
      sprintf( last_message, "%u %s %s", n_connections, &client_address[ HOST ], &client_address[ PORT ] );
      
	  // Mutex prevent index of the last (newest) client string to be read before being incremented
      //LOCK_THREAD( accept_lock );
      server_buffer->read_queue.last++;
      //UNLOCK_THREAD( accept_lock );
      
      #ifdef DEBUG_1
      printf( "async_accept_clients: clients number before: %d\n", n_connections );
      #endif
      
      add_async_connection( client );
      
      #ifdef DEBUG_1
      printf( "async_accept_clients: clients number after: %d\n", n_connections );
      #endif
    }
    else
      break;
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
  static Connection_Buffer* connection_buffer;
  static char* first_message;

  if( connection_id < 0 || connection_id >= (int) n_connections )
  {
    fprintf( stderr, "dequeue_message: no queue available for this connection: index %d\n", connection_id );
    return NULL;
  }
  
  connection_buffer = buffer_list[ connection_id ];
  
  if( connection_buffer->read_queue.last - connection_buffer->read_queue.first <= 0 )
  {
    #ifdef DEBUG_2
    fprintf( stderr, "dequeue_message: no messages available for this connection: index %d\n", connection_id );
    #endif
    
    return NULL;
  }
  
  first_message = connection_buffer->read_queue.cache[ connection_buffer->read_queue.first % MAX_MESSAGES ];
  
  #ifdef DEBUG_2
  printf( "dequeue_message: message from connection index %d: %s\n", connection_id, first_message );
  #endif
  
  // Mutex prevent index of the first (oldest) message to be read before being incremented
  //LOCK_THREAD( connection_buffer->read_queue.lock );
  connection_buffer->read_queue.first++;
  //UNLOCK_THREAD( connection_buffer->read_queue.lock );
  
  return first_message;
}

// Put message in the end of the given index corresponding write queue
// Method to be called from the main thread
int enqueue_message( int connection_id, const char* message )
{
  static Connection_Buffer* connection_buffer;
  static char* last_message;

  if( connection_id < 0 || connection_id >= (int) n_connections )
  {
    fprintf( stderr, "enqueue_message: no queue available for this connection: index %d\n", connection_id );    
    return -1;
  }
  
  connection_buffer = buffer_list[ connection_id ];
  
  if( connection_buffer->write_queue.last - connection_buffer->write_queue.first >= MAX_MESSAGES )
  {
    #ifdef DEBUG_2
    fprintf( stderr, "enqueue_message: write queue is full for this connection: index %d\n", connection_id );
    #endif

    return -1;
  }
  
  last_message = connection_buffer->write_queue.cache[ connection_buffer->write_queue.last % MAX_MESSAGES ];
  
  strcpy( last_message, message );
  
  // Mutex prevent index of the last (newest) message to be read before being incremented
  //LOCK_THREAD( connection_buffer->write_queue.lock );
  connection_buffer->write_queue.last++;
  //UNLOCK_THREAD( connection_buffer->write_queue.lock );
  
  return 0;
}

// Get (and remove) message from the end (newest) of the given index corresponding read queue
// Method to be called from the main thread
char* pop_message( int connection_id )
{
  static Connection_Buffer* connection_buffer;
  static char* last_message;

  if( connection_id < 0 || connection_id >= (int) n_connections )
  {
    fprintf( stderr, "pop_message: no queue available for this connection: index %d\n", connection_id );
    return NULL;
  }
  
  connection_buffer = buffer_list[ connection_id ];
  
  if( connection_buffer->read_queue.last - connection_buffer->read_queue.first <= 0 )
  {
    #ifdef DEBUG_2
    fprintf( stderr, "pop_message: no messages available for this connection: index %d\n", connection_id );
    #endif
    
    return NULL;
  }
  
  last_message = connection_buffer->read_queue.cache[ ( connection_buffer->read_queue.last - 1 ) % MAX_MESSAGES ];
  
  #ifdef DEBUG_1
  printf( "pop_message: connection index: %d - message: %s\n", connection_id, last_message );
  #endif
  
  // Mutex prevent index of the last (newest) message to be read before being decremented
  //LOCK_THREAD( connection_buffer->read_queue.lock );
  connection_buffer->read_queue.last--;
  //connection_buffer->read_queue.last = connection_buffer->read_queue.first; // ?
  //UNLOCK_THREAD( connection_buffer->read_queue.lock );

  return last_message;
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
    
    printf( "close_async_connection: waiting threads for connection id %u\n", connection_id );
    
    (void) wait_thread_end( buffer_list[ connection_id ]->read_queue.handle );
    printf( "close_async_connection: read thread for connection id %u returned\n", connection_id );
    (void) wait_thread_end( buffer_list[ connection_id ]->write_queue.handle );
    printf( "close_async_connection: write thread for connection id %u returned\n", connection_id );
    
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
    printf( "close_all_connections: closing connection id %u\n", connection_id );
    close_async_connection( connection_id );
  }
  
  if( n_connections > 0 ) free( buffer_list );
}

#ifdef __cplusplus
}
#endif

#endif /* ASYNC_CONNECT_H */