#ifndef ASYNC_IP_NETWORK_H
#define ASYNC_IP_NETWORK_H

#include <iostream>
#include <string>
#include <deque>
#include <thread>
#include <mutex>
#include <chrono>

#include "connection.h"

using std::string;

class AsyncConnection
{
  protected:
    Connection* connection = nullptr;
    virtual void SetupConnection() = 0;
    
    template< typename DataType >
    class AsyncDataQueue
    {
      private:
        std::deque<DataType> cache;
        std::mutex lock;
      public:
        const int MAX_DATA = 10;
        void EnqueueData( DataType& data )
        {
          std::lock_guard<std::mutex> locker( lock );
          cache.push_front( data );
          cache.resize( MAX_DATA );
        }
        DataType DequeueData()
        {
          if( not cache.empty() )
          {
            std::lock_guard<std::mutex> locker(  lock );
            DataType data = cache.back();
            cache.pop_back();
            return data;
          }
          return DataType();
        }
        size_t DataCount() { return cache.size(); }
    };
  
  private:
    const int BASE_WAIT_TIME = 1;
      
  public:
    AsyncConnection() {}
    ~AsyncConnection() {}
    string GetAddress() { return string( get_address( connection ); }
};

class ClientConnection : public AsyncConnection
{
  private:
    AsyncDataQueue<string> readQueue, writeQueue;
    std::thread readThread, writeThread;

    void ReadLoop()
    {
      while( true )
      {
        // Give CPU time to the other read/write threads based on how much of our queue is filled
        if( readQueue.DataCount() > 0 )
          std::this_thread::sleep_for( std::chrono::milliseconds( BASE_WAIT_TIME * readQueue.DataCount() ) );
        
        if( connection == nullptr )
        {
          //#ifdef DEBUG_1
          //printf( "async_read_queue: connection closed\n" );
          //#endif
          break;
        }
        
        // Do not proceed if queue is full
        if( readQueue.DataCount() == AsyncDataQueue::MAX_DATA )
        {
          //#ifdef DEBUG_2
          //printf( "async_read_queue: connection socket %d read cache full\n", reader_buffer->connection->sockfd );
          //#endif
          continue;
        }
        
        // Blocking call
        if( wait_message( connection, 2000 ) != 0 ) 
        {
          char* message_buffer;
          if( (message_buffer = receive_message( connection )) != NULL )
          {
            if( message_buffer[0] == '\0' ) continue;
                                        
            //#ifdef DEBUG_2
            //printf( "async_read_queue: connection socket %d received message: %s\n", reader->sockfd, reader->buffer );
            //#endif
            
            readQueue.EnqueueData( string( message_buffer ) );
          }
          else
            break;
        }
      }
    }
    
    void WriteLoop()
    {   
      while( true )
      {
        //#ifdef DEBUG_2
        //printf( "async_write_queue: connection socket %d message list: first: %d - last: %d\n", writer->sockfd, first_message_index, last_message_index );
        //#endif
        
        if( connection == nullptr )
        {
          //#ifdef DEBUG_1
          //printf( "async_write_queue: connection closed\n" );
          //#endif
          break;
        }
        
        // Do not proceed if queue is empty
        if( writeQueue.DataCount() == 0 )
        {
          //#ifdef DEBUG_2
          //printf( "async_write_queue: connection socket %d write cache empty\n", writer_buffer->connection->sockfd );
          //#endif
          std::this_thread::sleep_for( std::chrono::milliseconds( BASE_WAIT_TIME ) ); // Wait a little for messages to be sent
          continue;
        }
        
        string firstMessage = writeQueue.DequeueData(); 
        
        //#ifdef DEBUG_2
        //printf( "async_write_queue: connection socket %d sending message: %s\n", writer->sockfd, first_message );
        //#endif
        
        if( not firstMessage.empty() )
        {
          if( send_message( connection, firstMessage.data() ) == -1 )
            break;
        }
      }
    }
    
  protected:
    void SetupConnection()
    {
      readThread = std::thread( ReadLoop );
      writeThread = std::thread( WriteLoop );
    }
    
  public:
    ClientConnection() {}
    ClientConnection( Connection* connection ) 
    { 
      this->connection = connection;
      SetupConnection();
    }
    ~ClientConnection() 
    {
      close_connection( connection );
      connection = nullptr;
      
      readThread.join();
      writeThread.join();
    }
    inline int SendMessage( string& message ) { writeQueue.EnqueueData( message ); }
    inline string ReceiveMessage() { return readQueue.DequeueData(); }
};

class ServerConnection : public AsyncConnection
{
  private:
    AsyncDataQueue<ClientConnection> clientQueue;
    std::thread acceptThread;
    
    void AcceptLoop()
    {
      while( true ) 
      {
        // Give CPU time to the other read/write threads based on how much of our queue is filled
        if( clientQueue.DataCount() > 0 )
          std::this_thread::sleep_for( std::chrono::milliseconds( BASE_WAIT_TIME * readQueue.DataCount() ) );
        
        if( connection == nullptr )
        {
          //#ifdef DEBUG_1
          //printf( "async_accept_clients: connection closed\n" );
          //#endif
          break;
        }
        
        // Do not proceed if queue is full
        if( clientQueue.DataCount() >= AsyncDataQueue::MAX_DATA )
        {
          //#ifdef DEBUG_2
          //printf( "async_accept_clients: connection socket %d read cache full\n", server_buffer->connection->sockfd );
          //#endif
          continue;
        }
        
        // Blocking call
        if( wait_message( connection, 5000 ) != 0 ) 
        {
          printf( "async_accept_clients: client available\n" );
          Connection* client;
          if( (client = accept_client( connection )) != NULL )
          {
            if( client->sockfd == 0 ) continue;
            
            //#ifdef DEBUG_1
            //printf( "async_accept_clients: client accepted: socket: %d - type: %x\n", client->sockfd, client->type );
            //printf( "async_accept_clients: host: %s - port: %s\n", &client_address[ HOST ], &client_address[ PORT ] );

            //printf( "async_accept_clients: clients number before: %d\n", n_connections );
            //#endif
            
            clientQueue.EnqueueData( ClientConnection( client ) );
            
            //#ifdef DEBUG_1
            //printf( "async_accept_clients: clients number after: %d\n", n_connections );
            //#endif
          }
          else
            break;
        }
      }
    }
    
protected:
  void SetupConnection()
  {
    acceptThread = std::thread( AcceptLoop );
  }
    
public:
  ServerConnection() {}
  ~ServerConnection()
  {
    close_connection( connection );
    connection = nullptr;
    
    acceptThread.join();
  }
  inline ClientConnection GetNewClient() { return clientQueue.DequeueData(); }
};

class TCPClient : public ClientConnection
{
  public:
    TCPClient( string hostName, int portNumber )
    {
      connection = open_connection( hostName.data(), std::to_string( portNumber ).data(), TCP ); 
      SetupConnection();
    }
};

class UDPClient : public ClientConnection
{
  public:
    UDPClient( string hostName, int portNumber )
    {
      connection = open_connection( hostName.data(), std::to_string( portNumber ).data(), UDP ); 
      SetupConnection();
    }
};

class TCPServer : public ServerConnection
{
  private:
    TCPServer( int portNumber )
    {
      connection = open_connection( NULL, std::to_string( portNumber ).data(), TCP );
      SetupConnection();
    }
};

class UDPServer : public ServerConnection
{
  private:
    UDPServer( int portNumber )
    {
      connection = open_connection( NULL, std::to_string( portNumber ).data(), UDP );
      SetupConnection();
    }
};

#endif // IP_NETWORK_H