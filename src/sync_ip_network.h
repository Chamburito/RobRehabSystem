#ifndef SYNC_IP_NETWORK_H
#define SYNC_IP_NETWORK_H

#include <iostream>
#include <string>

#include "connection.h"

using std::string;

class SyncConnection
{
  protected:
    Connection* connection = nullptr;
  
  private:
    const int BASE_WAIT_TIME = 1;
      
  public:
    AsyncConnection() {}
    ~AsyncConnection()
    {
      close_connection( connection );
      connection = nullptr;
    }
    string GetAddress() { return string( get_address( connection ); }
};

class ClientConnection : public SyncConnection
{
  public:
    ClientConnection() {}
    ClientConnection( Connection* connection ) 
    { 
      this->connection = connection;
    }
    ~ClientConnection() {}
    inline int SendMessage( string& message ) { send_message( connection, message.data() ); }
    inline string ReceiveMessage() 
    { 
      if( wait_message( connection, 10 ) )
        return string( receive_message( connection ) );
      else
        return string();
    }
};

class ServerConnection : public SyncConnection
{
public:
  ServerConnection() {}
  ~ServerConnection() {}
  inline ClientConnection GetNewClient()
  { 
      if( wait_message( connection, 10 ) )
        return ClientConnection( accept_client( connection ) );
      else
        return ClientConnection();
    }
};

class TCPClient : public ClientConnection
{
  public:
    TCPClient( string hostName, int portNumber )
    {
      connection = open_connection( hostName.data(), std::to_string( portNumber ).data(), TCP ); 
    }
};

class UDPClient : public ClientConnection
{
  public:
    UDPClient( string hostName, int portNumber )
    {
      connection = open_connection( hostName.data(), std::to_string( portNumber ).data(), UDP ); 
    }
};

class TCPServer : public ServerConnection
{
  private:
    TCPServer( int portNumber )
    {
      connection = open_connection( NULL, std::to_string( portNumber ).data(), TCP );
    }
};

class UDPServer : public ServerConnection
{
  private:
    UDPServer( int portNumber )
    {
      connection = open_connection( NULL, std::to_string( portNumber ).data(), UDP );
    }
};

#endif // IP_NETWORK_H