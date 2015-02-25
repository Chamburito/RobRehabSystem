#ifndef ROBREHAB_NETWORK_H
#define ROBREHAB_NETWORK_H

#ifdef _CVI_
  #include "cvirte_connection.h"
#else
  #include "async_connection.h"
#endif

#include "control.h"

static int info_server_connection_id;
static int data_server_connection_id;

typedef struct _Network_Axis
{
  
}
Network_Axis;

int robrehab_network_init()
{
  if( (info_server_connection_id = open_async_connection( NULL, "10000", TCP )) == -1 )
    return -1;
  if( (data_server_connection_id = open_async_connection( NULL, "10001", UDP )) == -1 )
  {
    close_async_connection( info_server_connection_id );
    return -1;
  }
  
  control_init();
  
  return 0;
}

void robrehab_network_end()
{
  close_async_connection( info_server_connection_id );
  close_async_connection( data_server_connection_id );
  
  control_end();
  
  // End threading subsystem
  end_threading();
}

void process_incoming_messages()
{

}

void send_data_messages()
{

}

#endif //ROBREHAB_NETWORK_H
