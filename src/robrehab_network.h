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

static ListType info_clients_list;
static ListType data_clients_list;

static char robrehab_axes_info[ IP_CONNECTION_MSG_LEN ] = ""; // String containing used axes names

typedef struct _Network_Axis
{
  
}
Network_Axis;

int robrehab_network_init()
{
  if( (info_server_connection_id = async_connection_open( NULL, "50000", TCP )) == -1 )
    return -1;
  if( (data_server_connection_id = async_connection_open( NULL, "50001", UDP )) == -1 )
  {
    close_async_connection( info_server_connection_id );
    return -1;
  }
  
  info_clients_list = ListCreate( sizeof(int) );
  data_clients_list = ListCreate( sizeof(int) );
  
  control_init();
  
  for( size_t motor_id = 0; motor_id < CONTROL_N_JOINTS; motor_id++ )
  {
    if( control_get_motor_name( motor_id ) != NULL )
      snprintf( robrehab_axes_info, IP_CONNECTION_MSG_LEN, "%s%u:%s:", robrehab_axes_info, motor_id, control_get_motor_name( motor_id ) );
  }
  
  if( strlen( robrehab_axes_info ) > 0 )
    robrehab_axes_info[ strlen( robrehab_axes_info ) - 1 ] = '\0';
  
  return 0;
}

void robrehab_network_end()
{
  close_async_connection( info_server_connection_id );
  close_async_connection( data_server_connection_id );
  
  ListDispose( info_clients_list );
  ListDispose( data_clients_list );
  
  control_end();
}

static int CVICALLBACK motor_info_update( int, void*, void* );
static int CVICALLBACK motor_data_update( int, void*, void* );

void robrehab_network_update()
{
  static int new_info_client;
  static int new_data_client;
  
  if( (new_info_client = async_connection_get_client( info_server_connection_id )) != -1 )
  {
    async_connection_write_message( new_info_client, control_axes_info );
    ListInsertItem( info_clients_list, &new_info_client, END_OF_LIST );
  }
  
  if( (new_data_client = async_connection_get_client( data_server_connection_id )) != -1 )
    ListInsertItem( data_clients_list, &new_data_client, END_OF_LIST );
  
  size_t motor_info_client_list[ CONTROL_N_JOINTS ] = { 0 };
  ListApplyToEach( info_clients_list, 1, motor_info_update, (void*) motor_info_client_list );
  
  size_t motor_data_client_list[ CONTROL_N_JOINTS ] = { 0 };
  ListApplyToEach( data_clients_list, 1, motor_data_update, (void*) motor_data_client_list );
}

static int CVICALLBACK motor_info_update( int index, void* client_id_ref, void* motor_client_data )
{
  int client_id = *((int*) client_id_ref);
  size_t* motor_client_list = (size_t*) motor_client_data;
  
  static char* message_in;
  
  static bool* motor_states_list;
  static char message_out[ IP_CONNECTION_MSG_LEN ] = "";
  
  if( (message_in = async_connection_read_message( client_id )) == NULL ) return 0;
  
  for( char* command = strtok( message_in, ":" ); command != NULL; command = strtok( NULL, ":" ) )
  {
    unsigned int motor_id = (unsigned int) strtoul( command, NULL, 0 );
    
    if( motor_client_list[ motor_id ] != 0 ) continue;
    else motor_client_list[ motor_id ] = client_id;
    
    bool motor_enabled = (bool) strtoul( command, &command, 0 );
    
    if( motor_enabled ) control_enable_motor( motor_id );
    else control_disable_motor( motor_id );
    
    bool reset = (bool) strtoul( command, NULL, 0 );
    
    if( reset ) control_reset_motor( motor_id );
    
    if( (motor_states_list = control_get_motor_status( motor_id )) != NULL )
    {
      snprintf( message_out, IP_CONNECTION_MSG_LEN, "%s%u:", message_out, motor_id );
      
      for( size_t state_index = 0; state_index < AXIS_N_STATES; state_index++ )
        snprintf( message_out, IP_CONNECTION_MSG_LEN, "%s%c ", message_out, ( motor_states_list[ state_index ] == true ) ? '1' : '0' );
    }
  }
  
  if( strlen( message_out ) > 0 )
  {
    message_out[ strlen( message_out ) - 1 ] = '\0';
    async_connection_write_message( client_id, message_out );
  }
  
  return 0;
}

static int CVICALLBACK motor_data_update( int index, void* client_id_ref, void* motor_client_data )
{
  int client_id = *((int*) client_id_ref);
  size_t* motor_client_list = (size_t*) motor_client_data;
  
  static char* message_in;
  
  static char message_out[ IP_CONNECTION_MSG_LEN ] = "";
  
  static double motor_parameters_list[ AXIS_N_PARAMS ];
  
  static double* motor_measures_list;
  
  if( (message_in = async_connection_read_message( client_id )) == NULL ) return 0;
  
  for( char* command = strtok( message_in, ":" ); command != NULL; command = strtok( NULL, ":" ) )
  {
    unsigned int motor_id = (unsigned int) strtoul( command, NULL, 0 );
    
    if( motor_client_list[ motor_id ] != 0 ) continue;
    else motor_client_list[ motor_id ] = client_id;
    
    for( size_t parameter_index = 0; parameter_index < AXIS_N_PARAMS; parameter_index++ )
      motor_parameters_list[ parameter_index ] = strtod( command, &command );
    
    control_config_motor( motor_id, motor_parameters_list );
    
    if( (motor_measures_list = control_get_motor_measures( motor_id )) != NULL )
    {
      snprintf( message_out, IP_CONNECTION_MSG_LEN, "%s%u:", message_out, motor_id );
      
      for( size_t dimension_index = 0; dimension_index < AXIS_N_DIMS; dimension_index++ )
        snprintf( message_out, IP_CONNECTION_MSG_LEN, "%s%.3f ", message_out, motor_measures_list[ dimension_index ] );
    }
  }
  
  if( strlen( message_out ) > 0 )
  {
    message_out[ strlen( message_out ) - 1 ] = '\0';
    async_connection_write_message( client_id, message_out );
  }
  
  return 0;
}

#endif //ROBREHAB_NETWORK_H
