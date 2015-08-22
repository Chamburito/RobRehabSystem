///////////////////////////////////////////////////////////////////////////////////////////////////////////
///// Interface of Internet connection functions exposed to Python scripts running on the embedded    /////
///// interpreter. Just a wrapper around the methods of the C libraries for asyncrounous networking   /////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef SCRIPT_IP_NETWORK_H
#define SCRIPT_IP_NETWORK_H

#ifdef __cplusplus
extern "C"{
#endif

#include "Python.h"
    
#include "async_connection.h"

static PyObject* createConnection( PyObject *self, PyObject *args )
{
  const char* address;
  const char* port;
  int connected;

  static int connection_id;

  if ( !PyArg_ParseTuple( args, "ssi", &address, &port, &connected ) )
    return NULL;
    
  if( strcmp( address, "" ) == 0 || strcmp( address, "0.0.0.0" ) == 0 || strcmp( address, "::" ) == 0 )
    connection_id = open_async_connection( NULL, port, connected );
  else 
    connection_id = open_async_connection( address, port, connected );
    
  if( connection_id == -1 ) return Py_BuildValue( "" );

  return Py_BuildValue( "i", connection_id );
}

static PyObject* addClient( PyObject *self, PyObject *args )
{
  int server_id;
  const char* address;
  const char* port;
  
  int client_id;
  
  if ( !PyArg_ParseTuple( args, "iss", &server_id, &address, &port ) )
        return NULL;
  
  client_id = add_async_client( server_id, address, port );
  
  if( client_id > 0 )
    return Py_BuildValue( "i", client_id );
  else
    return Py_BuildValue( "" );
}

static PyObject* sendMessage( PyObject *self, PyObject *args )
{
  size_t connection_id;
  const char* message;

  if ( !PyArg_ParseTuple( args, "is", &connection_id, &message ) )
    return NULL;

  //return Py_BuildValue( "i", enqueue_message( connection_id, message ) );
  return Py_BuildValue( "i", dispatch_message( connection_id, message ) );
}

static PyObject* receiveMessage( PyObject *self, PyObject *args )
{
  size_t connection_id;
  char* message;

  if ( !PyArg_ParseTuple( args, "i", &connection_id ) )
    return NULL;

  if( (message = dequeue_message( connection_id )) == NULL )
    return Py_BuildValue( "" );

  return Py_BuildValue( "s", message );
}

static PyObject* receiveLastMessage( PyObject *self, PyObject *args )
{
  size_t connection_id;
  char* message;

  if ( !PyArg_ParseTuple( args, "i", &connection_id ) )
    return NULL;

  if( (message = pop_message( connection_id )) == NULL )
    return Py_BuildValue( "" );

  return Py_BuildValue( "s", message );
}

static PyObject* connectionsNumber( PyObject *self, PyObject *args )
{
  size_t connection_id;

  if ( !PyArg_ParseTuple( args, "" ) )
    return NULL;
    
  return Py_BuildValue( "i", connections_number() );
}

static PyObject* clientsNumber( PyObject *self, PyObject *args )
{
  size_t server_id;

  if ( !PyArg_ParseTuple( args, "i", &server_id ) )
    return NULL;
    
  return Py_BuildValue( "i", clients_number( server_id ) );
}

static PyObject* getAddress( PyObject *self, PyObject *args )
{
  size_t connection_id;

  static char* address;

  if ( !PyArg_ParseTuple( args, "i", &connection_id ) )
    return NULL;

  address = connection_address( connection_id );
    
  if( address == NULL ) return Py_BuildValue( "" );
    
  return Py_BuildValue( "(ss)", &address[ HOST ], &address[ PORT ] );
}

static PyObject* closeConnection( PyObject *self, PyObject *args )
{
  size_t connection_id;

  if ( !PyArg_ParseTuple( args, "i", &connection_id ) )
    return NULL;

  close_async_connection( connection_id );
    
  return Py_BuildValue( "" );
}

// Names of the Python methods exported, with corresponding C function pointer and documentation
static PyMethodDef ipNetworkMethods[] = {
    { "createConnection", createConnection, METH_VARARGS, "Stabilish network connection to remote computer." },
    { "addClient", addClient, METH_VARARGS, "Add an existing connection to a server connection clients list." },
    { "sendMessage", sendMessage, METH_VARARGS, "Send string through defined connection." },
    { "receiveMessage", receiveMessage, METH_VARARGS, "Get string from defined connection." },
    { "receiveLastMessage", receiveLastMessage, METH_VARARGS, "Get last string received from defined connection." },
    { "connectionsNumber", connectionsNumber, METH_VARARGS, "Get size of open connections list." },
    { "clientsNumber", clientsNumber, METH_VARARGS, "Get size of clients list for defined server index." },
    { "getAddress", getAddress, METH_VARARGS, "Get host and port strings from defined connection." },
    { "closeConnection", closeConnection, METH_VARARGS, "Close socket and end read/write threads for defined connection." },
    { NULL, NULL, 0, NULL }        // Sentinel
  };

static PyModuleDef ipNetworkModule = {
    PyModuleDef_HEAD_INIT, "IPNetworkInterface", NULL, -1, ipNetworkMethods,
    NULL, NULL, NULL, NULL
};

PyObject* initIPNetworkScriptInterface( void )
{
  PyObject *module, *dictionary, *macro;

  module = PyModule_Create( &ipNetworkModule );
  dictionary = PyModule_GetDict( module );

  macro = PyLong_FromLong( 0 );
  PyDict_SetItemString( dictionary, "UDP", macro );
  Py_DECREF( macro );

  macro = PyLong_FromLong( 1 );
  PyDict_SetItemString( dictionary, "TCP", macro );
  Py_DECREF( macro );

  macro = PyLong_FromLong( BUFFER_SIZE );
  PyDict_SetItemString( dictionary, "BUFFER_SIZE", macro );
  Py_DECREF( macro );

  return module;
}

void endIPNetworkScriptInterface()
{
  close_all_connections();
  end_threading();
}

#ifdef __cplusplus
}
#endif

#endif // SCRIPT_IP_NETWORK_H