//#include "Python.h"
#include <stdio.h>

#include "script_network.h"
#include <fcntl.h>

#ifdef WIN32

  #include <Windows.h>

  /** Windows char * to wchar_t conversion. */
  wchar_t *nstrws_convert( char* raw ) 
  {
    int size_needed = MultiByteToWideChar( CP_UTF8, 0, raw, -1, NULL, 0 );
    wchar_t *rtn = (wchar_t *) calloc( 1, size_needed * sizeof(wchar_t) );
    MultiByteToWideChar( CP_UTF8, 0, raw, -1, rtn, size_needed );
    return rtn;
  }

#else

  #include <locale.h>
  
  /** Unix-like platform char * to wchar_t conversion. */
    wchar_t* nstrws_convert( char* raw ) 
    {
      wchar_t* rtn = (wchar_t*) calloc( 1, (sizeof(wchar_t) * (strlen(raw) + 1)) );
      setlocale( LC_ALL, "en_US.UTF-8" ); // Seriously, eat a bag of dicks python developers. Unless you do this python 3 crashes.
      mbstowcs( rtn, raw, strlen(raw) );
      return rtn;
    }

#endif

char* get_text( FILE* file )
{
  char* text;
  long size;
  
  fseek( file, 0, SEEK_END );
  size = ftell( file );
  fseek( file, 0, SEEK_SET );
  
  text = (char*) calloc( (size + 1), sizeof(char) );
  
  fread( text, sizeof(char), size, file );
  text[ size ] = '\0';

  return text;
}

int main( int argc, char* argv[] )
{    
    FILE* file;
    char* python_commands;
  
  #ifdef WIN32
    char pySearchPath[] = "Python33";
    Py_SetPythonHome( nstrws_convert( pySearchPath ) );
  #endif
    
    PyImport_AppendInittab( "NetworkInterface", &initNetworkScriptInterface );
    Py_Initialize();

    file = fopen( "init.py", "r" );
    python_commands = get_text( file );
    fclose( file );
	//printf( "init: %s\n", python_commands );
    PyRun_SimpleString( python_commands );
    free( python_commands );

    file = fopen( "update.py", "r" );
    python_commands = get_text( file );
    fclose( file );
	//printf( "update: %s\n", python_commands );
    PyRun_SimpleString( python_commands );
    free( python_commands );
    
    endNetworkScriptInterface();

	/*int i;
	Connection *server, *client;

  printf( "server\n" );
  server = open_connection( NULL, "3490", SOCK_STREAM );
  printf( "server id: %d\n", server->sockfd );

  printf( "client\n" );
  while( wait_message( server, 100 ) == 0 )
    continue;
  
  client = add_client( server );
  printf( "client id: %d\n", client->sockfd );
  
  printf( "read\n" );
  
  for( i = 0; i < 100; i++ )
    printf( "%s\n", receive_message( client ) );
  
  close_connection( server );
  close_connection( client );*/
  
  //EXIT_THREAD;
  exit( 0 );
}