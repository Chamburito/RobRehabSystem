///////////////////////////////////////////////////////////////////////////////////
/////       Master server application for Robotic Rehabilitation System       /////
///////////////////////////////////////////////////////////////////////////////////

#include "Python.h"

#include "script_ip_network.h"
#include "debug.h"

#include <stdio.h>
#include <fcntl.h>

// Utility functions for conversion from ascii strings to unicode
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

// Utility function to extract the content of a text file as a single string (char array)
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

// Embedded interpreter initialization
void init_script_interface()
{ 
  PyImport_AppendInittab( "NetworkInterface", &initIPNetworkScriptInterface );
  Py_Initialize();
}

// Call interpreter to execute script
int run_script( const char* file_name )
{
  static FILE* file;
  static char* python_commands;
  if( (file = fopen( file_name, "r" )) == NULL )
  {
    print_platform_error( "run_script: error opening script file" );
    return -1;
  }
  python_commands = get_text( file );
  fclose( file );
  PyRun_SimpleString( python_commands );
  free( python_commands );
  
  return 0;
}

void end_script_interface()
{
  endIPNetworkScriptInterface();
  Py_Finalize();
}

// Main program function
const int N_ARGS = 3;
int main( int argc, char* argv[] )
{    
  if( argc < N_ARGS )
    fprintf( stderr, "Missing arguments\nApplication usage: RobRehabSystem <init_script_name> <update_script_name>\n\n" );
  else
  {
    // Run Program scripts
    init_script_interface();
    for( int file_index = 1; file_index < N_ARGS; file_index++ )
    {
      if( run_script( argv[ file_index ] ) == -1 )
      {
        fprintf( stderr, "main: error opening script file: %s\n", argv[ file_index ] );
        exit( -1 );
      }
    }
    end_script_interface();
  }

  exit( 0 );
}