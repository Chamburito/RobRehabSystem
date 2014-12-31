///////////////////////////////////////////////////////////////////////////////////
/////       Master server application for Robotic Rehabilitation System       /////
///////////////////////////////////////////////////////////////////////////////////

//#include "Python.h"
#include <stdio.h>

#include "script_network.h"
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

// Main program function
int main( int argc, char* argv[] )
{    
  FILE* file;
  char* python_commands;
    
  PyImport_AppendInittab( "NetworkInterface", &initNetworkScriptInterface );
  Py_Initialize();

  file = fopen( "scripts/server_init.py", "r" );
  python_commands = get_text( file );
  fclose( file );
  PyRun_SimpleString( python_commands );
  free( python_commands );

  file = fopen( "scripts/server_update.py", "r" );
  python_commands = get_text( file );
  fclose( file );
  PyRun_SimpleString( python_commands );
  free( python_commands );
    
  endNetworkScriptInterface();
    
  exit( 0 );
}