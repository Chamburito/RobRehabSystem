///////////////////////////////////////////////////////////////////////////////////
///// Graphical user interface for ExoKanguera control and data visualization /////
///////////////////////////////////////////////////////////////////////////////////

#include "script_epos_can.h"
#include "script_ip_network.h"
#include "control.h"
#include "Python.h"
#include <iostream>
#include <fstream>

// Utility functions for conversion from ascii strings to unicode
#ifdef WIN32
  #include <Windows.h>

  /** Windows char * to wchar_t conversion. */
  wchar_t* nstrws_convert( char* raw ) 
  {
    int size_needed = MultiByteToWideChar( CP_UTF8, 0, raw, -1, NULL, 0 );
    wchar_t* rtn = (wchar_t*) calloc( 1, size_needed * sizeof(wchar_t) );
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

using namespace std;

// Embedded interpreter initialization
string pythonCommands;
void initScriptInterface( /*const char* initFile, const char* loopFile*/ )
{ 
  PyImport_AppendInittab( "EposInterface", &initEposScriptInterface );
  PyImport_AppendInittab( "NetworkInterface", &initIPNetworkScriptInterface );
  Py_Initialize();

  //initEposScriptInterface();
  //initIPNetworkScriptInterface();

  ifstream file;
  file.open( "scripts/gui_init.py" );
  getline( file, pythonCommands, '\0' );
  file.close();
  PyRun_SimpleString( pythonCommands.c_str() );
  pythonCommands.clear();

  file.open( "scripts/gui_update.py" );
  getline( file, pythonCommands, '\0' );
  file.close();
}

void runScriptUpdate()
{
    PyRun_SimpleString( pythonCommands.c_str() );
}

void endScriptInterface()
{
    pythonCommands.clear();
    endIPNetworkScriptInterface();
    endEposScriptInterface();
    Py_Finalize();
}


/////////////////////////////////////////////////////////////
//////////          Main program function          //////////
/////////////////////////////////////////////////////////////

int main( int argc, char** argv ) 
{
  // Run Program scripts
  initScriptInterface();
  runScriptUpdate();
  endScriptInterface();

  return 0;
}


