///////////////////////////////////////////////////////////////////////////////////
///// Graphical user interface for ExoKanguera control and data visualization /////
///////////////////////////////////////////////////////////////////////////////////

#include "script_network.h"
#include "control.h"
#include "Python.h"
#include <iostream>
#include <fstream>

// Utility function for conversion from ascii strings to unicode
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////
///// Python interface for ExoKangura control and data access through CAN interface (CANOpen protocol) /////
///// Exposed for scripts running on the embedded Python interpreter                                   /////
////////////////////////////////////////////////////////////////////////////////////////////////////////////

static PyObject* getStatusWord( PyObject *self, PyObject *args )
{
    int eposId, index;

    if ( !PyArg_ParseTuple( args, "ii", &eposId, &index ) )
        return NULL;

    return Py_BuildValue( "i", EPOS[eposId].PDOgetStatusWord( index ) );
}

static PyObject* setControlWord( PyObject *self, PyObject *args )
{
    int eposId, index, value;

    if ( !PyArg_ParseTuple( args, "iii", &eposId, &index, &value ) )
        return NULL;

    EPOS[eposId].PDOsetControlWord( index, (bool)value );

    return Py_BuildValue("");
}

static PyObject* getAxisValue( PyObject *self, PyObject *args )
{
    int eposId, index;

    if ( !PyArg_ParseTuple( args, "ii", &eposId, &index ) )
        return NULL;

    return Py_BuildValue( "d", EPOS[eposId].PDOgetValue( index ) );
}

static PyObject* setAxisValue( PyObject *self, PyObject *args )
{
    int eposId, index;
    double value;

    if ( !PyArg_ParseTuple( args, "iid", &eposId, &index, &value ) )
        return NULL;

    EPOS[eposId].PDOsetValue( index, value );

    return Py_BuildValue("");
}

static PyObject* getControlValue( PyObject *self, PyObject *args )
{
    int eposId, index;

    if ( !PyArg_ParseTuple( args, "ii", &eposId, &index ) )
        return NULL;

    return Py_BuildValue( "d", EPOS[eposId].GetControlValue( index ) );
}

static PyObject* setControlValue( PyObject *self, PyObject *args )
{
    int eposId, index;
    double value;

    if ( !PyArg_ParseTuple( args, "iid", &eposId, &index, &value ) )
        return NULL;

    EPOS[eposId].SetControlValue( index, value );

    return Py_BuildValue("");
}

static PyObject* setOutput( PyObject *self, PyObject *args )
{
    int eposId, state;

    if ( !PyArg_ParseTuple( args, "ii", &eposId, &state ) )
        return NULL;

    EPOS[eposId].PDOsetOutput( state );

    return Py_BuildValue("");
}

static PyObject* setOperationMode( PyObject *self, PyObject *args )
{
    int eposId, mode;

    if ( !PyArg_ParseTuple( args, "ii", &eposId, &mode ) )
        return NULL;
    
    eposNetwork.Sync();

    EPOS[eposId].VCS_SetOperationMode( mode );

    return Py_BuildValue("");
}

static PyObject* readValues( PyObject *self, PyObject *args )
{
    int eposId;

    if ( !PyArg_ParseTuple( args, "i", &eposId ) )
        return NULL;
    
    eposNetwork.Sync();
    
    EPOS[eposId].ReadPDO01();
    EPOS[eposId].ReadPDO02();

    return Py_BuildValue( "" );
}

static PyObject* writeValues( PyObject *self, PyObject *args )
{
    int eposId;

    if ( !PyArg_ParseTuple( args, "i", &eposId ) )
        return NULL;

    EPOS[eposId].WritePDO01();
    EPOS[eposId].WritePDO02();

    return Py_BuildValue("");
}

static PyObject* setAxisEnabled( PyObject *self, PyObject *args )
{
    int eposId, enable;

    if ( !PyArg_ParseTuple( args, "ii", &eposId, &enable ) )
        return NULL;

    //cout << "Enabling/Disabling axis " << eposId << endl;
    
    if( enable )
		EnableAxis( eposId );
	else
		DisableAxis( eposId );

    return Py_BuildValue("");
}

static PyObject* setControl( PyObject *self, PyObject *args )
{
    int eposId, active, mode;

    if ( !PyArg_ParseTuple( args, "iii", &eposId, &active, &mode ) )
        return NULL;
    
    if( active > 0 ) run_thread( controlFunction( mode ), (void*) &(EPOS[ eposId ]) );
    
    return Py_BuildValue( "" );
}

static PyObject* getExecTime( PyObject *self, PyObject *args )
{
    if ( !PyArg_ParseTuple( args, "" ) )
        return NULL;
    
    return Py_BuildValue( "k", get_exec_time_milisseconds() );
}

// Names of the Python methods exported, with corresponding C function pointer and documentation
static PyMethodDef interfaceMethods[] = {
            { "getStatusWord", getStatusWord, METH_VARARGS, "Get value from the status word." },
            { "setControlWord", setControlWord, METH_VARARGS, "Define value of the control word." },
	        { "getAxisValue", getAxisValue, METH_VARARGS, "Get value from the EPOS motor." },
            { "setAxisValue", setAxisValue, METH_VARARGS, "Define value fot the EPOS motor." },
	        { "getControlValue", getControlValue, METH_VARARGS, "Get control parameter value." },
            { "setControlValue", setControlValue, METH_VARARGS, "Define parameter value for the control." },	    
            { "setOutput", setOutput, METH_VARARGS, "Define state of the output for the selected axis." },
            { "setOperationMode", setOperationMode, METH_VARARGS, "Define operation mode (position, velocity or current) for the selected axis." },
            { "readValues", readValues, METH_VARARGS, "Read bytes from the PDO CAN frames." },
            { "writeValues", writeValues, METH_VARARGS, "Write bytes to the PDO CAN frames." },
	        { "setAxisEnabled", setAxisEnabled, METH_VARARGS, "Set EPOS motor enabled or disabled state." },
	        { "setControl", setControl, METH_VARARGS, "Activate control for the given axis Epos id." },
			{ "getExecTime", getExecTime, METH_VARARGS, "Get the system time in milisseconds." },
            {NULL, NULL, 0, NULL}        // Sentinel
        };

static PyModuleDef interfaceModule = {
    PyModuleDef_HEAD_INIT, "EposInterface", NULL, -1, interfaceMethods,
    NULL, NULL, NULL, NULL
};

static PyObject* initInterfaceModule(void)
{
    return PyModule_Create( &interfaceModule );
}

// Embedded interpreter initialization
string pythonCommands;
void initScriptInterface( /*const char* initFile, const char* loopFile*/ )
{
  #ifdef WIN32
    char pySearchPath[] = "Python33";
    Py_SetPythonHome( nstrws_convert( pySearchPath ) );
  #endif
    
  PyImport_AppendInittab( "EposInterface", &initInterfaceModule );
  PyImport_AppendInittab( "NetworkInterface", &initNetworkScriptInterface );
  Py_Initialize();

  //initInterfaceModule();
  //initNetworkScriptInterface();

  ifstream file;
  file.open( "gui_init.py" );
  getline( file, pythonCommands, '\0' );
  file.close();
  PyRun_SimpleString( pythonCommands.c_str() );
  pythonCommands.clear();

  file.open( "gui_update.py" );
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
    endNetworkScriptInterface();
    Py_Finalize();
}


/////////////////////////////////////////////////////////////
//////////          Main program function          //////////
/////////////////////////////////////////////////////////////

int main( int argc, char** argv ) 
{
    // Start CAN network transmission
    for( int nodeId = 1; nodeId <= N_EPOS; nodeId++ )
      eposNetwork.StartPDOS( nodeId );

	// Initially turn axes motors off
	for( int id = 0; id < N_EPOS; id++ )
      DisableAxis( id );
    
	// Run Program scripts
    initScriptInterface();
    runScriptUpdate();
    endScriptInterface();

    // Turn off axes motors that were turned on
    for( int id = 0; id < N_EPOS; id++ )
      DisableAxis( id );	

    // End CAN network transmission
    eposNetwork.StopPDOS(1);

    delay( 2000 );
    
	// End threading subsystem
    end_threading();

    return 0;
}


