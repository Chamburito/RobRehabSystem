////////////////////////////////////////////////////////////////////////////////////////////////////////////
///// Python interface for ExoKangura control and data access through CAN interface (CANOpen protocol) /////
///// Exposed for scripts running on the embedded Python interpreter                                   /////
////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef SCRIPT_EPOS_NETWORK_H
#define SCRIPT_EPOS_NETWORK_H

#include "control.h"

#include "Python.h"


static PyObject* getStatusWord( PyObject *self, PyObject *args )
{
  int eposId, index;
  
  if ( !PyArg_ParseTuple( args, "ii", &eposId, &index ) )
    return NULL;
  
  return Py_BuildValue( "i", EPOS[ eposId ].PDOgetStatusWord( (StatusWord) index ) );
}

static PyObject* setControlWord( PyObject *self, PyObject *args )
{
  int eposId, index, value;
  
  if ( !PyArg_ParseTuple( args, "iii", &eposId, &index, &value ) )
    return NULL;
  
  EPOS[ eposId ].PDOsetControlWord( (ControlWord) index, (bool) value );
  
  return Py_BuildValue( "" );
}

static PyObject* getAxisValue( PyObject *self, PyObject *args )
{
  int eposId, index;
  
  if ( !PyArg_ParseTuple( args, "ii", &eposId, &index ) )
    return NULL;
  
  return Py_BuildValue( "d", EPOS[ eposId ].PDOgetValue( (Dimension) index ) );
}

static PyObject* setAxisValue( PyObject *self, PyObject *args )
{
  int eposId, index;
  double value;
  
  if ( !PyArg_ParseTuple( args, "iid", &eposId, &index, &value ) )
    return NULL;
  
  EPOS[ eposId ].PDOsetValue( (Dimension) index, value );
  
  return Py_BuildValue( "" );
}

static PyObject* getControlValue( PyObject *self, PyObject *args )
{
  int eposId, index;
  
  if ( !PyArg_ParseTuple( args, "ii", &eposId, &index ) )
    return NULL;
  
  return Py_BuildValue( "d", EPOS[ eposId ].GetControlValue( (Parameter) index ) );
}

static PyObject* setControlValue( PyObject *self, PyObject *args )
{
  int eposId, index;
  double value;
  
  if ( !PyArg_ParseTuple( args, "iid", &eposId, &index, &value ) )
    return NULL;
  
  EPOS[ eposId ].SetControlValue( (Parameter) index, value );
  
  return Py_BuildValue( "" );
}

static PyObject* setOutput( PyObject *self, PyObject *args )
{
  int eposId, state;
  
  if ( !PyArg_ParseTuple( args, "ii", &eposId, &state ) )
    return NULL;
  
  EPOS[ eposId ].PDOsetOutput( state );
  
  return Py_BuildValue( "" );
}

static PyObject* setOperationMode( PyObject *self, PyObject *args )
{
  int eposId, mode;
  
  if ( !PyArg_ParseTuple( args, "ii", &eposId, &mode ) )
    return NULL;
  
  eposNetwork.Sync();
  
  EPOS[ eposId ].VCS_SetOperationMode( (OperationMode) mode );
  
  return Py_BuildValue( "" );
}

static PyObject* readValues( PyObject *self, PyObject *args )
{
  int eposId;
  
  if ( !PyArg_ParseTuple( args, "i", &eposId ) )
    return NULL;
  
  eposNetwork.Sync();
  
  EPOS[ eposId ].ReadPDO01();
  EPOS[ eposId ].ReadPDO02();
  
  return Py_BuildValue( "" );
}

static PyObject* writeValues( PyObject *self, PyObject *args )
{
  int eposId;
  
  if ( !PyArg_ParseTuple( args, "i", &eposId ) )
    return NULL;
  
  EPOS[ eposId ].WritePDO01();
  EPOS[ eposId ].WritePDO02();
  
  return Py_BuildValue( "" );
}

static PyObject* setAxisEnabled( PyObject *self, PyObject *args )
{
  int eposId, enable;
  
  if ( !PyArg_ParseTuple( args, "ii", &eposId, &enable ) )
    return NULL;
  
  //cout << "Enabling/Disabling axis " << eposId << endl;
  
  if( enable )
    EPOS[ eposId ].Enable();
  else
    EPOS[ eposId ].Disable();
  
  return Py_BuildValue( "" );
}

static PyObject* setControl( PyObject *self, PyObject *args )
{
  int eposId, active, mode;
  
  if ( !PyArg_ParseTuple( args, "iii", &eposId, &active, &mode ) )
    return NULL;
  
  if( active > 0 ) run_thread( controlFunction( (Joint) mode ), (void*) &(EPOS[ eposId ]), DETACHED );
  
  return Py_BuildValue( "" );
}

static PyObject* getExecTime( PyObject *self, PyObject *args )
{
  if ( !PyArg_ParseTuple( args, "" ) )
    return NULL;
  
  return Py_BuildValue( "k", get_exec_time_milisseconds() );
}

// Names of the Python methods exported, with corresponding C function pointer and documentation
static PyMethodDef eposInterfaceMethods[] = {
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

static PyModuleDef eposInterfaceModule = {
    PyModuleDef_HEAD_INIT, "EposInterface", NULL, -1, eposInterfaceMethods,
    NULL, NULL, NULL, NULL
};

PyObject* initEposScriptInterface( void )
{
  // Start CAN network transmission
  for( int nodeId = 1; nodeId <= N_EPOS; nodeId++ )
    eposNetwork.StartPDOS( nodeId );
  
  return PyModule_Create( &eposInterfaceModule );
}

void endEposScriptInterface()
{
  // End CAN network transmission
  eposNetwork.StopPDOS( 1 );
  delay( 2000 );
}


#endif // SCRIPT_EPOS_NETWORK_H

    
    