#include "actuator_control/interface.h"

#include <string.h>

typedef struct _ControlData
{
  ControlVariablesList outputsList;
}
ControlData;

DECLARE_MODULE_INTERFACE( ACTUATOR_CONTROL_INTERFACE )

Controller InitController( void )
{
  void* newController = malloc( sizeof(ControlData) );
  memset( newController, 0, sizeof(ControlData) );
  
  return (Controller) newController;
}

double* RunControlStep( Controller controller, double* measuresList, double* setpointsList, double* ref_error )
{ 
  if( controller == NULL ) return NULL;
  
  ControlData* controlData = (ControlData*) controller;

  controlData->outputsList[ CONTROL_POSITION ] = setpointsList[ CONTROL_POSITION ];
  controlData->outputsList[ CONTROL_VELOCITY ] = setpointsList[ CONTROL_VELOCITY ];
  controlData->outputsList[ CONTROL_FORCE ] = setpointsList[ CONTROL_FORCE ];
  controlData->outputsList[ CONTROL_ACCELERATION ] = setpointsList[ CONTROL_ACCELERATION ];

  return (double*) controlData->outputsList;
}

void EndController( Controller controller )
{
  free( (void*) controller );
}
