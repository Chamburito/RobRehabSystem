#ifndef SIGNAL_AQUISITION_TYPES_H
#define SIGNAL_AQUISITION_TYPES_H

#include "signal_aquisition/ni_daqmx/signal_aquisition.h"
//#include "signal_aquisition/power_daq/signal_aquisition.h"

typedef struct _SignalAquisitionType
{
  const char* name;
  SignalAquisitionInterface interface;
}
SignalAquisitionType;

const SignalAquisitionType SIGNAL_AQUISITION_TYPES_LIST[] = { { "NIDAQmx", &NIDAQmxOperations }/*, { "PowerDAQ", &PowerDAQOperations }*/ };

const size_t SIGNAL_AQUISITION_TYPES_NUMBER = sizeof(SIGNAL_AQUISITION_TYPES_LIST) / sizeof(SignalAquisitionType);

static SignalAquisitionInterface GetSignalAquisitionInterface( const char* interfaceName )
{
  for( size_t signalAquisitionTypeIndex = 0; signalAquisitionTypeIndex < SIGNAL_AQUISITION_TYPES_NUMBER; signalAquisitionTypeIndex++ )
  {
    if( strcmp( interfaceName, SIGNAL_AQUISITION_TYPES_LIST[ signalAquisitionTypeIndex ].name ) == 0 )
    {
      DEBUG_PRINT( "Found signal aquisition interface %s", interfaceName );
      return SIGNAL_AQUISITION_TYPES_LIST[ signalAquisitionTypeIndex ].interface;
    }
  }
  
  return NULL;
}

#endif // SIGNAL_AQUISITION_TYPES_H
