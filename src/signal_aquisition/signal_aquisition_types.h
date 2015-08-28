#ifndef SIGNAL_AQUISITION_TYPES_H
#define SIGNAL_AQUISITION_TYPES_H

#include "signal_aquisition/ni_daqmx/signal_aquisition.h"

typedef struct _SignalAquisitionType
{
  const char* name;
  SignalAquisitionInterface interface;
}
SignalAquisitionType;

const SignalAquisitionType SIGNAL_AQUISITION_TYPES_LIST = { { "NIDAQmx", &NIDAQmxOperations } };

const size_t SIGNAL_AQUISITION_TYPES_NUMBER = sizeof(SIGNAL_AQUISITION_TYPES_LIST) / sizeof(SignalAquisitionType);

static SignalAquisitionInterface GetInterface( const char* interfaceName )
{
  for( size_t signalAquisitionTypeIndex = 0; signalAquisitionTypeIndex < SIGNAL_AQUISITION_TYPES_NUMBER; signalAquisitionTypeIndex++ )
  {
    if( strcmp( interfaceName, SIGNAL_AQUISITION_TYPES_LIST[ signalAquisitionTypeIndex ].name ) == 0 )
      return SIGNAL_AQUISITION_TYPES_LIST[ signalAquisitionTypeIndex ].interface;
  }
  
  return NULL;
}

const struct
{
  SignalAquisitionInterface (*GetInterface)( const char* );
}
SignalAquisitionTypes = { .GetInterface = GetInterface };

#endif // SIGNAL_AQUISITION_TYPES_H
