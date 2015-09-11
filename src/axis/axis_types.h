#ifndef AXIS_TYPES_H
#define AXIS_TYPES_H

#include "ni_can_epos_axis/axis.h"
//#include "pci_daq_axis/axis.h"

typedef struct _AxisTypeData
{
  const char* name;
  AxisInterface interface;
}
AxisTypeData;

static const AxisTypeData AXIS_TYPES_LIST[] = { { "CAN_EPOS", &AxisCANEPOSOperations }/*, { "PCI_DAQ", &AxisPCIDAQOperations }*/ };
static const size_t AXIS_TYPES_NUMBER = sizeof(AXIS_TYPES_LIST) / sizeof(AxisTypeData);

static AxisInterface GetAxisInterface( const char* typeName )
{
  for( size_t axisTypeIndex = 0; axisTypeIndex < AXIS_TYPES_NUMBER; axisTypeIndex++ )
  {
    if( strcmp( typeName, AXIS_TYPES_LIST[ axisTypeIndex ].name ) == 0 )
      return AXIS_TYPES_LIST[ axisTypeIndex ].interface;
  }
  
  return NULL;
}

#endif // AXIS_TYPES_H
