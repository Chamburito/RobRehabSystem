#ifndef ACTUATOR_TYPES_H
#define ACTUATOR_TYPES_H

#include "series_elastic_actuator/actuator.h"
#include "dual_line_actuator/actuator.h"

typedef struct _ActuatorTypeData
{
  const char* name;
  AxisInterface interface;
}
ActuatorTypeData;

static const ActuatorTypeData ACTUATOR_TYPES_LIST[] = { { "SEA", &SEActuatorMethods }, { "DLA", &DLActuatorMethods } };
static const size_t ACTUATOR_TYPES_NUMBER = sizeof(ACTUATOR_TYPES_LIST) / sizeof(ActuatorTypeData);

static ActuatorInterface GetInterface( const char* typeName )
{
  for( size_t axisTypeIndex = 0; axisTypeIndex < AXIS_TYPES_NUMBER; axisTypeIndex++ )
  {
    if( strcmp( typeName, AXIS_TYPES_LIST[ axisTypeIndex ].name ) == 0 )
      return AXIS_TYPES_LIST[ axisTypeIndex ].interface;
  }
  
  return NULL;
}

const struct 
{
  AxisInterface (*GetInterface)( const char* );
}
ActuatorTypes = { .GetInterface = GetInterface };

#endif // ACTUATOR_TYPES_H
