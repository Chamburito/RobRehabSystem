#ifndef ACTUATOR_TYPES_H
#define ACTUATOR_TYPES_H

#include "series_elastic_actuator/actuator.h"
#include "dual_line_actuator/actuator.h"

typedef struct _ActuatorTypeData
{
  const char* name;
  ActuatorInterface interface;
}
ActuatorTypeData;

static const ActuatorTypeData ACTUATOR_TYPES_LIST[] = { { "SEA", &SEActuatorOperations }, { "DLA", &DLActuatorOperations } };
static const size_t ACTUATOR_TYPES_NUMBER = sizeof(ACTUATOR_TYPES_LIST) / sizeof(ActuatorTypeData);

static ActuatorInterface GetInterface( const char* typeName )
{
  for( size_t actuatorTypeIndex = 0; actuatorTypeIndex < AXIS_TYPES_NUMBER; actuatorTypeIndex++ )
  {
    if( strcmp( typeName, AXIS_TYPES_LIST[ actuatorTypeIndex ].name ) == 0 )
      return AXIS_TYPES_LIST[ actuatorTypeIndex ].interface;
  }
  
  return NULL;
}

const struct 
{
  ActuatorInterface (*GetInterface)( const char* );
}
ActuatorTypes = { .GetInterface = GetInterface };

#endif // ACTUATOR_TYPES_H
