#ifndef SIGNAL_AQUISITION_INTERFACE_H
#define SIGNAL_AQUISITION_INTERFACE_H

#include "interface.h"

#include <stdbool.h>

/*typedef struct _SignalAquisitionOperations
{
  int (*InitTask)( const char* );
  void (*EndTask)( int );
  double* (*Read)( int, unsigned int, size_t* );
  bool (*AquireChannel)( int, unsigned int );
  void (*ReleaseChannel)( int, unsigned int );
  size_t (*GetMaxSamplesNumber)( int );
}
SignalAquisitionOperations;

typedef SignalAquisitionOperations* SignalAquisitionInterface;*/

#define SignalAquisition( function_init ) \
        function_init( int, InitTask, const char* ) \
        function_init( void, EndTask, int ) \
        function_init( double*, Read, int, unsigned int, size_t* ) \
        function_init( bool, AquireChannel, int, unsigned int ) \
        function_init( void, ReleaseChannel, int, unsigned int ) \
        function_init( size_t, GetMaxSamplesNumber, int )

DEFINE_INTERFACE( SignalAquisition )

#endif // SIGNAL_AQUISITION_INTERFACE_H 
