#ifndef SIGNAL_AQUISITION_TYPES_H
#define SIGNAL_AQUISITION_TYPES_H

#include "plugin_loader.h"
#include "signal_aquisition/signal_aquisition_interface.h"

#include "debug/async_debug.h"

bool GetSignalAquisitionInterface( const char* interfaceFilePath, SignalAquisitionInterface* ref_interface )
{
  static char interfaceFilePathExt[ 256 ];
  
  sprintf( interfaceFilePathExt, "../plugins/signal_aquisition/%s.%s", interfaceFilePath, PLUGIN_EXTENSION );
  
  DEBUG_PRINT( "trying to load plugin %s", interfaceFilePathExt );
  
  HINSTANCE pluginHandle = LOAD_PLUGIN( interfaceFilePathExt );
  if( pluginHandle == NULL ) return false;
  
  DEBUG_PRINT( "found plugin %s (%p)", interfaceFilePathExt, pluginHandle );
  
  LOAD_PLUGIN_FUNCTIONS( SignalAquisition, ref_interface )
  //ref_interface->InitTask = (int (*)( const char* )) GET_PLUGIN_FUNCTION( pluginHandle, FUNCTION_NAME( InitTask ) );
  //printf( "loaded function %s (%p)\n", FUNCTION_NAME( InitTask ), ref_interface->InitTask );
  
  DEBUG_PRINT( "plugin %s loaded", interfaceFilePathExt );
  
  return true;
}

#endif // SIGNAL_AQUISITION_TYPES_H
