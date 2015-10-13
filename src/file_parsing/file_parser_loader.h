#ifndef FILE_PARSER_LOADER_H
#define FILE_PARSER_LOADER_H

#include "plugin_loader.h"
#include "file_parsing/file_parser_interface.h"

#include "debug/async_debug.h"

bool GetFileParserInterface( const char* interfaceFilePath, FileParserInterface* ref_interface )
{
  static char interfaceFilePathExt[ 256 ];
  
  sprintf( interfaceFilePathExt, "%s.%s", interfaceFilePath, PLUGIN_EXTENSION );
  
  DEBUG_PRINT( "trying to load plugin %s", interfaceFilePathExt );
  
  PLUGIN_HANDLE pluginHandle = LOAD_PLUGIN( interfaceFilePathExt );
  if( pluginHandle == NULL ) return false;
  
  DEBUG_PRINT( "found plugin %s (%p)", interfaceFilePathExt, pluginHandle );
  
  LOAD_PLUGIN_FUNCTIONS( FileParser, ref_interface )
  
  DEBUG_PRINT( "plugin %s loaded", interfaceFilePathExt );
  
  return true;
}

#endif // FILE_PARSER_LOADER_H
