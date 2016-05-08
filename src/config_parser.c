#include <string.h>

#include "plugin_loader.h"

#include "debug/sync_debug.h"

#include "config_parser.h"

static bool pluginLoaded = false;


DEFINE_NAMESPACE_INTERFACE( ConfigParsing, CONFIG_PARSING_INTERFACE )


bool ConfigParsing_Init( const char* pluginName )
{
  char searchPath[ DATA_IO_MAX_FILE_PATH_LENGTH ];
  snprintf( searchPath, DATA_IO_MAX_FILE_PATH_LENGTH, "data_io/%s", pluginName );
  
  GET_PLUGIN_IMPLEMENTATION( DATA_IO_INTERFACE, searchPath, (&parser), (&pluginLoaded) );
  
  return pluginLoaded;
}

int ConfigParsing_LoadConfigFile( const char* configFilePath )
{
  if( !pluginLoaded ) return DATA_INVALID_ID;
  
  char filePath[ DATA_IO_MAX_FILE_PATH_LENGTH ];
  snprintf( filePath, DATA_IO_MAX_FILE_PATH_LENGTH, "config/%s", configFilePath );
  
  DEBUG_PRINT( "looking for config file %s", configFilePath );
  
  return parser.LoadFileData( filePath );
}

inline int ConfigParsing_LoadConfigString( const char* configString )
{
  if( !pluginLoaded ) return DATA_INVALID_ID;
  
  return parser.LoadStringData( configString );
}

ConfigParser ConfigParsing_GetParser( void )
{
  return parser;
}
