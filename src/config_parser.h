#ifndef CONFIG_PARSING_H
#define CONFIG_PARSING_H

#include "data_io/interface.h"
#include "plugin_loader.h"

#include "debug/async_debug.h"

#include <string.h>

typedef struct
{
  DECLARE_MODULE_INTERFACE_REF( DATA_IO_INTERFACE );
}
ConfigParser;
static ConfigParser parser;
static bool pluginLoaded = false;

#define CONFIG_PARSING_FUNCTIONS( namespace, function_init ) \
        function_init( bool, namespace, Init, const char* ) \
        function_init( int, namespace, LoadConfigFile, const char* ) \
        function_init( int, namespace, LoadConfigString, const char* ) \
        function_init( ConfigParser, namespace, GetParser, void )

INIT_NAMESPACE_INTERFACE( ConfigParsing, CONFIG_PARSING_FUNCTIONS )

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

#endif // CONFIG_PARSING_H
