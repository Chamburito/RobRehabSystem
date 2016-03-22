#ifndef CONFIG_PARSING_H
#define CONFIG_PARSING_H

#include "data_io/interface.h"
#include "plugin_loader.h"

#include "debug/async_debug.h"

#include <string.h>

static ParserInterface parser;
static bool pluginLoaded = false;

#define CONFIG_PARSING_FUNCTIONS( namespace, function_init ) \
        function_init( bool, namespace, Init, const char* ) \
        function_init( int, namespace, LoadConfigFile, const char* ) \
        function_init( int, namespace, LoadConfigString, const char* ) \
        function_init( ParserInterface, namespace, GetParser, void )

INIT_NAMESPACE_INTERFACE( ConfigParsing, CONFIG_PARSING_FUNCTIONS )

bool ConfigParsing_Init( const char* pluginName )
{
  char searchPath[ PARSER_MAX_FILE_PATH_LENGTH ];
  snprintf( searchPath, PARSER_MAX_FILE_PATH_LENGTH, "parsing/%s", pluginName );
  
  GET_PLUGIN_INTERFACE( PARSER_FUNCTIONS, searchPath, parser, pluginLoaded );
  
  return pluginLoaded;
}

inline int ConfigParsing_LoadConfigFile( const char* configFilePath )
{
  if( !pluginLoaded ) return PARSED_DATA_INVALID_ID;
  
  char filePath[ PARSER_MAX_FILE_PATH_LENGTH ];
  snprintf( filePath, PARSER_MAX_FILE_PATH_LENGTH, "config/%s", configFilePath );
  
  return parser.LoadFileData( filePath );
}

inline int ConfigParsing_LoadConfigString( const char* configString )
{
  if( !pluginLoaded ) return PARSED_DATA_INVALID_ID;
  
  return parser.LoadStringData( configString );
}

ParserInterface ConfigParsing_GetParser( void )
{
  return parser;
}

#endif // CONFIG_PARSING_H
