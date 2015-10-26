#ifndef CONFIG_PARSER_H
#define CONFIG_PARSER_H

#include "parsing/parser_interface.h"
#include "plugin_loader.h"

#include "debug/async_debug.h"

#include <string.h>

static ParserInterface parser;
static bool pluginLoaded = false;

#define CONFIG_PARSER_FUNCTIONS( namespace, function_init ) \
        function_init( bool, namespace, Init, const char* ) \
        PARSER_FUNCTIONS( namespace, function_init )

INIT_NAMESPACE_INTERFACE( ConfigParser, CONFIG_PARSER_FUNCTIONS )

bool ConfigParser_Init( const char* pluginPath )
{
  char searchPath[ PARSER_MAX_FILE_PATH_LENGTH ];
  snprintf( searchPath, PARSER_MAX_FILE_PATH_LENGTH, "parsing/%s", pluginPath );
  
  GET_PLUGIN_INTERFACE( PARSER_FUNCTIONS, searchPath, parser, pluginLoaded );
  
  return pluginLoaded;
}

int ConfigParser_LoadFileData( const char* filePath )
{
  if( !pluginLoaded ) return PARSED_DATA_INVALID_ID;
  
  char searchPath[ PARSER_MAX_KEY_PATH_LENGTH ];
  snprintf( searchPath, PARSER_MAX_FILE_PATH_LENGTH, "config/%s", filePath );

  return parser.LoadFileData( searchPath );
}

int ConfigParser_LoadStringData( const char* configString )
{
  if( !pluginLoaded ) return PARSED_DATA_INVALID_ID;

  return parser.LoadStringData( configString );
}

void ConfigParser_UnloadData( int fileID )
{
  if( !pluginLoaded ) return;

  parser.UnloadData( fileID );
}

void ConfigParser_SetBaseKey( int fileID, const char* searchPath )
{
  if( !pluginLoaded ) return;

  parser.SetBaseKey( fileID, searchPath );
}

char* ConfigParser_GetStringValue( int fileID, const char* searchPath, char* defaultValue )
{
  if( !pluginLoaded ) return NULL;

  return parser.GetStringValue( fileID, searchPath, defaultValue );
}

long ConfigParser_GetIntegerValue( int fileID, const char* searchPath, long defaultValue )
{
  if( !pluginLoaded ) return 0;

  return parser.GetIntegerValue( fileID, searchPath, defaultValue );
}

double ConfigParser_GetRealValue( int fileID, const char* searchPath, double defaultValue )
{
  if( !pluginLoaded ) return 0.0;

  return parser.GetRealValue( fileID, searchPath, defaultValue );
}

bool ConfigParser_GetBooleanValue( int fileID, const char* searchPath, bool defaultValue )
{
  if( !pluginLoaded ) return false;

  return parser.GetBooleanValue( fileID, searchPath, defaultValue );
}

size_t ConfigParser_GetListSize( int fileID, const char* searchPath )
{
  if( !pluginLoaded ) return 0;

  return parser.GetListSize( fileID, searchPath );
}

bool ConfigParser_HasKey( int fileID, const char* searchPath )
{
  if( !pluginLoaded ) return false;

  return parser.HasKey( fileID, searchPath );
}

#endif // JSON_PARSER_H
