#ifndef CONFIG_PARSER_H
#define CONFIG_PARSER_H

#include "parsing/parser_interface.h"
#include "plugin_loader.h"

#include "debug/async_debug.h"

#include <string.h>

static FileParserInterface parser;
static bool pluginLoaded = false;
static char searchPath[ PARSER_MAX_KEY_PATH_LENGTH ];

#define CONFIG_PARSER_FUNCTIONS( namespace, function_init ) \
        function_init( bool, namespace, Init, const char* ) \
        FILE_PARSER_FUNCTIONS( namespace, function_init )

INIT_NAMESPACE_INTERFACE( ConfigParser, CONFIG_PARSER_FUNCTIONS )

bool ConfigParser_Init( const char* pluginPath )
{
  snprintf( searchPath, PARSER_MAX_KEY_PATH_LENGTH, "file_parsing/%s", pluginPath );
  
  GET_PLUGIN_INTERFACE( FILE_PARSER_FUNCTIONS, searchPath, parser, pluginLoaded );
  
  return pluginLoaded;
}

int ConfigParser_LoadFile( const char* filePath )
{
  if( !pluginLoaded ) return PARSED_DATA_INVALID_ID;
  
  snprintf( searchPath, PARSER_MAX_KEY_PATH_LENGTH, "config/%s", filePath );

  return parser.LoadFile( searchPath );
}

void ConfigParser_UnloadFile( int fileID )
{
  if( !pluginLoaded ) return;

  parser.UnloadFile( fileID );
}

void ConfigParser_SetBaseKey( int fileID, const char* searchPath )
{
  if( !pluginLoaded ) return;

  parser.SetBaseKey( fileID, searchPath );
}

char* ConfigParser_GetStringValue( int fileID, const char* searchPath )
{
  if( !pluginLoaded ) return NULL;

  return parser.GetStringValue( fileID, searchPath );
}

long ConfigParser_GetIntegerValue( int fileID, const char* searchPath )
{
  if( !pluginLoaded ) return 0;

  return parser.GetIntegerValue( fileID, searchPath );
}

double ConfigParser_GetRealValue( int fileID, const char* searchPath )
{
  if( !pluginLoaded ) return 0.0;

  return parser.GetRealValue( fileID, searchPath );
}

bool ConfigParser_GetBooleanValue( int fileID, const char* searchPath )
{
  if( !pluginLoaded ) return false;

  return parser.GetBooleanValue( fileID, searchPath );
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
