#include <string.h>

#include "debug/sync_debug.h"

#include "config_parser.h"


DECLARE_MODULE_INTERFACE( DATA_IO_INTERFACE )
static ConfigParserImplementation parser = { DEFINE_MODULE_INTERFACE_REF( DATA_IO_INTERFACE ) };

DEFINE_NAMESPACE_INTERFACE( ConfigParsing, CONFIG_PARSING_INTERFACE )


bool ConfigParsing_Init( const char* pluginName )
{
  char searchPath[ DATA_IO_MAX_FILE_PATH_LENGTH ];
  snprintf( searchPath, DATA_IO_MAX_FILE_PATH_LENGTH, "data_io/%s", pluginName );
  
  bool pluginLoaded = false;
  LOAD_MODULE_IMPLEMENTATION( DATA_IO_INTERFACE, searchPath, (&parser), (&pluginLoaded) );
  
  return pluginLoaded;
}

int ConfigParsing_LoadConfigFile( const char* configFilePath )
{  
  char filePath[ DATA_IO_MAX_FILE_PATH_LENGTH ];
  snprintf( filePath, DATA_IO_MAX_FILE_PATH_LENGTH, "config/%s", configFilePath );
  
  DEBUG_PRINT( "looking for config file %s", configFilePath );
  
  return parser.LoadFileData( filePath );
}

inline int ConfigParsing_LoadConfigString( const char* configString )
{  
  return parser.LoadStringData( configString );
}

ConfigParser ConfigParsing_GetParser( void )
{
  return &parser;
}


int LoadStringData( const char* configString ) { return DATA_INVALID_ID; }
int LoadFileData( const char* filePath ) { return DATA_INVALID_ID; }
void UnloadData( int dataID ) { return; }
char* GetStringValue( int dataID, char* defaultValue, const char* pathFormat, ... ) { return defaultValue; }
long GetIntegerValue( int dataID, long defaultValue, const char* pathFormat, ... ) { return defaultValue; }
double GetRealValue( int dataID, double defaultValue, const char* pathFormat, ... ) { return defaultValue; }
bool GetBooleanValue( int dataID, bool defaultValue, const char* pathFormat, ... ) { return defaultValue; }
size_t GetListSize( int dataID, const char* pathFormat, ... ) { return 0; }
bool HasKey( int dataID, const char* pathFormat, ... ) { return false; }
