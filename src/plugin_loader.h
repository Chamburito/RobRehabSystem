#ifndef PLUGIN_LOADER_H
#define PLUGIN_LOADER_H

#ifdef __cplusplus
    extern "C" {
#endif

#include "interface.h"
      
#ifdef WIN32
  //#include <ansi_c.h>
  #include <windows.h>

  #define LOAD_PLUGIN( pluginName ) LoadLibrary( pluginName )
  #define GET_PLUGIN_FUNCTION( pluginHandle, funcName ) GetProcAddress( pluginHandle, funcName )
  #define UNLOAD_PLUGIN( pluginHandle ) FreeLibrary( pluginHandle )
  
  #define PLUGIN_EXTENSION "dll"
#else
  #include <stdint.h>
      
  #define LOAD_PLUGIN( pluginName ) dlopen( pluginName, RTLD_NOW )
  #define GET_PLUGIN_FUNCTION( pluginHandle, funcName ) (intptr_t) dlsym( pluginHandle, funcName )
  #define UNLOAD_PLUGIN( pluginHandle ) dlclose( pluginHandle ) 
  
  #define PLUGIN_EXTENSION "so" 
#endif
  
#define FUNCTION_NAME( func ) # func
  
#define INIT_INTERFACE_SYMBOL( rtype, func, ... ) do { ref_interface -> func = (rtype (*)( __VA_ARGS__ )) GET_PLUGIN_FUNCTION( pluginHandle, FUNCTION_NAME( func ) ); \
                                                       printf( "loaded function %s (%p)\n", FUNCTION_NAME( func ), ref_interface -> func ); } while( 0 );
  
#define LOAD_PLUGIN_FUNCTIONS( interface, ref_interface ) interface( INIT_INTERFACE_SYMBOL )
  
  
/*bool IsPluginAvailable( const char* directoryName, const char* interfaceName )
{
  static char interfaceFileName[ 256 ];
  
  sprintf( interfaceFileName, "%s.%s", interfaceName, PLUGIN_EXTENSION );
  
  DIR* directory = opendir( directoryName );
  if( directory != NULL )
  {
    struct dirent directoryEntry;
    while( (directoryEntry = readdir( directory )) != NULL )
    {
      if( strcmp( directoryEntry->d_name, interfaceFileName ) == 0 )
      {
        DEBUG_PRINT( "found file %s", interfaceFilePath );
        return true;
      }
    }
    
    closedir( directory );
  }
  
  return false;
}*/

#ifdef __cplusplus
    }
#endif

#endif  // PLUGIN_LOADER_H
