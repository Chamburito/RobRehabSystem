#ifndef PLUGIN_LOADER_H
#define PLUGIN_LOADER_H


#ifdef WIN32
  #include <windows.h>

  #define LOAD_PLUGIN( pluginName ) LoadLibrary( pluginName )
  #define LOAD_PLUGIN_SYMBOL( pluginHandle, funcName ) GetProcAddress( pluginHandle, funcName )
  #define UNLOAD_PLUGIN( pluginHandle ) FreeLibrary( pluginHandle )
  
  #define PLUGIN_HANDLE HINSTANCE
  //#define PLUGIN_PREFIX ""
  #define PLUGIN_EXTENSION "dll"
#else
  #include <stdint.h>
  #include <dlfcn.h>
      
  #define LOAD_PLUGIN( pluginName ) dlopen( pluginName, RTLD_NOW )
  #define LOAD_PLUGIN_SYMBOL( pluginHandle, funcName ) (intptr_t) dlsym( pluginHandle, funcName )
  #define UNLOAD_PLUGIN( pluginHandle ) dlclose( pluginHandle ) 
  
  #define PLUGIN_HANDLE void*
  //#define PLUGIN_PREFIX "lib"
  #define PLUGIN_EXTENSION "so" 
#endif
      
#include <stdio.h>
#include <stdbool.h>
  
#define FUNCTION_NAME( func ) # func

#define LOAD_PLUGIN_FUNCTION( rtype, interface, func, ... ) interface -> func = (rtype (*)( __VA_ARGS__ )) LOAD_PLUGIN_SYMBOL( pluginHandle, FUNCTION_NAME( func ) );

#define LOAD_PLUGIN_FUNCTIONS( interfaceFunctions, interfaceName ) interfaceFunctions( interfaceName, LOAD_PLUGIN_FUNCTION )
      
#define GET_PLUGIN_IMPLEMENTATION( interfaceFunctions, interfaceFilePath, interfaceName, success ) \
  do { char interfaceFilePathExt[ 256 ]; \
  sprintf( interfaceFilePathExt, "plugins/%s." PLUGIN_EXTENSION, interfaceFilePath ); \
  printf( "trying to load plugin %s\n", interfaceFilePathExt ); \
  PLUGIN_HANDLE pluginHandle = LOAD_PLUGIN( interfaceFilePathExt ); \
  if( success != NULL ) *success = (bool) pluginHandle; \
  LOAD_PLUGIN_FUNCTIONS( interfaceFunctions, interfaceName ) } while( 0 )


#endif  // PLUGIN_LOADER_H
