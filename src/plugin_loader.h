#ifndef PLUGIN_LOADER_H
#define PLUGIN_LOADER_H

#ifdef __cplusplus
    extern "C" {
#endif

//#include "interface.h"
      
#ifdef WIN32
  #include <windows.h>

  #define LOAD_PLUGIN( pluginName ) LoadLibrary( pluginName )
  #define GET_PLUGIN_FUNCTION( pluginHandle, funcName ) GetProcAddress( pluginHandle, funcName )
  #define UNLOAD_PLUGIN( pluginHandle ) FreeLibrary( pluginHandle )
  
  #define PLUGIN_HANDLE HINSTANCE
  #define PLUGIN_EXTENSION "dll"
#else
  #include <stdint.h>
  #include <dlfcn.h>
      
  #define LOAD_PLUGIN( pluginName ) dlopen( pluginName, RTLD_NOW )
  #define GET_PLUGIN_FUNCTION( pluginHandle, funcName ) (intptr_t) dlsym( pluginHandle, funcName )
  #define UNLOAD_PLUGIN( pluginHandle ) dlclose( pluginHandle ) 
  
  #define PLUGIN_HANDLE void*
  #define PLUGIN_EXTENSION "so" 
#endif
      
#include <stdio.h>
#include <stdbool.h>
  
#define FUNCTION_NAME( func ) # func
  
#define INIT_INTERFACE_SYMBOL( rtype, interfaceStruct, func, ... ) interfaceStruct . func = (rtype (*)( __VA_ARGS__ )) GET_PLUGIN_FUNCTION( pluginHandle, FUNCTION_NAME( func ) );
  
#define LOAD_PLUGIN_FUNCTIONS( interfaceFunctions, interfaceName ) interfaceFunctions( interfaceName, INIT_INTERFACE_SYMBOL )
      
#define GET_PLUGIN_INTERFACE( interfaceFunctions, interfaceFilePath, interfaceName, success ) \
  do { char interfaceFilePathExt[ 256 ]; \
  sprintf( interfaceFilePathExt, "plugins/%s.%s", interfaceFilePath, PLUGIN_EXTENSION ); \
  printf( "trying to load plugin %s\n", interfaceFilePathExt ); \
  PLUGIN_HANDLE pluginHandle = LOAD_PLUGIN( interfaceFilePathExt ); \
  success = (bool) pluginHandle; \
  LOAD_PLUGIN_FUNCTIONS( interfaceFunctions, interfaceName ) } while( 0 )

#ifdef __cplusplus
    }
#endif

#endif  // PLUGIN_LOADER_H
