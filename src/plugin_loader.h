#ifndef PLUGIN_LOADER_H
#define PLUGIN_LOADER_H


#ifdef WIN32
  #include <windows.h>

  #define LOAD_PLUGIN( pluginName ) LoadLibrary( pluginName )
  #define LOAD_PLUGIN_SYMBOL( pluginHandle, funcName ) GetProcAddress( pluginHandle, funcName )
  #define UNLOAD_PLUGIN( pluginHandle ) FreeLibrary( pluginHandle )
  
  #define PLUGIN_HANDLE HINSTANCE
  #define PLUGIN_EXTENSION "dll"
#else
  #include <stdint.h>
  #include <dlfcn.h>
      
  #define LOAD_PLUGIN( pluginName ) dlopen( pluginName, RTLD_NOW )
  #define LOAD_PLUGIN_SYMBOL( pluginHandle, funcName ) (intptr_t) dlsym( pluginHandle, funcName )
  #define UNLOAD_PLUGIN( pluginHandle ) dlclose( pluginHandle ) 
  
  #define PLUGIN_HANDLE void*
  #define PLUGIN_EXTENSION "so" 
#endif
      
#include <stdio.h>
#include <stdbool.h>
  
#define FUNCTION_NAME( func ) # func

#define LOAD_PLUGIN_FUNCTION( rtype, implementationRef, func, ... ) implementationRef -> func = (rtype (*)( __VA_ARGS__ )) LOAD_PLUGIN_SYMBOL( pluginHandle, FUNCTION_NAME( func ) );

#define LOAD_PLUGIN_FUNCTIONS( INTERFACE, Module ) INTERFACE( Module, LOAD_PLUGIN_FUNCTION )
      
#define GET_PLUGIN_IMPLEMENTATION( INTERFACE, pluginPath, Module, success ) \
  do { char pluginPathExt[ 256 ]; \
  sprintf( pluginPathExt, "plugins/%s." PLUGIN_EXTENSION, pluginPath ); \
  printf( "trying to load plugin %s\n", pluginPathExt ); \
  PLUGIN_HANDLE pluginHandle = LOAD_PLUGIN( pluginPathExt ); \
  printf( "plugin handle: %p\n", pluginHandle ); \
  if( success != NULL ) *success = (bool) pluginHandle; \
  LOAD_PLUGIN_FUNCTIONS( INTERFACE, Module ) } while( 0 )


#endif  // PLUGIN_LOADER_H
