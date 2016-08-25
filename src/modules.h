#ifndef MODULES_H
#define MODULES_H

#include <stdio.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>

#ifdef WIN32
  #include <windows.h>

  #define LOAD_PLUGIN( pluginName ) LoadLibrary( pluginName )
  #define LOAD_PLUGIN_SYMBOL( pluginHandle, funcName ) GetProcAddress( pluginHandle, funcName )
  #define UNLOAD_PLUGIN( pluginHandle ) FreeLibrary( pluginHandle )
  
  #define PLUGIN_HANDLE HINSTANCE
  #define PLUGIN_EXTENSION "dll"
#elif __unix__
  #include <stdint.h>
  #include <dlfcn.h>
  #include <unistd.h>
  #define __declspec(dllexport)
      
  #define LOAD_PLUGIN( pluginName ) dlopen( pluginName, RTLD_NOW )
  #define LOAD_PLUGIN_SYMBOL( pluginHandle, funcName ) ( (intptr_t) dlsym( pluginHandle, funcName ) )
  #define UNLOAD_PLUGIN( pluginHandle ) dlclose( pluginHandle ) 
  
  #define PLUGIN_HANDLE void*
  #define PLUGIN_EXTENSION "so" 
#endif

#ifdef __cplusplus 
  #define C_FUNCTION extern "C"
#else
  #define C_FUNCTION
#endif

  
#define FUNCTION_NAME( func ) # func


#define DECLARE_MODULE_FUNCTION( rtype, Namespace, func, ... ) C_FUNCTION __declspec(dllexport) rtype func( __VA_ARGS__ );
#define DECLARE_MODULE_FUNC_REF( rtype, Namespace, func, ... ) rtype (*func)( __VA_ARGS__ );
#define DEFINE_MODULE_FUNC_REF( rtype, Namespace, func, ... ) .func = func,

#define LOAD_PLUGIN_FUNCTION( rtype, implementationRef, func, ... ) implementationRef -> func = (rtype (*)( __VA_ARGS__ )) LOAD_PLUGIN_SYMBOL( pluginHandle, FUNCTION_NAME( func ) );

      
#define DECLARE_MODULE_INTERFACE( INTERFACE ) INTERFACE( NULL, DECLARE_MODULE_FUNCTION )
#define DECLARE_MODULE_INTERFACE_REF( INTERFACE ) INTERFACE( NULL, DECLARE_MODULE_FUNC_REF )
#define DEFINE_MODULE_INTERFACE_REF( INTERFACE ) INTERFACE( NULL, DEFINE_MODULE_FUNC_REF )

#define LOAD_PLUGIN_FUNCTIONS( INTERFACE, Module ) INTERFACE( Module, LOAD_PLUGIN_FUNCTION )
      
#define LOAD_MODULE_IMPLEMENTATION( INTERFACE, pluginPath, Module, success ) \
  do { char pluginPathExt[ 256 ]; \
  sprintf( pluginPathExt, "plugins/%s." PLUGIN_EXTENSION, pluginPath ); \
  printf( "\ttrying to load plugin %s\n", pluginPathExt ); \
  PLUGIN_HANDLE pluginHandle = LOAD_PLUGIN( pluginPathExt ); \
  printf( "\tplugin handle: %p\n", pluginHandle ); \
  if( success != NULL ) *success = (bool) pluginHandle; \
  LOAD_PLUGIN_FUNCTIONS( INTERFACE, Module ) } while( 0 )


#endif  // MODULES_H
