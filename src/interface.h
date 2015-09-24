#ifndef INTERFACE_H
#define INTERFACE_H

#ifdef __cplusplus
    extern "C" {
#endif

#define INIT_INTERFACE_FILE( rtype, name, ... ) static rtype name( __VA_ARGS__ );
#define INIT_INTERFACE_POINTER( rtype, name, ... ) rtype (*name)( __VA_ARGS__ );
#define INIT_INTERFACE_STRUCT( rtype, name, ... ) .name = name,
      
#define INIT_NAMESPACE_FILE( rtype, namespace, name, ... ) static rtype namespace##_##name( __VA_ARGS__ );
#define INIT_NAMESPACE_POINTER( rtype, namespace, name, ... ) INIT_INTERFACE_POINTER( rtype, name, __VA_ARGS__ )        
#define INIT_NAMESPACE_STRUCT( rtype, namespace, name, ... ) .name = namespace##_##name,
      
#define INIT_INTERFACE( INTERFACE_FUNCTIONS, interfaceName ) \
  INTERFACE_FUNCTIONS( INIT_INTERFACE_FILE, interfaceName ) \
  const struct { \
      INTERFACE_FUNCTIONS( INIT_INTERFACE_POINTER, interfaceName ) \
    } \
    interfaceName = { INTERFACE_FUNCTIONS( INIT_INTERFACE_STRUCT, interfaceName ) };
      
/*#ifdef WIN32
  #include <ansi_c.h>
  #include <windows.h>

  #define LOAD_LIBRARY( libName ) LoadLibrary( libName )
  #define GET_LIBRARY_FUNCTION( libHandle, funcName ) GetProcAddress( libHandle, funcName )
  #define UNLOAD_LIBRARY( libHandle ) UnloadLibrary( libHandle )
#else
  #include <stdint.h>
      
  #define LOAD_LIBRARY( libName ) dlopen( libName, RTLD_NOW )
  #define GET_LIBRARY_FUNCTION( libHandle, funcName ) (intptr_t) dlsym( libHandle, funcName )
  #define UNLOAD_LIBRARY( libHandle ) dlclose( libHandle )    
#endif*/

#ifdef __cplusplus
    }
#endif

#endif  // INTERFACE_H
