#ifndef INTERFACES_H
#define INTERFACES_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#ifdef __unix__
  #include <unistd.h>
  #define __declspec(dllexport)
#endif

#ifdef __cplusplus 
  #define C_FUNCTION extern "C"
#else
  #define C_FUNCTION
#endif

#define DEFINE_INTERFACE_FUNCTION( rtype, interface, func, ... ) C_FUNCTION __declspec(dllexport) rtype func( __VA_ARGS__ );
#define DEFINE_INTERFACE_FUNC_PTR( rtype, interface, func, ... ) rtype (*func)( __VA_ARGS__ );
#define INIT_INTERFACE_FUNC_PTR( rtype, interface, func, ... ) .func = func,
      
#define DEFINE_NAMESPACE_FUNCTION( rtype, namespace, func, ... ) static rtype namespace##_##func( __VA_ARGS__ );
#define DEFINE_NAMESPACE_FUNC_PTR( rtype, namespace, func, ... ) rtype (*func)( __VA_ARGS__ );        
#define INIT_NAMESPACE_FUNC_PTR( rtype, namespace, func, ... ) .func = namespace##_##func,


#define DEFINE_INTERFACE( interfaceFunctions ) interfaceFunctions( NULL, DEFINE_INTERFACE_FUNCTION )
#define DEFINE_INTERFACE_REF( interfaceFunctions ) interfaceFunctions( NULL, DEFINE_INTERFACE_FUNC_PTR )

#define DEFINE_INTERFACE_MODULE( interfaceName, interfaceFunctions ) \
        typedef struct { \
          interfaceFunctions( interfaceName, DEFINE_INTERFACE_FUNC_PTR ) \
        } \
        interfaceName##Interface;
      
#define INIT_NAMESPACE_INTERFACE( namespace, functions ) \
        functions( namespace, DEFINE_NAMESPACE_FUNCTION ) \
        const struct { \
          functions( namespace, DEFINE_NAMESPACE_FUNC_PTR ) \
        } \
        namespace = { functions( namespace, INIT_NAMESPACE_FUNC_PTR ) };

#endif  // INTERFACES_H
