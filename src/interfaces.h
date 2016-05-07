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


#define DECLARE_MODULE_FUNCTION( rtype, Namespace, func, ... ) C_FUNCTION __declspec(dllexport) rtype func( __VA_ARGS__ );
#define DECLARE_MODULE_FUNC_REF( rtype, Namespace, func, ... ) rtype (*func)( __VA_ARGS__ );
#define DEFINE_MODULE_FUNC_REF( rtype, Namespace, func, ... ) .func = func,
      
#define DECLARE_MODULE_INTERFACE( INTERFACE ) INTERFACE( NULL, DECLARE_MODULE_FUNCTION )
#define DECLARE_MODULE_INTERFACE_REF( INTERFACE ) INTERFACE( NULL, DECLARE_MODULE_FUNC_REF )

      
#define DECLARE_NAMESPACE_FUNCTION( rtype, namespace, func, ... ) rtype namespace##_##func( __VA_ARGS__ );
#define DECLARE_NAMESPACE_FUNC_REF( rtype, namespace, func, ... ) rtype (*func)( __VA_ARGS__ );        
#define DEFINE_NAMESPACE_FUNC_REF( rtype, namespace, func, ... ) .func = namespace##_##func,
      
#define INIT_NAMESPACE_INTERFACE( namespace, functions ) \
        functions( namespace, DECLARE_NAMESPACE_FUNCTION ) \
        const struct { \
          functions( namespace, DECLARE_NAMESPACE_FUNC_REF ) \
        } \
        namespace = { functions( namespace, DEFINE_NAMESPACE_FUNC_REF ) };
        
#define DECLARE_NAMESPACE_INTERFACE( Namespace, INTERFACE ) \
        INTERFACE( Namespace, DECLARE_NAMESPACE_FUNCTION ) \
        extern const struct Namespace { \
          INTERFACE( Namespace, DECLARE_NAMESPACE_FUNC_REF ) \
        } \
        Namespace;
        
#define DEFINE_NAMESPACE_INTERFACE( Namespace, INTERFACE ) \
        const struct Namespace \
        Namespace = { INTERFACE( Namespace, DEFINE_NAMESPACE_FUNC_REF ) };

#endif  // INTERFACES_H
