#ifndef INTERFACES_H
#define INTERFACES_H

#include "debug/debug.h"

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#ifdef __unix__
  #include <unistd.h>
#endif

#define INIT_INTERFACE_FILE( rtype, interface, func, ... ) extern __declspec(dllexport) rtype func( __VA_ARGS__ );
#define INIT_INTERFACE_POINTER( rtype, interface, func, ... ) rtype (*func)( __VA_ARGS__ );
#define INIT_INTERFACE_STRUCT( rtype, interface, func, ... ) .func = func,
      
#define INIT_NAMESPACE_FILE( rtype, namespace, func, ... ) static rtype namespace##_##func( __VA_ARGS__ );
#define INIT_NAMESPACE_POINTER( rtype, namespace, func, ... ) rtype (*func)( __VA_ARGS__ );        
#define INIT_NAMESPACE_STRUCT( rtype, namespace, func, ... ) .func = namespace##_##func,

#define DEFINE_INTERFACE( interfaceName, interfaceFunctions ) \
        typedef struct { \
          interfaceFunctions( interfaceName, INIT_INTERFACE_POINTER ) \
        } \
        interfaceName##Interface;
      
#define IMPLEMENT_INTERFACE( interfaceFunctions ) interfaceFunctions( interfaceName, INIT_INTERFACE_FILE )
      
#define INIT_NAMESPACE_INTERFACE( namespace, functions ) \
        functions( namespace, INIT_NAMESPACE_FILE ) \
        const struct { \
          functions( namespace, INIT_NAMESPACE_POINTER ) \
        } \
        namespace = { functions( namespace, INIT_NAMESPACE_STRUCT ) };

#endif  // INTERFACES_H
