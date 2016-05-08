#ifndef NAMESPACES_H
#define NAMESPACES_H

      
#define DECLARE_NAMESPACE_FUNCTION( rtype, namespace, func, ... ) static rtype namespace##_##func( __VA_ARGS__ );
#define DECLARE_NAMESPACE_FUNC_REF( rtype, namespace, func, ... ) rtype (*func)( __VA_ARGS__ );        
#define DEFINE_NAMESPACE_FUNC_REF( rtype, namespace, func, ... ) .func = namespace##_##func,
      
#define INIT_NAMESPACE_INTERFACE( namespace, functions ) \
        functions( namespace, DECLARE_NAMESPACE_FUNCTION ) \
        const struct { \
          functions( namespace, DECLARE_NAMESPACE_FUNC_REF ) \
        } \
        namespace = { functions( namespace, DEFINE_NAMESPACE_FUNC_REF ) };
        
#define DECLARE_NAMESPACE_INTERFACE( Namespace, INTERFACE ) \
        extern const struct Namespace { \
          INTERFACE( Namespace, DECLARE_NAMESPACE_FUNC_REF ) \
        } \
        Namespace;
        
#define DEFINE_NAMESPACE_INTERFACE( Namespace, INTERFACE ) \
        INTERFACE( Namespace, DECLARE_NAMESPACE_FUNCTION ) \
        const struct Namespace \
        Namespace = { INTERFACE( Namespace, DEFINE_NAMESPACE_FUNC_REF ) };

#endif  // NAMESPACES_H
