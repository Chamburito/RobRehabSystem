#ifndef INTERFACE_H
#define INTERFACE_H

#ifdef __cplusplus
    extern "C" {
#endif

#define INIT_INTERFACE_FILE( rtype, name, ... ) extern __declspec(dllexport) rtype name( __VA_ARGS__ );
#define INIT_INTERFACE_POINTER( rtype, name, ... ) rtype (*name)( __VA_ARGS__ );
#define INIT_INTERFACE_STRUCT( rtype, name, ... ) .name = name,
      
#define INIT_NAMESPACE_FILE( rtype, namespace, name, ... ) static rtype namespace##_##name( __VA_ARGS__ );
#define INIT_NAMESPACE_POINTER( rtype, namespace, name, ... ) rtype (*name)( __VA_ARGS__ );        
#define INIT_NAMESPACE_STRUCT( rtype, namespace, name, ... ) .name = namespace##_##name,

#define DEFINE_INTERFACE( interface ) \
        typedef struct { \
          interface( INIT_INTERFACE_POINTER ) \
        } \
        /*interface##Operations; \
        typedef interface##Operations**/ interface##Interface;
      
#define IMPLEMENT_INTERFACE( interface, implementation ) \
        interface( INIT_INTERFACE_FILE ) \
        const /*interface##Operations*/interface##Interface implementation = { interface( INIT_INTERFACE_STRUCT ) };
      
#define INIT_NAMESPACE_INTERFACE( namespace, functions ) \
        functions( namespace, INIT_NAMESPACE_FILE ) \
        const struct { \
          functions( namespace, INIT_NAMESPACE_POINTER ) \
        } \
        namespace = { functions( namespace, INIT_NAMESPACE_STRUCT ) };

#ifdef __cplusplus
    }
#endif

#endif  // INTERFACE_H
