#ifndef ROBREHAB_SUBSYSTEM_H
#define ROBREHAB_SUBSYSTEM_H

#include "namespaces.h"


extern const unsigned long UPDATE_INTERVAL_MS;

#define ROBREHAB_SUBSYSTEM_INTERFACE( Namespace, INIT_FUNCTION ) \
        INIT_FUNCTION( int, Namespace, Init, const char* ) \
        INIT_FUNCTION( void, Namespace, End, void ) \
        INIT_FUNCTION( void, Namespace, Update, void )

DECLARE_NAMESPACE_INTERFACE( SubSystem, ROBREHAB_SUBSYSTEM_INTERFACE )

#endif // ROBREHAB_SUBSYSTEM_H
