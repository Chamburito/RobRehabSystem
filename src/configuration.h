#ifndef CONFIG_PARSING_H
#define CONFIG_PARSING_H

#include "namespaces.h"
#include "modules.h"

#include "data_io/interface.h"


typedef struct
{
  DECLARE_MODULE_INTERFACE_REF( DATA_IO_INTERFACE );
}
DataIOImplementation;
typedef DataIOImplementation* DataIOHandler;
//static DataIOHandler parser;

#define CONFIGURATION_INTERFACE( Namespace, INIT_FUNCTION ) \
        INIT_FUNCTION( bool, Namespace, Init, const char* ) \
        INIT_FUNCTION( void, Namespace, SetBaseDirectory, const char* ) \
        INIT_FUNCTION( int, Namespace, LoadConfigFile, const char* ) \
        INIT_FUNCTION( int, Namespace, ParseConfigString, const char* ) \
        INIT_FUNCTION( DataIOHandler, Namespace, GetIOHandler, void )

DECLARE_NAMESPACE_INTERFACE( Configuration, CONFIGURATION_INTERFACE )


#endif // CONFIG_PARSING_H
