#ifndef CONFIG_PARSING_H
#define CONFIG_PARSING_H

#include "interfaces.h"

#include "data_io/interface.h"


typedef struct
{
  DECLARE_MODULE_INTERFACE_REF( DATA_IO_INTERFACE );
}
ConfigParser;
static ConfigParser parser;

#define CONFIG_PARSING_INTERFACE( Namespace, INIT_FUNCTION ) \
        INIT_FUNCTION( bool, Namespace, Init, const char* ) \
        INIT_FUNCTION( int, Namespace, LoadConfigFile, const char* ) \
        INIT_FUNCTION( int, Namespace, LoadConfigString, const char* ) \
        INIT_FUNCTION( ConfigParser, Namespace, GetParser, void )

DECLARE_NAMESPACE_INTERFACE( ConfigParsing, CONFIG_PARSING_INTERFACE )


#endif // CONFIG_PARSING_H
