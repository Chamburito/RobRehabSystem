#ifndef FILE_PARSER_H
#define FILE_PARSER_H

#include "interface.h"

#include <stdbool.h>

#ifdef _CVI_
  #include <utility.h>
  #define SET_PATH( dirPath ) SetDir( dirPath );
#else
  #define SET_PATH( dirPath ) system( "cd " dirPath );
#endif

#define FILE_PARSER_MAX_PATH_LENGTH 256

#define FileParser( function_init ) \
        function_init( int, LoadFile, const char* ) \
        function_init( void, UnloadFile, int ) \
        function_init( void, SetBaseKey, int, const char* ) \
        function_init( long, GetIntegerValue, int, const char* ) \
        function_init( double, GetRealValue, int, const char* ) \
        function_init( char*, GetStringValue, int, const char* ) \
        function_init( bool, GetBooleanValue, int, const char* ) \
        function_init( size_t, GetListSize, int, const char* ) \
        function_init( bool, HasKey, int, const char* )

DEFINE_INTERFACE( FileParser )

#endif // FILE_PARSER_H
