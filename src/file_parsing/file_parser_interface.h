#ifndef FILE_PARSER_H
#define FILE_PARSER_H

#include "interface.h"

#include <stdbool.h>

#ifdef _CVI_
  #include <windows.h>
  #include <utility.h>
  #define SET_PATH( dirPath ) SetDir( dirPath );
#elif WIN32
  #include <direct.h>
  #define SET_PATH( dirPath ) _chdir( dirPath );
#else
  #include <unistd.h>
  #define SET_PATH( dirPath ) chdir( dirPath );
#endif

#define FILE_PARSER_MAX_PATH_LENGTH 256

const int INVALID_FILE_ID = -1;

#define FILE_PARSER_FUNCTIONS( interface, function_init ) \
        function_init( int, interface, LoadFile, const char* ) \
        function_init( void, interface, UnloadFile, int ) \
        function_init( void, interface, SetBaseKey, int, const char* ) \
        function_init( long, interface, GetIntegerValue, int, const char* ) \
        function_init( double, interface, GetRealValue, int, const char* ) \
        function_init( char*, interface, GetStringValue, int, const char* ) \
        function_init( bool, interface, GetBooleanValue, int, const char* ) \
        function_init( size_t, interface, GetListSize, int, const char* ) \
        function_init( bool, interface, HasKey, int, const char* )

DEFINE_INTERFACE( FileParser, FILE_PARSER_FUNCTIONS )

#endif // FILE_PARSER_H
