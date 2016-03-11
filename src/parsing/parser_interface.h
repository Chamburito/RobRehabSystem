#ifndef PARSER_H
#define PARSER_H

#include "interfaces.h"

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

#include <stdarg.h>

#define PARSER_MAX_FILE_PATH_LENGTH 256
#define PARSER_MAX_KEY_PATH_LENGTH 256

const int PARSED_DATA_INVALID_ID = -1;

#define PARSER_FUNCTIONS( interface, function_init ) \
        function_init( int, interface, LoadFileData, const char* ) \
        function_init( int, interface, LoadStringData, const char* ) \
        function_init( void, interface, UnloadData, int ) \
        function_init( long, interface, GetIntegerValue, int, long, const char*, ... ) \
        function_init( double, interface, GetRealValue, int, double, const char*, ... ) \
        function_init( char*, interface, GetStringValue, int, char*, const char*, ... ) \
        function_init( bool, interface, GetBooleanValue, int, bool, const char*, ... ) \
        function_init( size_t, interface, GetListSize, int, const char*, ... ) \
        function_init( bool, interface, HasKey, int, const char*, ... )

DEFINE_INTERFACE( Parser, PARSER_FUNCTIONS )

#endif // PARSER_H
