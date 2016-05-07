#ifndef DATA_IO_H
#define DATA_IO_H

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

#define DATA_IO_MAX_FILE_PATH_LENGTH 256
#define DATA_IO_MAX_KEY_PATH_LENGTH 256
#define DATA_IO_MAX_VALUE_LENGTH 128

const int DATA_INVALID_ID = -1;

#define DATA_IO_INTERFACE( Interface, INIT_FUNCTION ) \
        INIT_FUNCTION( int, Interface, LoadFileData, const char* ) \
        INIT_FUNCTION( int, Interface, LoadStringData, const char* ) \
        INIT_FUNCTION( void, Interface, UnloadData, int ) \
        INIT_FUNCTION( long, Interface, GetIntegerValue, int, long, const char*, ... ) \
        INIT_FUNCTION( double, Interface, GetRealValue, int, double, const char*, ... ) \
        INIT_FUNCTION( char*, Interface, GetStringValue, int, char*, const char*, ... ) \
        INIT_FUNCTION( bool, Interface, GetBooleanValue, int, bool, const char*, ... ) \
        INIT_FUNCTION( size_t, Interface, GetListSize, int, const char*, ... ) \
        INIT_FUNCTION( bool, Interface, HasKey, int, const char*, ... )

#endif // DATA_IO_H
