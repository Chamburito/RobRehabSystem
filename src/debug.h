///////////////////////////////////////////////////////////////////////
/////             Debug and error handling utilities              /////
///////////////////////////////////////////////////////////////////////

#ifndef DEBUG_H
#define DEBUG_H

#include <stdio.h>
#include <stdlib.h>

#ifdef _CVI_
  #define CLEAR_SCREEN Cls()
#elif WIN32
  #include <Windows.h>
  #define CLEAR_SCREEN system( "cls" )
#else
  #define CLEAR_SCREEN system( "clear" )
#endif

#ifdef _MSC_VER
  #define INLINE __forceinline /* use __forceinline (VC++ specific) */
#else
  #define INLINE inline        /* use standard inline */
#endif

const size_t DEBUG_MESSAGE_LENGTH = 256;

#ifdef DEBUG
  #define DEBUG_PRINT( format, ... ) printf( "debug: %s: " format "\n", __func__, __VA_ARGS__ )
#else
  #define DEBUG_PRINT( format, ... ) do { } while( 0 )
#endif

#define ERROR_PRINT( format, ... ) fprintf( stderr, "error: %s: " format "\n", __func__, __VA_ARGS__ )
  
extern INLINE void print_platform_error( const char* message )
{
  #ifdef _CVI_
  #elif WIN32
  fprintf( stderr, "%s: code: %d\n", message, GetLastError() );
  #else
  perror( message );
  #endif
}

#endif // DEBUG_H
