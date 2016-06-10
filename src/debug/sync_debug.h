///////////////////////////////////////////////////////////////////////
/////             Debug and error handling utilities              /////
///////////////////////////////////////////////////////////////////////

#ifndef DEBUG_H
#define DEBUG_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#ifdef _CVI_
  #define CLEAR_SCREEN Cls()
#elif WIN32
  //#include <Windows.h>
  #define CLEAR_SCREEN system( "cls" )
#else
  #define CLEAR_SCREEN system( "clear" )
#endif

//#ifdef _MSC_VER
//  #define INLINE __forceinline /* use __forceinline (VC++ specific) */
//#else
//  #define INLINE inline        /* use standard inline */
//#endif

#ifdef DEBUG
  #define DEBUG_PRINT( format, ... ) fprintf( stderr, "debug: %s: " format "\n", __func__, __VA_ARGS__ )
#else
  #define DEBUG_PRINT( format, ... ) do { } while( 0 )
#endif

#define ERROR_PRINT( format, ... ) fprintf( stderr, "error: %s: " format ": %s\n", __func__, __VA_ARGS__, strerror( errno ) )

#endif // DEBUG_H
