///////////////////////////////////////////////////////////////////////
/////                  Error handling utilities                   /////
///////////////////////////////////////////////////////////////////////

#ifndef ERROR_H
#define ERROR_H

#ifdef __cplusplus
extern "C"{
#endif

#include <stdio.h>
#include <stdlib.h>
  
#ifdef _MSC_VER
  #define INLINE __forceinline /* use __forceinline (VC++ specific) */
#else
  #define INLINE inline        /* use standard inline */
#endif

extern INLINE void print_platform_error( const char* message )
{
  #ifdef WIN32
  fprintf( stderr, "%s: code: %d\n", message, GetLastError() );
  #else
  perror( message );
  #endif
}



#ifdef __cplusplus
}
#endif

#endif // ERROR_H