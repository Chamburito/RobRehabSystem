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
  
#if __STDC_VERSION__ >= 199901L
  /* "inline" is a keyword */
#else
# define inline //static
#endif

inline void print_platform_error( const char* message )
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