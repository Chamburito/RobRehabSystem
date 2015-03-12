///////////////////////////////////////////////////////////////////////
/////             Debug and error handling utilities              /////
///////////////////////////////////////////////////////////////////////

#ifndef ERROR_H
#define ERROR_H

#ifdef _LINK_CVI_LVRT_
  #include <ansi_c.h>
  #include "threads_realtime.h"
  #include "timing_realtime.h"
  #define CLEAR_SCREEN Cls()
#elif WIN32
  #include <Windows.h>
  #include "threads_windows.h"
  #include "timing_windows.h"
  #define CLEAR_SCREEN system( "cls" )
#else
  #include <stdio.h>
  #include <stdlib.h>
  #include "threads_unix.h"
  #include "timing_unix.h"
  #define CLEAR_SCREEN system( "clear" );
#endif

#ifdef _MSC_VER
  #define INLINE __forceinline /* use __forceinline (VC++ specific) */
#else
  #define INLINE inline        /* use standard inline */
#endif

static const size_t DEBUG_MESSAGE_SIZE = 256;
static const size_t MAX_DEBUG_MESSAGES = 1000;
  
typedef char DebugMessage[ DEBUG_MESSAGE_SIZE ];

static DebugMessage debugMessages[ MAX_DEBUG_MESSAGES ];
static size_t debugMessagesCount = 0;

static Thread_Lock debugPrintLock = NULL;

size_t searchDebugMessage( const char* event, const char* source )
{
  size_t index;
  for( index = 0; index < debugMessagesCount; index++ )
  {
    if( strstr( debugMessages[ index ], event ) != NULL 
    && strstr( debugMessages[ index ], source ) != NULL )
      break;
  }
  
  return index;
}

void* printDebugMessages( void* args )
{
  while( *debugPrintLock != 0 )
  {
    CLEAR_SCREEN;
    
    LOCK_THREAD( debugPrintLock );
    #ifdef _LINK_CVI_LVRT_
    printf( "\n\n\n\n\n\n\n\n" );
    #endif
    for( size_t i = 0; i < debugMessagesCount; i++ )
      puts( debugMessages[ i ] );
    UNLOCK_THREAD( debugPrintLock );
    
    delay( 100 );
  }
  
  exit_thread( 0 );
  return 0;
}

#define DEBUG_PRINT( event, format, ... ) \
  do { \
    if( debugPrintLock == NULL ) { \
      debugPrintLock = get_new_thread_lock(); \
      run_thread( printDebugMessages, NULL, DETACHED ); \
    } \
    if( debugMessagesCount < MAX_DEBUG_MESSAGES ) { \
      size_t debugMessageIndex = searchDebugMessage( event, __func__ ); \
      if( debugMessageIndex == debugMessagesCount ) debugMessagesCount++; \
        sprintf( debugMessages[ debugMessageIndex ], "%s: " event ": " format, __func__, __VA_ARGS__ ); } \
  } while( 0 );
    

#define ERROR_PRINT( format, ... ) fprintf( stderr, "error: %s: " format, __func__, __VA_ARGS__ )

#ifdef DEEP_DEBUG
  #define EVENT_DEBUG( format, ... ) DEBUG_PRINT( "event", format, __VA_ARGS__ )
  #define LOOP_DEBUG( format, ... ) DEBUG_PRINT( "update", format, __VA_ARGS__ )
#elif SIMPLE_DEBUG
  #define EVENT_DEBUG( format, ... ) DEBUG_PRINT( "event", format, __VA_ARGS__ )
  #define LOOP_DEBUG( format, ... ) do { } while( 0 )
#else
  #define EVENT_DEBUG( format, ... ) do { } while( 0 )
  #define LOOP_DEBUG( format, ... ) do { } while( 0 )
#endif
  
extern INLINE void print_platform_error( const char* message )
{
  #ifdef _CVI_DLL_
  #elif WIN32
  fprintf( stderr, "%s: code: %d\n", message, GetLastError() );
  #else
  perror( message );
  #endif
}

#endif // ERROR_H
