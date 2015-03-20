///////////////////////////////////////////////////////////////////////
/////       Asyncronous Debug and error handling utilities        /////
///////////////////////////////////////////////////////////////////////

#ifndef ASYNC_DEBUG_H
#define ASYNC_DEBUG_H

#include <string.h>
#include <stdarg.h>

#ifdef _LINK_CVI_LVRT_
  #include "timing_realtime.h"
#elif WIN32
  #include "timing_windows.h"
#else
  #include "timing_unix.h"
#endif

#ifdef _CVI_
  #include "threads_realtime.h"
#elif WIN32
  #include "threads_windows.h"
#else
  #include "threads_unix.h"
#endif

#include "debug.h" 

#define NO_CODE do { } while( 0 )

#ifdef SIMPLE_DEBUG || DEEP_DEBUG

  const size_t DEBUG_MESSAGE_SIZE = 256;
  const size_t MAX_DEBUG_MESSAGES = 1000;
  
  typedef char DebugMessage[ DEBUG_MESSAGE_SIZE ];

  DebugMessage debugMessages[ MAX_DEBUG_MESSAGES ];
  size_t debugMessagesCount = 0;

  short messageUpdate = -1;

  size_t SearchDebugMessage( const char* event, const char* source )
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

  static void* PrintDebugMessages( void* args )
  {
    while( messageUpdate != -1 )
    {
      if( messageUpdate == 1 )
      {
        CLEAR_SCREEN;

        #ifdef _LINK_CVI_LVRT_
        printf( "\n\n\n\n\n\n\n\n" );
        #endif
        for( size_t i = 0; i < debugMessagesCount; i++ )
          puts( debugMessages[ i ] );

        messageUpdate = 0;
      }
    
      delay( 100 );
    }
  
    thread_exit( 0 );
    return 0;
  }

  extern inline void InitDebug() 
  { 
    messageUpdate = 0; 
    thread_start( PrintDebugMessages, NULL, DETACHED ); 
  }

  extern inline void EndDebug()
  {
    messageUpdate = -1;
  }

  #define DEBUG_INIT InitDebug()
  #define DEBUG_END EndDebug()

  #define ASYNC_DEBUG_PRINT( event, format, ... ) \
    do { \
      if( debugMessagesCount < MAX_DEBUG_MESSAGES ) { \
        size_t debugMessageIndex = SearchDebugMessage( event, __func__ ); \
        if( debugMessageIndex == debugMessagesCount ) debugMessagesCount++; \
          sprintf( debugMessages[ debugMessageIndex ], "%s: " event ": " format, __func__, __VA_ARGS__ ); \ 
        messageUpdate = 1; \
      } \
    } while( 0 )

  #define DEBUG_EVENT( format, ... ) ASYNC_DEBUG_PRINT( "event", format, __VA_ARGS__ )
  #define ERROR_EVENT( format, ... ) ASYNC_DEBUG_PRINT( "error", format, __VA_ARGS__ ) 

#else
  #define DEBUG_INIT NO_CODE
  #define DEBUG_END NO_CODE
  #define DEBUG_EVENT( format, ... ) NO_CODE
  #define ERROR_EVENT( format, ... ) NO_CODE
#endif

#ifdef DEEP_DEBUG
  #define DEBUG_UPDATE( format, ... ) ASYNC_DEBUG_PRINT( "update", format, __VA_ARGS__ )
#else
  #define DEBUG_UPDATE( format, ... ) NO_CODE
#endif

#endif // ASYNC_DEBUG_H
