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

  const size_t DEBUG_MESSAGE_LENGTH = 256;
  const size_t MAX_DEBUG_MESSAGES = 1000;
  
  typedef char DebugMessage[ DEBUG_MESSAGE_LENGTH ];

  static DebugMessage debugMessages[ MAX_DEBUG_MESSAGES ];
  static size_t debugMessagesCount = 0;

  static short messageUpdate = -1;

  static INLINE size_t AsyncDebug_SearchMessage( const char* source, const char* event )
  {
    size_t index;
    for( index = 0; index < debugMessagesCount; index++ )
    {
      if( strstr( debugMessages[ index ], source ) != NULL )
      {
        if( strstr( debugMessages[ index ], event ) != NULL )
          break;
      }
    }
  
    return index;
  }

  static void* AsyncDebug_Print( void* args )
  {
    int lastUpdateTime = get_exec_time_seconds();
    
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
        
        lastUpdateTime = get_exec_time_seconds();
      }
      
      if( get_exec_time_seconds() - lastUpdateTime > 5 )
        messageUpdate = -1;
    
      delay( 100 );
    }
  
    thread_exit( 0 );
    return 0;
  }

  void AsyncDebug_SetEvent( const char* source, const char* event, const char* format, ... )
  {
    static va_list printArgs;
    static char debugMessageFormat[ DEBUG_MESSAGE_LENGTH ];
    
    va_start( printArgs, format );
    
    if( messageUpdate == -1 )
    {
      messageUpdate = 0; 
      thread_start( AsyncDebug_Print, NULL, DETACHED );
    }
    
    if( debugMessagesCount < MAX_DEBUG_MESSAGES ) 
    { 
      size_t debugMessageIndex = AsyncDebug_SearchMessage( source, event );
      if( debugMessageIndex == debugMessagesCount ) debugMessagesCount++;
      if( snprintf( debugMessageFormat, DEBUG_MESSAGE_LENGTH, "%s: %s: %s", event, source, format ) > 0 )
      {
        vsnprintf( debugMessages[ debugMessageIndex ], DEBUG_MESSAGE_LENGTH, debugMessageFormat, printArgs ); 
        messageUpdate = 1;
      }
    }
    
    va_end( printArgs );
  }

  #define DEBUG_EVENT( format, ... ) AsyncDebug_SetEvent( __func__, "event", format, __VA_ARGS__ )
  #define ERROR_EVENT( format, ... ) AsyncDebug_SetEvent( __func__, "error", format ": %s", __VA_ARGS__, strerror( errno ) ) 

#else
  #define DEBUG_EVENT( format, ... ) NO_CODE
  #define ERROR_EVENT( format, ... ) NO_CODE
#endif

#ifdef DEEP_DEBUG
  #define DEBUG_UPDATE( format, ... ) AsyncDebug_SetEvent( __func__, "update", format, __VA_ARGS__ )
#else
  #define DEBUG_UPDATE( format, ... ) NO_CODE
#endif

#endif // ASYNC_DEBUG_H
