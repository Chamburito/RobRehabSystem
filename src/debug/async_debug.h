///////////////////////////////////////////////////////////////////////
/////       Asyncronous Debug and error handling utilities        /////
///////////////////////////////////////////////////////////////////////

#ifndef ASYNC_DEBUG_H
#define ASYNC_DEBUG_H

#include <string.h>
#include <stdarg.h>

#ifdef _LINK_CVI_LVRT_
  #include "time/timing_realtime.h"
#elif WIN32
  #include "time/timing_windows.h"
#else
  #include "time/timing_unix.h"
#endif

#ifdef _CVI_
  #include "threads/threads_realtime.h"
#elif WIN32
  #include "threads/threads_windows.h"
#else
  #include "threads/threads_unix.h"
#endif

#include "debug.h" 

#define NO_CODE do { } while( 0 )

const size_t DEBUG_MESSAGE_LENGTH = 256;
  
#if defined SIMPLE_DEBUG || defined DEEP_DEBUG

  const size_t EVENT_NAME_LENGTH = 8;
  const size_t SOURCE_NAME_LENGTH = 64;
  
  const size_t MAX_DEBUG_MESSAGES = 1000;
  
  typedef struct _DebugMessage
  {
    uint8_t key;
    char event[ EVENT_NAME_LENGTH ];
    char source[ SOURCE_NAME_LENGTH ];
    char message[ DEBUG_MESSAGE_LENGTH ];
    uint32_t count;
  }
  DebugMessage;

  static DebugMessage debugMessagesList[ MAX_DEBUG_MESSAGES ];
  static size_t debugMessagesCount = 0;

  static short messageUpdate = -1;
  static ThreadLock printLock;

  static INLINE size_t AsyncDebug_SearchMessage( uint8_t key, const char* source, const char* event )
  {
    size_t index;
    for( index = 0; index < debugMessagesCount; index++ )
    {
      if( key == debugMessagesList[ index ].key )
      {
        if( strncmp( debugMessagesList[ index ].event, event, EVENT_NAME_LENGTH ) == 0 )
        {
          if( strncmp( debugMessagesList[ index ].source, source, SOURCE_NAME_LENGTH ) == 0 )
            break;
        }
      }
    }
  
    return index;
  }

  static void* AsyncDebug_Print( void* args )
  {
    //SetStdioWindowOptions( 1000000, 1, 0 );
    
    int lastUpdateTime = Timing_GetExecTimeSeconds();
    
    while( messageUpdate != -1 )
    {
      if( messageUpdate == 1 )
      {
        //ThreadLock_Aquire( printLock );
        
        CLEAR_SCREEN;

        for( size_t i = 0; i < debugMessagesCount; i++ )
          printf( "%s[%u]: %s: %s\n", debugMessagesList[ i ].event, debugMessagesList[ i ].count, 
                                      debugMessagesList[ i ].source, debugMessagesList[ i ].message );

        messageUpdate = 0;
        
        //ThreadLock_Release( printLock );
        
        lastUpdateTime = Timing_GetExecTimeSeconds();
      }
      
      if( Timing_GetExecTimeSeconds() - lastUpdateTime > 0 )
      {
        messageUpdate = -1;
        //ThreadLock_Discard( printLock );
      }
    
      Timing_Delay( 200 );
    }
  
    Thread_Exit( 0 );
    return 0;
  }

  void AsyncDebug_SetEvent( uint8_t key, const char* source, const char* event, const char* format, ... )
  {
    static va_list printArgs;
    static char debugMessageBuffer[ DEBUG_MESSAGE_LENGTH ];
    
    va_start( printArgs, format );
    
    if( messageUpdate == -1 )
    {
      messageUpdate = 0;
      //printLock = ThreadLock_Create();
      Thread_Start( AsyncDebug_Print, NULL, THREAD_DETACHED );
    }
    
    if( debugMessagesCount < MAX_DEBUG_MESSAGES ) 
    {
      size_t eventIndex = AsyncDebug_SearchMessage( key, source, event );
      
      if( eventIndex == debugMessagesCount ) 
      {
        debugMessagesCount++;
        debugMessagesList[ eventIndex ].key = key;
        strncpy( debugMessagesList[ eventIndex ].event, event, EVENT_NAME_LENGTH );
        strncpy( debugMessagesList[ eventIndex ].source, source, SOURCE_NAME_LENGTH );
        debugMessagesList[ eventIndex ].count = 1;
      }
      
      if( vsnprintf( debugMessageBuffer, DEBUG_MESSAGE_LENGTH, format, printArgs ) > 0 )
      {
        if( strncmp( debugMessagesList[ eventIndex ].message, debugMessageBuffer, DEBUG_MESSAGE_LENGTH ) == 0 )
          debugMessagesList[ eventIndex ].count++;
        else
          strncpy( debugMessagesList[ eventIndex ].message, debugMessageBuffer, DEBUG_MESSAGE_LENGTH );
        
        messageUpdate = 1;
      }
    }
    
    va_end( printArgs );
  }

  #define DEBUG_EVENT( key, format, ... ) AsyncDebug_SetEvent( (uint8_t) key, __func__, "event", format, __VA_ARGS__ )
  #define ERROR_EVENT( format, ... ) AsyncDebug_SetEvent( (uint8_t) errno, __func__, "error", format ": %s", __VA_ARGS__, strerror( errno ) ) 

#else
  #define DEBUG_EVENT( key, format, ... ) NO_CODE
  #define ERROR_EVENT( format, ... ) NO_CODE
#endif

#if defined DEEP_DEBUG
  #define DEBUG_UPDATE( format, ... ) AsyncDebug_SetEvent( 0, __func__, "update", format, __VA_ARGS__ )
#else
  #define DEBUG_UPDATE( format, ... ) NO_CODE
#endif

#endif // ASYNC_DEBUG_H
