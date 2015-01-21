#ifndef ___nixnet_h___
#define ___nixnet_h___

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

#ifndef WIN32
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <termios.h>
#include <assert.h>

#include <stdlib.h>
#include <string.h>

#define STDIN 0

int getch(void) 
{
    static fd_set read_fds;
    static struct timeval timeout = { 0, 0 };
    
    FD_ZERO( &read_fds );
    FD_SET( STDIN, &read_fds );
      
      int c = 0;

      struct termios org_opts, new_opts;
      int res=0;
          //-----  store old settings -----------
      res=tcgetattr(STDIN_FILENO, &org_opts);
      //assert(res==0);
          //---- set new terminal parms --------
      memcpy(&new_opts, &org_opts, sizeof(new_opts));
      new_opts.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ECHOPRT | ECHOKE | ICRNL);
      tcsetattr(STDIN_FILENO, TCSANOW, &new_opts);
      
      select( STDIN + 1, &read_fds, NULL, NULL, &timeout);
  
      if( FD_ISSET( STDIN, &read_fds ) )
        c = getchar();
      
          //------  restore old settings ---------
      res=tcsetattr(STDIN_FILENO, TCSANOW, &org_opts);
      //assert(res==0);
      return(c);
}
#endif

#define nxMode_SignalInSinglePoint           0  // SignalInSinglePoint
#define nxMode_SignalInWaveform              1  // SignalInWaveform
#define nxMode_SignalInXY                    2  // SignalInXY
#define nxMode_SignalOutSinglePoint          3  // SignalOutSinglePoint
#define nxMode_SignalOutWaveform             4  // SignalOutWaveform
#define nxMode_SignalOutXY                   5  // SignalOutXY
#define nxMode_FrameInStream                 6  // FrameInStream
#define nxMode_FrameInQueued                 7  // FrameInQueued
#define nxMode_FrameInSinglePoint            8  // FrameInSinglePoint
#define nxMode_FrameOutStream                9  // FrameOutStream
#define nxMode_FrameOutQueued                10 // FrameOutQueued
#define nxMode_FrameOutSinglePoint           11 // FrameOutSinglePoint

typedef uint8_t u8;
typedef uint32_t u32;
typedef uint64_t u64;
typedef int32_t i32;
typedef double f64; 

// Session Reference (handle).
typedef u32 nxSessionRef_t;

// Absolute timestamp.
typedef u64 nxTimestamp_t;

// Return value
typedef i32 nxStatus_t;

#define nxFrameType_CAN_Data                 0x00

#define nxSuccess                            0

typedef struct {
                   nxTimestamp_t       Timestamp; 
                   u32                 Identifier; 
                   u8                  Type; 
                   u8                  Flags; 
                   u8                  Info; 
                   u8                  PayloadLength; 
                   u8                  Payload[8]; 
               }
        nxFrameVar_t;
            
nxStatus_t nxCreateSession( const char* DatabaseName, const char* ClusterName, const char* List, const char* Interface, u32 Mode, nxSessionRef_t* SessionRef )
{
    static int count;
    *SessionRef = count++;
    
    printf( "database: %s - cluster name: %s - list: %s - interface: %s - session ref: %d\n", DatabaseName, ClusterName, List, Interface, *SessionRef );
    
    return nxSuccess;
}

nxStatus_t nxWriteFrame( nxSessionRef_t SessionRef, void* Buffer, u32 NumberOfBytesForFrames, f64 Timeout )
{
    return nxSuccess;
}

nxStatus_t nxReadFrame( nxSessionRef_t SessionRef, void* Buffer, u32 SizeOfBuffer, f64 Timeout, u32* NumberOfBytesReturned )
{
    static nxFrameVar_t frame;
  
    static int position;
    static int velocity;
    static char key;
  
    #ifndef WIN32
    velocity = 0;
    
    if( (key = getch()) )
    {
        if( key == 'w' )
	{
	  position += 10;
	  velocity = 10;
	}
	else if( key == 's' ) 
	{
	  position -= 10;
	  velocity = -10;
	}
    }
    #endif 
  
    if( SessionRef == 5 )
    {
	frame.Payload[0] = ( position & 0x000000ff );
	frame.Payload[1] = ( position & 0x0000ff00 ) / 0x100;
	frame.Payload[2] = ( position & 0x00ff0000 ) / 0x10000;
	frame.Payload[3] = ( position & 0xff000000 ) / 0x1000000;
	frame.Payload[4] = ( SessionRef & 0x000000ff );
	frame.Payload[5] = ( SessionRef & 0x0000ff00 ) / 0x100; 
	frame.Payload[6] = ( 0 & 0x000000ff );
	frame.Payload[7] = ( 0 & 0x0000ff00 ) / 0x100;
    }
    else if( SessionRef == 7 )
    {
	frame.Payload[0] = ( velocity & 0x000000ff );
	frame.Payload[1] = ( velocity & 0x0000ff00 ) / 0x100;
	frame.Payload[2] = ( velocity & 0x00ff0000 ) / 0x10000;
	frame.Payload[3] = ( velocity & 0xff000000 ) / 0x1000000;
	frame.Payload[4] = ( SessionRef & 0x000000ff );
	frame.Payload[5] = ( SessionRef & 0x0000ff00 ) / 0x100; 
	frame.Payload[6] = ( 0 & 0x000000ff );
	frame.Payload[7] = ( 0 & 0x0000ff00 ) / 0x100;
    }
    
    memcpy( Buffer, &frame, sizeof(frame) );
      
    return nxSuccess;
}

void nxStatusToString( nxStatus_t Status, u32 SizeofString, char* StatusDescription )
{
    return;
}

nxStatus_t nxClear( nxSessionRef_t SessionRef )
{
    return nxSuccess;
}


#endif /* ___nixnet_h___ */