/* 
 * File:   CAN_FRAME.h
 * Author: Guilherme Fernandes
 *
 * ESTA CLASSE REALIZA A LEITURA E ESCRITA DE FRAMES CAN NAS PLACAS DA NATIONAL INSTRUMENTS
 * A MONTAGEM DO PAYLOAD DE ACORDO COM O NECESSÁRIO DEVE SER REALIZADA EXTERNAMENTE, A 
 * FUNÇÃO DA CLASSE É APENAS ESCREVER E REALIZAR A LEITURA DOS COBID SELECIONADOS.
 * 
 * 
 * Created on 26 de Janeiro de 2012, 10:19
 */

#ifndef CAN_FRAME_H
#define	CAN_FRAME_H

#ifdef _CVI_
  #include <nixnet.h>
#elif NIXNET
  #include "nixnet.h"
#else
  #include "nixnet_stub.h"
#endif

#include "debug/async_debug.h"

enum CANFrameMode { FRAME_IN = nxMode_FrameInSinglePoint, FRAME_OUT = nxMode_FrameOutSinglePoint };

// CAN Frame data structure
typedef struct _CANFrameData
{
  nxSessionRef_t ref_session;
  char* name;
  u8 flags;
  u8 type;
  u8 buffer[ sizeof(nxFrameVar_t) ];
} 
CANFrameData;

typedef CANFrameData* CANFrame;

// Display CAN error string based on status code
static void PrintFrameStatus( nxStatus_t statusCode, const char* frameName, const char* source )
{
  static char statusString[ 1024 ];
    
  nxStatusToString( statusCode, sizeof(statusString), statusString );
  /*ERROR_EVENT*/DEBUG_PRINT( "%s - NI-XNET Status: %s", source, statusString );
}

// CAN Frame initializer
CANFrame CANFrame_Init( enum CANFrameMode mode, const char* interfaceName, const char* databaseName, const char* clusterName, const char* frameName )
{
  CANFrame frame = (CANFrame) malloc( sizeof(CANFrameData) );

  frame->flags = 0;
  frame->type = nxFrameType_CAN_Data;	//MACRO

  frame->name = (char*) calloc( strlen(frameName) + 1, sizeof(char) );
  strcpy( frame->name, frameName );
  
  DEBUG_PRINT( "creating frame %s of type %d and mode %d", frame->name, frame->type, mode );
  
  //Create an xnet session
  nxStatus_t statusCode = nxCreateSession( databaseName, clusterName, frameName, interfaceName, (u32) mode, &(frame->ref_session) );
  if( statusCode != nxSuccess )
  {
    PrintFrameStatus( statusCode, frameName, "(nxCreateSession)" );
    nxClear( frame->ref_session );
    return NULL;
  }
  
  /*DEBUG_EVENT( 1,*/DEBUG_PRINT( "created frame %s session %u", frameName, frame->ref_session );

  return frame;
}

void CANFrame_End( CANFrame frame )
{
  if( frame != NULL )
  {
    nxClear( frame->ref_session );
    free( frame->name );
    free( frame );
    frame = NULL;
  }
}

// Read data from CAN frame to array
void CANFrame_Read( CANFrame frame, u8 payload[8] )
{
  nxFrameVar_t* ptr_frame = (nxFrameVar_t*) frame->buffer;

  u32 temp;
    
  nxStatus_t statusCode = nxReadFrame( frame->ref_session, frame->buffer, sizeof(frame->buffer), 0, &temp );   
  if( statusCode != nxSuccess )
    PrintFrameStatus( statusCode, frame->name, "(nxReadFrame)" );
  else
    memcpy( payload, ptr_frame->Payload, sizeof(u8) * ptr_frame->PayloadLength );
}

// Write data from payload to CAN frame
void CANFrame_Write( CANFrame frame, u8 payload[8] )
{
  nxFrameVar_t* ptr_frame = (nxFrameVar_t*) frame->buffer;
  
  ptr_frame->Timestamp = 0;
  ptr_frame->Flags = frame->flags;
  ptr_frame->Identifier = 66; 
  ptr_frame->Type = frame->type;
  ptr_frame->PayloadLength= 8;

  memcpy( ptr_frame->Payload, payload, sizeof(u8) * ptr_frame->PayloadLength );

  //DEBUG_EVENT( 1,  "trying to write with session %u", frame->ref_session );
  
  nxStatus_t statusCode = nxWriteFrame( frame->ref_session, &(frame->buffer), sizeof(nxFrameVar_t), 0.0 );
  if( statusCode != nxSuccess )
    PrintFrameStatus( statusCode, frame->name, "(nxWriteFrame)" );
}

#endif	/* CAN_FRAME_H */

