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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <malloc.h>

enum Frame_Mode { FRAME_IN = nxMode_FrameInSinglePoint, FRAME_OUT = nxMode_FrameOutSinglePoint };

// CAN Frame data structure
typedef struct _CAN_Frame
{
  nxSessionRef_t session_ref;
  char* name;
  u8 flags;
  u8 type;
  u8 buffer[ sizeof(nxFrameVar_t) ];
} 
CAN_Frame;

// Display CAN error string based on status code
static void print_frame_status( nxStatus_t status_code, const char* frame_name, const char* source )
{
  static char status_string[1024];
    
  nxStatusToString( status_code, sizeof(status_string), status_string );
  fprintf( stderr, "%s - NI-XNET Status: %s\n", source, status_string );
  fprintf( stderr, "------------------\n" );
}

// CAN Frame initializer
CAN_Frame* can_frame_init( Frame_Mode mode, const char* interface_name, const char* database_name, const char* cluster_name, const char* frame_name )
{
  CAN_Frame* frame = (CAN_Frame*) malloc( sizeof(CAN_Frame) );

  frame->flags = 0;
  frame->type = nxFrameType_CAN_Data;	//MACRO

  frame->name = (char*) calloc( strlen(frame_name) + 1, sizeof(char) );
  strcpy( frame->name, frame_name );

  //Create an xnet session for Signal Input
  nxStatus_t status_code = nxCreateSession( database_name, cluster_name, frame_name, interface_name, (u32) mode, &(frame->session_ref) );
  if( status_code != nxSuccess )
  {
    print_frame_status( status_code, frame_name, "(nxCreateSession)" );
    nxClear( frame->session_ref );
    return NULL;
  }

  return frame;
}

// Read data from CAN frame to array
void can_frame_read( CAN_Frame* frame, u8 payload[8] )
{
  nxFrameVar_t* ptr_frame = (nxFrameVar_t*) frame->buffer;

  u32 temp;
  nxStatus_t status_code = nxReadFrame( frame->session_ref, frame->buffer, sizeof(frame->buffer), 0, &temp );   
  if( status_code != nxSuccess )
  {
    print_frame_status( status_code, frame->name, "(nxReadFrame)" );
    nxClear( frame->session_ref );
  }
  else
    memcpy( payload, ptr_frame->Payload, sizeof(u8) * 8/*ptr_frame->PayloadLength*/ );
}

// Write data from payload to CAN frame
void can_frame_write( CAN_Frame* frame, u8 payload[8] )
{
  nxFrameVar_t* ptr_frame = (nxFrameVar_t*) frame->buffer;

  ptr_frame->Timestamp = 0;
  ptr_frame->Flags = frame->flags;
  ptr_frame->Identifier = 66; 
  ptr_frame->Type = frame->type;
  ptr_frame->PayloadLength= 8;

  memcpy( ptr_frame->Payload, payload, sizeof(u8) * 8/*ptr_frame->PayloadLength*/ );

  nxStatus_t status_code = nxWriteFrame( frame->session_ref, &(frame->buffer), sizeof(nxFrameVar_t), 10 );
  if( status_code != nxSuccess )
  {
    print_frame_status( status_code, frame->name, "(nxWriteFrame)" );
    nxClear( frame->session_ref );
  }
}

void can_frame_end( CAN_Frame* frame )
{
  if( frame != NULL )
  {
    free( frame->name );
    free( frame );
    frame = NULL;
  }
}

#endif	/* CAN_FRAME_H */

