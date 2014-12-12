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

#ifdef NIXNET
  #include "nixnet.h"
#else
  #include "nixnet_stub.h"
#endif
#include <iostream>
#include <string>

using namespace std;

class CANFrame{

private:

    //Variáveis da classe
    nxSessionRef_t  g_SessionRef;
	
    // Declare all variables for function 
	string frameArray;
    u8 flags;
    u8 frameType;
    u8 buffer[ sizeof(nxFrameVar_t) ];
    nxFrameVar_t * ptrFrame;

public:
	
//INICIALIZADOR DO FRAME CAN
void Init( int type, string sInterface, string& sDatabase, string& sCluster, string sFrame ){
	
    //INICIALIZA VARIAVEIS
    nxStatus_t g_Status = 0;
	flags = 0;
	frameType = nxFrameType_CAN_Data;	//MACRO

	frameArray = sFrame;
  
	//Create an xnet session for Signal Input
	g_Status = nxCreateSession( sDatabase.c_str(), sCluster.c_str(), frameArray.c_str(), sInterface.c_str(), type, &g_SessionRef );
	
	PrintStatus( g_Status, frameArray + "(nxCreateSession)" );
}

//CONSTRUTOR DA CLASSE - default
CANFrame() {}

//=============================================================================  
// Display Error Function 
//============================================================================= 
void PrintStatus( nxStatus_t status, string source ) 
{
    char statusString[1024];
     
    if( status != nxSuccess ) 
    {  
	nxStatusToString( status, sizeof(statusString), statusString );
	cout << source << " - NI-XNET Status: " << statusString << endl;
	nxClear( g_SessionRef );
	cout << "------------------" << endl;
    }
}
	
//Método de leitura de um frame
void Write( u8 payload[8] ){

	ptrFrame = (nxFrameVar_t*)buffer;
	nxStatus_t g_Status;
	g_Status = 0;

	//Configura o valor dos FRAMES
	ptrFrame->Timestamp = 0;
	ptrFrame->Flags = flags;
	ptrFrame->Identifier = 66; 
	ptrFrame->Type = frameType;
	ptrFrame->PayloadLength= 8;
	
	//Configura o valor do FRAME
	ptrFrame->Payload[0] = payload[0];
	ptrFrame->Payload[1] = payload[1];
	ptrFrame->Payload[2] = payload[2];
	ptrFrame->Payload[3] = payload[3];
	ptrFrame->Payload[4] = payload[4];
	ptrFrame->Payload[5] = payload[5];
	ptrFrame->Payload[6] = payload[6];
	ptrFrame->Payload[7] = payload[7];

	//ENVIA FRAME PARA A INTERFACE CAN
	g_Status = nxWriteFrame( g_SessionRef, &buffer, sizeof(nxFrameVar_t), 10 );
	
	PrintStatus( g_Status, frameArray + "(nxWriteFrame)" );  
}

	
//Método de leitura de um frame
void Read( u8 payload[8] ){

    //DECLARAÇÃO DE VARIAVEIS
	u32 temp;
	nxStatus_t g_Status;
	g_Status = 0;

	g_Status = nxReadFrame( g_SessionRef, buffer, sizeof(buffer), 0, &temp );

	if( g_Status == nxSuccess )
	{
	    ptrFrame = (nxFrameVar_t*)buffer;
	    payload[0] = ptrFrame->Payload[0]; 
	    payload[1] = ptrFrame->Payload[1]; 
	    payload[2] = ptrFrame->Payload[2]; 
	    payload[3] = ptrFrame->Payload[3]; 
	    payload[4] = ptrFrame->Payload[4]; 
	    payload[5] = ptrFrame->Payload[5]; 
	    payload[6] = ptrFrame->Payload[6]; 
	    payload[7] = ptrFrame->Payload[7];
	}
	 
	PrintStatus( g_Status, frameArray + "(nxReadFrame)" );  
	
}
        
};


#endif	/* CAN_FRAME_H */

