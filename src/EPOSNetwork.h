/* 
 * File:   EPOS_NETWORK.h
 * Author: GUILHERME FERNANDES
 *
 * Created on 26 de Janeiro de 2012, 10:26
 */

#ifndef EPOS_NETWORK_H
#define	EPOS_NETWORK_H

#include "CANFrame.h"
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <string>

class EPOSNetwork{

private:

	//FRAMES NMT E SYNC DA REDE
	CANFrame NMT;
	CANFrame SYNC;
	
	
public:

//Construtor da classe
EPOSNetwork(){}


//Construtor da classe
EPOSNetwork( string selectedDatabase, string selectedCluster ){

//ENDEREÇA E INICIALIZA O FRAME NMT
NMT.Init( nxMode_FrameOutSinglePoint, "CAN2", selectedDatabase, selectedCluster, "NMT" ); //11

//ENDEREÇA E INICIALIZA O FRAME SYNC
SYNC.Init( nxMode_FrameOutSinglePoint, "CAN2", selectedDatabase, selectedCluster, "SYNC" ); //11

}
	
//M…TODO QUE ENVIA O START PARA A TRANSMISS√O DOS PDOS PARA A REDE CAN
void StartPDOS( int nodeId ){

    //MONTA O PAYLOAD DE WRITE
	static u8 writePayload[8];
    writePayload[0] = 0x01;
    writePayload[1] = nodeId;
    writePayload[2] = 0x0;
    writePayload[3] = 0x0;
    writePayload[4] = 0x0;
    writePayload[5] = 0x0;
    writePayload[6] = 0x0;
    writePayload[7] = 0x0;

    //ENVIA COMANDO DE START PDOS PARA A REDE
    NMT.Write( writePayload );
}

void Sync(){

    //MONTA O PAYLOAD DE WRITE
	static u8 writePayload[8];
    writePayload[0] = 0x0;
    writePayload[1] = 0x0;
    writePayload[2] = 0x0;
    writePayload[3] = 0x0;
    writePayload[4] = 0x0;
    writePayload[5] = 0x0;
    writePayload[6] = 0x0;
    writePayload[7] = 0x0;

    SYNC.Write( writePayload );
}


//M…TODO QUE ENVIA O STOP PARA A TRANSMISS√O DOS PDOS PARA A REDE CAN
void StopPDOS( int nodeId ){

    //ENVIA COMANDO DE STOP PDOS PARA A REDE
    //MONTA O PAYLOAD DE WRITE
	static u8 writePayload[8];
    writePayload[0] = 0x80;
    writePayload[1] = nodeId;
    writePayload[2] = 0x0;
    writePayload[3] = 0x0;
    writePayload[4] = 0x0;
    writePayload[5] = 0x0;
    writePayload[6] = 0x0;
    writePayload[7] = 0x0;

    //ENVIA COMANDO DE START PDOS PARA A REDE
    NMT.Write( writePayload );
}

//M…TODO QUE ENVIA O RESET TODA A COMUNICA«√O DA REDE
void ResetComm(){

    //ENVIA COMANDO DE RESET PDOS PARA A REDE
    //MONTA O PAYLOAD DE WRITE
	static u8 writePayload[8];
    writePayload[0] = 0x82;
    writePayload[1] = 0x0;
    writePayload[2] = 0x0;
    writePayload[3] = 0x0;
    writePayload[4] = 0x0;
    writePayload[5] = 0x0;
    writePayload[6] = 0x0;
    writePayload[7] = 0x0;

    //ENVIA COMANDO DE START PDOS PARA A REDE
    NMT.Write( writePayload );
}

	//M…TODO QUE ENVIA O RESET PARA TODOS OS NOS DA REDE
void ResetNodes(){
		
    //ENVIA COMANDO DE RESET PDOS PARA A REDE
    //MONTA O PAYLOAD DE WRITE
	static u8 writePayload[8];
    writePayload[0] = 0x81;
    writePayload[1] = 0x0;
    writePayload[2] = 0x0;
    writePayload[3] = 0x0;
    writePayload[4] = 0x0;
    writePayload[5] = 0x0;
    writePayload[6] = 0x0;
    writePayload[7] = 0x0;

    //ENVIA COMANDO DE START PDOS PARA A REDE
    NMT.Write( writePayload );
}

//MÈtodo de destruiÁ„o da classe
~EPOSNetwork(){}

};

#endif	/* EPOS_NETWORK_H */

