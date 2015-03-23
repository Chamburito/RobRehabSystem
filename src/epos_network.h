/* 
 * File:   EPOS_NETWORK.h
 * Author: GUILHERME FERNANDES
 *
 * Created on 26 de Janeiro de 2012, 10:26
 */

#ifndef EPOS_NETWORK_H
#define	EPOS_NETWORK_H

#include "can_frame.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//CAN database addressing
static char CANDatabase[ 256 ];
static char CANCluster[ 256 ];

// Network control frames
static CANFrame* NMT = NULL;
static CANFrame* SYNC = NULL;

void EposNetwork_Start( const char* databaseName, const char* clusterName, u8 nodeID )
{
  // Stores default database name and cluster name for initialized CAN network
  strcpy( CANDatabase, databaseName );
  strcpy( CANCluster, clusterName );

  // Address and initialize NMT (Network Master) frame
  NMT = CANFrame_Init( FRAME_OUT, "CAN2", CANDatabase, CANCluster, "NMT" );
  // Address and initialize SYNC (Syncronization) frame
  SYNC = CANFrame_Init( FRAME_OUT, "CAN2", CANDatabase, CANCluster, "SYNC" );

  // Start PDOs sending Start payload to the network
  u8 payload[8] = { 0x01, nodeID }; // Rest of the array as 0x0
  CANFrame_Write( NMT, payload );
}

CANFrame* EposNetwork_InitFrame( enum FrameMode mode, const char* interfaceName, const char* frameName )
{
  CANFrame* epos_network_frame = (CANFrame*) malloc( sizeof(CANFrame*) );

  if( (epos_network_frame = CANFrame_Init( mode, interfaceName, CANDatabase, CANCluster, frameName )) == NULL )
    return NULL;

  return epos_network_frame;
}

extern inline void EposNetwork_EndFrame( CANFrame* frame )
{
  CANFrame_End( frame );
}

void EposNetwork_Sync()
{
  // Build Sync payload (all 0x0) 
  static u8 payload[8];
    
  CANFrame_Write( SYNC, payload );
}

// Stop CAN network communications
void EposNetwork_Stop( u8 nodeID )
{
  // Stop PDOs sending Stop payload to the network
  u8 payload[8] = { 0x80, nodeID }; // Rest of the array as 0x0
  CANFrame_Write( NMT, payload );

  CANFrame_End( NMT );
  CANFrame_End( SYNC );
}

void EposNetwork_ResetCommunication()
{
  u8 payload[8] = { 0x82 }; // Rest of the array as 0x0
  CANFrame_Write( NMT, payload );
}

void EposNetwork_ResetNodes()
{
  u8 payload[8] = { 0x821 }; // Rest of the array as 0x0
  CANFrame_Write( NMT, payload );
}

#endif	/* EPOS_NETWORK_H */

