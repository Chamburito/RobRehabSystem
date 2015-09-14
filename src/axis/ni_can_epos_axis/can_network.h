/* 
 * File:   CAN_NETWORK.h
 * Author: GUILHERME FERNANDES
 *
 * Created on 26 de Janeiro de 2012, 10:26
 */

#ifndef CAN_NETWORK_H
#define CAN_NETWORK_H

#include "ni_can_epos_axis/can_frame.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//CAN database addressing
static char networkDatabaseName[ 256 ];
static char networkClusterName[ 256 ];

// Network control frames
static CANFrame* NMT = NULL;
static CANFrame* SYNC = NULL;

void CANNetwork_Reset();

void CANNetwork_Start( const char* databaseName, const char* clusterName )
{
  // Stores default database name and cluster name for initialized CAN network
  strcpy( networkDatabaseName, databaseName );
  strcpy( networkClusterName, clusterName );

  // Address and initialize NMT (Network Master) frame
  NMT = CANFrame_Init( FRAME_OUT, "CAN2", networkDatabaseName, networkClusterName, "NMT" );
  // Address and initialize SYNC (Syncronization) frame
  SYNC = CANFrame_Init( FRAME_OUT, "CAN2", networkDatabaseName, networkClusterName, "SYNC" );

  CANNetwork_Reset();
}

void CANNetwork_InitNode( uint8_t nodeID )
{
  // Start PDOs sending Start payload to the network
  u8 payload[8] = { 0x01, nodeID }; // Rest of the array as 0x0
  CANFrame_Write( NMT, payload );
}

void CANNetwork_EndNode( uint8_t nodeID )
{
  // Stop PDOs sending Stop payload to the network
  u8 payload[8] = { 0x80, nodeID }; // Rest of the array as 0x0
  CANFrame_Write( NMT, payload );
}

CANFrame* CANNetwork_InitFrame( enum FrameMode mode, const char* interfaceName, const char* frameName )
{
  CANFrame* eposNetworkFrame = (CANFrame*) malloc( sizeof(CANFrame*) );

  if( (eposNetworkFrame = CANFrame_Init( mode, interfaceName, networkDatabaseName, networkClusterName, frameName )) == NULL )
    return NULL;

  return eposNetworkFrame;
}

extern inline void CANNetwork_EndFrame( CANFrame* frame )
{
  CANFrame_End( frame );
}

void CANNetwork_Sync()
{
  // Build Sync payload (all 0x0) 
  static u8 payload[8];
  CANFrame_Write( SYNC, payload );
}

// Stop CAN network communications
void CANNetwork_Stop()
{
  // Stop PDOs sending Stop payload to the network
  u8 payload[8] = { 0x80 }; // Rest of the array as 0x0
  CANFrame_Write( NMT, payload );

  CANFrame_End( NMT );
  CANFrame_End( SYNC );
}

void CANNetwork_Reset()
{
  u8 payload[8] = { 0x82 }; // Rest of the array as 0x0
  CANFrame_Write( NMT, payload );
  
  Timing_Delay( 200 );
  
  payload[0] = 0x01; // Rest of the array as 0x0
  CANFrame_Write( NMT, payload );
}

void CANNetwork_ResetNodes()
{
  u8 payload[8] = { 0x821 }; // Rest of the array as 0x0
  CANFrame_Write( NMT, payload );
}

#endif	/* CAN_NETWORK_H */
