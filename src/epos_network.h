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
static char CAN_database[ 256 ];
static char CAN_cluster[ 256 ];

// Network control frames
static CAN_Frame* NMT = NULL;
static CAN_Frame* SYNC = NULL;

void epos_network_start( const char* database_name, const char* cluster_name, int nodeId )
{
  // Stores default database name and cluster name for initialized CAN network
  strcpy( CAN_database, database_name );
  strcpy( CAN_cluster, cluster_name );

  // Address and initialize NMT (Network Master) frame
  NMT = can_frame_init( FRAME_OUT, "CAN2", CAN_database, CAN_cluster, "NMT" );
  // Address and initialize SYNC (Syncronization) frame
  SYNC = can_frame_init( FRAME_OUT, "CAN2", CAN_database, CAN_cluster, "SYNC" );

  // Start PDOs sending Start payload to the network
  u8 payload[8] = { 0x01, nodeId }; // Rest of the array as 0x0
  can_frame_write( NMT, payload );
}

CAN_Frame* epos_network_init_frame( Frame_Mode mode, const char* interface_name, const char* frame_name )
{
  CAN_Frame* epos_network_frame = (CAN_Frame*) malloc( sizeof(CAN_Frame*) );

  if( (epos_network_frame = can_frame_init( mode, interface_name, CAN_database, CAN_cluster, frame_name )) == NULL )
    return NULL;

  return epos_network_frame;
}

extern inline void epos_network_end_frame( CAN_Frame* frame )
{
  can_frame_end( frame );
}

void epos_network_sync()
{
  // Build Sync payload (all 0x0) 
  static u8 payload[8];
    
  can_frame_write( SYNC, payload );
}

// Stop CAN network communications
void epos_network_stop( int nodeId )
{
  // Stop PDOs sending Stop payload to the network
  u8 payload[8] = { 0x80, nodeId }; // Rest of the array as 0x0
  can_frame_write( NMT, payload );

  can_frame_end( NMT );
  can_frame_end( SYNC );
}

void epos_network_reset_comm()
{
  u8 payload[8] = { 0x82 }; // Rest of the array as 0x0
  can_frame_write( NMT, payload );
}

void epos_network_reset_nodes()
{
  u8 payload[8] = { 0x821 }; // Rest of the array as 0x0
  can_frame_write( NMT, payload );
}

#endif	/* EPOS_NETWORK_H */

