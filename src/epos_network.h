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

// List of user created frames
static CAN_Frame** frame_list = NULL;
static size_t n_frames = 0;

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
  frame_list = (CAN_Frame**) realloc( frame_list, ( n_frames + 1 ) * sizeof(CAN_Frame*) );

  if( (frame_list[ n_frames ] = can_frame_init( mode, interface_name, CAN_database, CAN_cluster, frame_name )) == NULL )
    return NULL;

  return frame_list[ n_frames++ ];
}

void epos_network_sync()
{
  // Build Sync pauload (all 0x0) 
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

  for( int frame_id = 0; frame_id < n_frames; frame_id++ )
    can_frame_end( frame_list[ frame_id ] );

  free( frame_list );
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

