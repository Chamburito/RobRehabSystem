//////////////////////////////////////////////////////////////////////////////////////////////////////////
/////                   AXIS CONTROL THROUGH SHARED MEMORY (Anklebot on Linux RT)                    /////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef AXIS_SHM_INTERFACE_H
#define AXIS_SHM_INTERFACE_H

#ifdef WIN32
  #include "../timing_windows.h"
#else
  #include "../timing_unix.h"
#endif

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <malloc.h>

#include <stdbool.h>

#include "../async_debug.h"

#include "../klib/khash.h"

#include "../axis_interface.h"

#include "ruser.h"
#include "robdecls.h"
#include "rtl_inc.h"

static const uint8_t operationModes[ AXIS_OP_MODES_NUMBER ] = { 0xFF, 0xFE };

static int Connect( int );
static void Disconnect( int );
static void Enable( int );
static void Disable( int );
static void Reset( int );
static bool IsEnabled( int );
static bool HasError( int );
static bool ReadMeasures( int, double[ AXIS_MEASURES_NUMBER ] );
static void WriteControl( int, double[ AXIS_OP_MODES_NUMBER ] );
static void SetOperationMode( int, enum AxisOperationModes );

KHASH_MAP_INIT_INT( Shm, Ob* );
static khash_t( Shm )* interfacesList = NULL;

const AxisInterface AxisShmInterface = { .Connect = Connect,
                                         .Disconnect = Disconnect,
                                         .Enable = Enable,
                                         .Disable = Disable,
                                         .Reset = Reset,
                                         .IsEnabled = IsEnabled,
                                         .HasError = HasError,
                                         .ReadMeasures = ReadMeasures,
                                         .WriteControl = WriteControl,
                                         .SetOperationMode = SetOperationMode };
                                         
static int Connect( int sharedMemoryKey )
{
  DEBUG_EVENT( 0, "connecting shared memory interface for key %x", sharedMemoryKey );
  
  if( interfacesList == NULL ) interfacesList = kh_init( Shm );
  
  int sharedMemoryID = shmget( OB_KEY, sizeof(Ob), 0666 );
  
  if( sharedMemoryID == -1 )
  {
    ERROR_EVENT( "failed allocating shared memory for key %x", sharedMemoryKey );
    return -1;
  }
  
  int insertionStatus;
  khiter_t interfaceID = kh_put( Shm, interfacesList, sharedMemoryKey, &insertionStatus );
  if( insertionStatus > 0 )
  {
    Ob* sharedMemoryObject = shmat( interfaceID, NULL, 0 );
    
    if( (int) sharedMemoryObject == -1 )
    {
      ERROR_EVENT( "failed binding shared memory with key %x to interface object", sharedMemoryKey );
      return -1;
    }
    
    kh_value( interfacesList, interfaceID ) = sharedMemoryObject;
    
    return interfaceID;
  }
  
  if( insertionStatus == -1 )
    ERROR_EVENT( "failed creating interface for shared memory key %x", sharedMemoryKey );
  else if( insertionStatus == 0 )
    ERROR_EVENT( "interface for shared memory key %x already exists", sharedMemoryKey );
    
  return -1;  
}

static void Disconnect( int interfaceID )
{
  if( kh_exist( interfacesList, interfaceID ) )
  {
    shmdt( kh_value( interfacesList, interfaceID ) );
    
    kh_del( Shm, interfacesList, interfaceID );
    
    if( kh_size( interfacesList ) == 0 )
    {
      kh_destroy( Shm, interfacesList );
      interfacesList = NULL;
    }
  }
}

static void Enable( int interfaceID )
{
  if( kh_exist( interfacesList, interfaceID ) )
  {
    Ob* sharedMemoryObject = kh_value( interfacesList, interfaceID );
    
    sharedMemoryObject->ankle.stiff = 50.0;
  }
}

static void Disable( int interfaceID )
{
  if( kh_exist( interfacesList, interfaceID ) )
  {
    Ob* sharedMemoryObject = kh_value( interfacesList, interfaceID );
    
    sharedMemoryObject->ankle.stiff = 0.0;
  }
}

static void Reset( int interfaceID )
{
  return;
}

static bool IsEnabled( int interfaceID )
{
  return;
}

static bool HasError( int interfaceID )
{
  return;
}

static bool ReadMeasures( int interfaceID, double measuresList[ AXIS_MEASURES_NUMBER ] )
{
  if( kh_exist( interfacesList, interfaceID ) )
  {
    Ob* sharedMemoryObject = kh_value( interfacesList, interfaceID );
    
    measuresList[ AXIS_MEASURES_POSITION ] = sharedMemoryObject->ankle.pos.dp;
    measuresList[ AXIS_MEASURES_VELOCITY ] = sharedMemoryObject->ankle.vel.dp;
    measuresList[ AXIS_MEASURES_ACCELERATION ] = sharedMemoryObject->ankle.accel.dp;
    measuresList[ AXIS_MEASURES_TORQUE ] = sharedMemoryObject->ankle.torque.dp;
  }
  
  return false;
}

static void WriteControl( int interfaceID, double setpointsList[ AXIS_OP_MODES_NUMBER ] )
{
  if( kh_exist( interfacesList, interfaceID ) )
  {
    Ob* sharedMemoryObject = kh_value( interfacesList, interfaceID );
    
    sharedMemoryObject->ankle.ref_pos.dp = setpointsList[ AXIS_OP_MODES_POSITION ];
  }
}

static void SetOperationMode( int, enum AxisOperationModes )
{
  return;
}

#endif /* AXIS_SHM_INTERFACE_H */

