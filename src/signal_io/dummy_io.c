////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  Copyright (c) 2016 Leonardo José Consoni                                  //
//                                                                            //
//  This file is part of RobRehabSystem.                                      //
//                                                                            //
//  RobRehabSystem is free software: you can redistribute it and/or modify    //
//  it under the terms of the GNU Lesser General Public License as published  //
//  by the Free Software Foundation, either version 3 of the License, or      //
//  (at your option) any later version.                                       //
//                                                                            //
//  RobRehabSystem is distributed in the hope that it will be useful,         //
//  but WITHOUT ANY WARRANTY; without even the implied warranty of            //
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the              //
//  GNU Lesser General Public License for more details.                       //
//                                                                            //
//  You should have received a copy of the GNU Lesser General Public License  //
//  along with RobRehabSystem. If not, see <http://www.gnu.org/licenses/>.    //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////


#include "signal_io/interface.h"

#include "klib/khash.h"

#include "debug/async_debug.h"

enum { INPUT_POSITION, INPUT_VELOCITY, INPUT_CURRENT, INPUT_CHANNELS_NUMBER };
enum { OUTPUT_POSITION, OUTPUT_VELOCITY, OUTPUT_CURRENT, OUTPUT_CHANNELS_NUMBER };

static const size_t AQUISITION_BUFFER_LENGTH = 1;

typedef struct _SignalIOTaskData
{
  unsigned int inputChannelUsesList[ INPUT_CHANNELS_NUMBER ];
  bool isOutputChannelUsed;
  double measuresList[ INPUT_CHANNELS_NUMBER ];
}
SignalIOTaskData;

typedef SignalIOTaskData* SignalIOTask;

KHASH_MAP_INIT_INT( TaskInt, SignalIOTask )
static khash_t( TaskInt )* tasksList = NULL;

DECLARE_MODULE_INTERFACE( SIGNAL_IO_INTERFACE ) 

static inline bool IsTaskStillUsed( SignalIOTask );

int InitTask( const char* taskConfig )
{
  if( tasksList == NULL ) tasksList = kh_init( TaskInt );
  
  int taskKey = (int) kh_str_hash_func( taskConfig );
  
  int insertionStatus;
  khint_t newTaskIndex = kh_put( TaskInt, tasksList, taskKey, &insertionStatus );
  if( insertionStatus > 0 )
  {
    kh_value( tasksList, newTaskIndex ) = (SignalIOTask) malloc( sizeof(SignalIOTaskData) );
    if( kh_value( tasksList, newTaskIndex ) == NULL )
    {
      DEBUG_PRINT( "loading task %s failed", taskConfig );
      EndTask( taskKey ); 
      return -1;
    }
        
    DEBUG_PRINT( "new key %d inserted (iterator: %u - total: %u)", kh_key( tasksList, newTaskIndex ), newTaskIndex, kh_size( tasksList ) );
  }
  else if( insertionStatus == 0 ) { DEBUG_PRINT( "task key %d already exists (iterator %u)", taskKey, newTaskIndex ); }
  
  return (int) kh_key( tasksList, newTaskIndex );
}

void EndTask( int taskID )
{
  khint_t taskIndex = kh_get( TaskInt, tasksList, (khint_t) taskID );
  if( taskIndex == kh_end( tasksList ) ) return;
  
  SignalIOTask task = kh_value( tasksList, taskIndex );
  
  if( IsTaskStillUsed( task ) ) return;
  
  free( task );
  
  kh_del( TaskInt, tasksList, (khint_t) taskID );
  
  if( kh_size( tasksList ) == 0 )
  {
    kh_destroy( TaskInt, tasksList );
    tasksList = NULL;
  }
}

size_t GetMaxInputSamplesNumber( int taskID )
{
  khint_t taskIndex = kh_get( TaskInt, tasksList, (khint_t) taskID );
  if( taskIndex == kh_end( tasksList ) ) return 0;
  
  return 1;
}

size_t Read( int taskID, unsigned int channel, double* ref_value )
{
  khint_t taskIndex = kh_get( TaskInt, tasksList, (khint_t) taskID );
  if( taskIndex == kh_end( tasksList ) ) return 0;
  
  SignalIOTask task = kh_value( tasksList, taskIndex );
  
  if( channel >= INPUT_CHANNELS_NUMBER ) return 0;
    
  *ref_value = (double) ( taskIndex + channel );

  return AQUISITION_BUFFER_LENGTH;
}

bool HasError( int taskID )
{
  khint_t taskIndex = kh_get( TaskInt, tasksList, (khint_t) taskID );
  if( taskIndex == kh_end( tasksList ) ) return false;
  
  SignalIOTask task = kh_value( tasksList, taskIndex );

  return false;
}

void Reset( int taskID )
{
  khint_t taskIndex = kh_get( TaskInt, tasksList, (khint_t) taskID );
  if( taskIndex == kh_end( tasksList ) ) return;
  
  SignalIOTask task = kh_value( tasksList, taskIndex );
  
  Timing.Delay( 200 );
}

bool AquireInputChannel( int taskID, unsigned int channel )
{
  khint_t taskIndex = kh_get( TaskInt, tasksList, (khint_t) taskID );
  if( taskIndex == kh_end( tasksList ) ) return false;
  
  //DEBUG_PRINT( "aquiring channel %u from task %d", channel, taskID );
  
  SignalIOTask task = kh_value( tasksList, taskIndex );
  
  if( channel >= INPUT_CHANNELS_NUMBER ) return false;
  
  if( task->inputChannelUsesList[ channel ] >= SIGNAL_INPUT_CHANNEL_MAX_USES ) return false;
  
  task->inputChannelUsesList[ channel ]++;
  
  return true;
}

void ReleaseInputChannel( int taskID, unsigned int channel )
{
  khint_t taskIndex = kh_get( TaskInt, tasksList, (khint_t) taskID );
  if( taskIndex == kh_end( tasksList ) ) return;
  
  SignalIOTask task = kh_value( tasksList, taskIndex );
  
  if( channel >= INPUT_CHANNELS_NUMBER ) return;
  
  if( task->inputChannelUsesList[ channel ] > 0 ) task->inputChannelUsesList[ channel ]--;
  
  if( !IsTaskStillUsed( task ) ) EndTask( taskID );
}

void EnableOutput( int taskID, bool enable )
{
  khint_t taskIndex = kh_get( TaskInt, tasksList, (khint_t) taskID );
  if( taskIndex == kh_end( tasksList ) ) return;
  
  SignalIOTask task = kh_value( tasksList, taskIndex );
  
  Timing.Delay( 200 );
}

bool IsOutputEnabled( int taskID )
{
  khint_t taskIndex = kh_get( TaskInt, tasksList, (khint_t) taskID );
  if( taskIndex == kh_end( tasksList ) ) return false;
  
  SignalIOTask task = kh_value( tasksList, taskIndex );
  
  return true;
}

bool Write( int taskID, unsigned int channel, double value )
{
  khint_t taskIndex = kh_get( TaskInt, tasksList, (khint_t) taskID );
  if( taskIndex == kh_end( tasksList ) ) return false;
  
  SignalIOTask task = kh_value( tasksList, taskIndex );
  
  return true;
}

bool AquireOutputChannel( int taskID, unsigned int channel )
{
  const int OPERATION_MODES[ OUTPUT_CHANNELS_NUMBER ] = { 0xFF, 0xFE, 0xFD };
  
  khint_t taskIndex = kh_get( TaskInt, tasksList, (khint_t) taskID );
  if( taskIndex == kh_end( tasksList ) ) return false;
  
  SignalIOTask task = kh_value( tasksList, taskIndex );
  
  if( channel >= OUTPUT_CHANNELS_NUMBER ) return false;
  
  if( task->isOutputChannelUsed ) return false;
  
  DEBUG_PRINT( "setting operation mode %X", OPERATION_MODES[ channel ] );
  
  task->isOutputChannelUsed = true;
  
  return true;
}

void ReleaseOutputChannel( int taskID, unsigned int channel )
{
  khint_t taskIndex = kh_get( TaskInt, tasksList, (khint_t) taskID );
  if( taskIndex == kh_end( tasksList ) ) return;
  
  SignalIOTask task = kh_value( tasksList, taskIndex );
  
  if( channel >= OUTPUT_CHANNELS_NUMBER ) return;
  
  task->isOutputChannelUsed = false;
  
  EndTask( taskID );
}

bool IsTaskStillUsed( SignalIOTask task )
{
  bool isStillUsed = false;
  if( task != NULL )
  {
    for( size_t channel = 0; channel < INPUT_CHANNELS_NUMBER; channel++ )
    {
      if( task->inputChannelUsesList[ channel ] > 0 )
      {
        isStillUsed = true;
        break;
      }
    }
  }
  
  if( task->isOutputChannelUsed ) isStillUsed = true;
  
  return isStillUsed;
}
