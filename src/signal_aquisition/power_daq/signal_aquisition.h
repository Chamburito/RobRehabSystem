#ifndef SIGNAL_AQUISITION_POWER_DAQ_H
#define SIGNAL_AQUISITION_POWER_DAQ_H

#include "axis/pci_daq_axis/powerdaq/win_sdk_types.h"
#include "axis/pci_daq_axis/powerdaq/powerdaq.h"
#include "axis/pci_daq_axis/powerdaq/powerdaq32.h"

#include "signal_aquisition/signal_aquisition_interface.h"

#include "klib/khash.h"
#include "file_parsing/json_parser.h"

#include "debug/async_debug.h"

static const int DAQ_ACQUIRE = 1;
static const int DAQ_RELEASE = 0;
static const int PDAQ_ENABLE = 1;
static const int PDAQ_DISABLE = 0;

typedef struct _SignalAquisitionTask
{
  int handle;
  Thread_Handle threadID;
  ThreadLock threadLock;
  bool isReading;
  size_t channelsNumber;
  bool* channelAvailableList;
  size_t aquisitionSamplesNumber;
  double* samplesList;
  double** samplesTable;
  size_t* aquiredSamplesCountList;
}
SignalAquisitionTask;

KHASH_MAP_INIT_STR( Task, SignalAquisitionTask )
static khash_t( Task )* tasksList = NULL;

static int InitTask( const char*, size_t );
static void EndTask( int );
static double* Read( int, unsigned int, size_t* );
static size_t GetChannelsNumber( int );
static size_t GetMaxSamplesNumber( int );

const SignalAquisitionOperations PowerDAQOperations = { .InitTask = InitTask, .EndTask = EndTask, .Read = Read, 
                                                        .GetChannelsNumber = GetChannelsNumber, .GetMaxSamplesNumber = GetMaxSamplesNumber };

static void* AsyncReadBuffer( void* );

static SignalAquisitionTask* LoadTaskData( const char* );
static void UnloadTaskData( SignalAquisitionTask* );

static int InitTask( const char* configFileName )
{
  SignalAquisitionTask* ref_newTaskData = LoadTaskData( configFileName );
  if( ref_newTaskData == NULL ) return -1;
  
  if( tasksList == NULL ) tasksList = kh_init( Task );
  
  int insertionStatus;
  khint_t newTaskID = kh_put( Task, tasksList, configFileName, &insertionStatus );
  if( insertionStatus > 0 )
  {
    SignalAquisitionTask* newTask = &(kh_value( tasksList, newTaskID ));
    memcpy( newTask, ref_newTaskData, sizeof(SignalAquisitionTask) );
  }
  else
  {
    UnloadTaskData( ref_newTaskData );
    if( insertionStatus == -1 ) return -1;
  }
  
  return (int) newTaskID;
}

static void EndTask( int taskID )
{
  if( !kh_exist( tasksList, (khint_t) taskID ) ) return;
  
  UnloadTaskData( &(kh_value( tasksList, (khint_t) taskID )) );
  
  kh_del( Task, tasksList, (khint_t) taskID );
  
  if( kh_size( tasksList ) == 0 )
  {
    kh_destroy( Task, tasksList );
    tasksList = NULL;
  }
}

static double* Read( int taskID, unsigned int channel, size_t* ref_aquiredSamplesCount )
{
  if( !kh_exist( tasksList, (khint_t) taskID ) ) return NULL;
  
  SignalAquisitionTask* task = &(kh_value( tasksList, (khint_t) taskID ));
  
  if( channel >= task->channelsNumber ) return NULL;
  
  if( !task->isReading ) return NULL;
 
  ThreadLock_Aquire( task->threadLock );
  
  double* aquiredSamplesList = ( task->aquiredSamplesCountList[ channel ] > 0 ) ? task->samplesTable[ channel ] : NULL;
  
  *ref_aquiredSamplesCount = (size_t) task->aquiredSamplesCountList[ channel ];
  task->aquiredSamplesCountList[ channel ] = 0;
    
  ThreadLock_Release( task->threadLock );
  
  return aquiredSamplesList;
}

static size_t GetChannelsNumber( int taskID )
{
  if( !kh_exist( tasksList, (khint_t) taskID ) ) return 0;
  
  return kh_value( tasksList, (khint_t) taskID ).channelsNumber;
}

size_t GetMaxSamplesNumber( int taskID )
{
  if( !kh_exist( tasksList, (khint_t) taskID ) ) return 0;
  
  return kh_value( tasksList, (khint_t) taskID ).aquisitionSamplesNumber;
}



static void* AsyncReadBuffer( void* callbackData )
{
  SignalAquisitionTask* task = (SignalAquisitionTask*) callbackData;
  
  size_t aquiredSamplesCount;
  
  while( task->isReading )
  {
    int readStatus = _PdAInGetSamples( task->handle, task->channelsNumber * task->aquisitionSamplesNumber, task->samplesList, &aquiredSamplesCount );
  
    if( readStatus <= 0 )
    {
      /*ERROR_EVENT*/DEBUG_PRINT( "error reading from board %d analog input %d: error code %d", 0, task->handle, readStatus );
      break;
    }
    
    ThreadLock_Aquire( task->threadLock );
    
    for( size_t channelIndex = 0; channelIndex < task->channelsNumber; channelIndex++ )
    {
      for( size_t sampleIndex = 0; sampleIndex < aquiredSamplesCount; sampleIndex++ )
        task->samplesTable[ channelIndex ][ sampleIndex ] = task->samplesList[ channelIndex * aquiredSamplesCount + sampleIndex ];
    }
    
    ThreadLock_Release( task->threadLock );
  }
  
  Thread_Exit( 0 );
  return NULL;
}

SignalAquisitionTask* LoadTaskData( const char* configFileName )
{
  static SignalAquisitionTask newTaskData;
  
  bool loadError = false;
  
  memset( &newTaskData, 0, sizeof(SignalAquisitionTask) );
  
  FileParser parser = JSONParser;
  int configFileID = parser.OpenFile( configFileName );
  if( configFileID != -1 )
  {
    int daqBoardID = parser.GetIntegerValue( "board_id" );
    if( (newTaskData.aquisitionSamplesNumber = (size_t) parser.GetIntegerValue( "samples_number" )) > 0 )
    {
      if( PdGetNumberAdapters() > 0 && daqBoardID < PdGetNumberAdapters() )
      {
        newTaskData.handle = PdAcquireSubsystem( daqBoardID, AnalogIn, DAQ_ACQUIRE );
        
        Adapter_Info adapterInfo;
        _PdGetAdapterInfo( daqBoardID, &adapterInfo );
        newTaskData.channelsNumber = adapterInfo.SSI[ AnalogIn ].dwChannels;
        
        _PdAInReset( newTaskData.handle );
        DWORD analogInputConfig = (AIN_BIPOLAR | AIN_CL_CLOCK_SW | AIN_CV_CLOCK_CONTINUOUS | AIN_SINGLE_ENDED | AIN_RANGE_10V);
        _PdAInSetCfg( newTaskData.handle, analogInputConfig, 0, 0 );
        DWORD channelsList[ newTaskData.channelsNumber ];
        for( size_t channelIndex = 0; channelIndex < newTaskData.channelsNumber; channelIndex++ )
          channelsList[ channelIndex ] = CHLIST_ENT( channelIndex, 1, 1 );//channelIndex | SLOW;
        _PdAInSetChList( newTaskData.handle, newTaskData.channelsNumber, channelsList );
        _PdAInEnableConv( newTaskData.handle, PDAQ_ENABLE );
        _PdAInSwStartTrig( newTaskData.handle );
        _PdAInSwClStart( newTaskData.handle );
        
        newTaskData.samplesList = (double*) calloc( newTaskData.channelsNumber * newTaskData.aquisitionSamplesNumber, sizeof(double) );
        newTaskData.samplesTable = (double**) calloc( newTaskData.channelsNumber, sizeof(double*) );
        for( size_t channelIndex = 0; channelIndex < newTaskData.channelsNumber; channelIndex++ )
          newTaskData.samplesTable[ channelIndex ] = (double*) calloc( newTaskData.aquisitionSamplesNumber, sizeof(double) );
        newTaskData.aquiredSamplesCountList = (size_t*) calloc( newTaskData.channelsNumber, sizeof(size_t) );
        
        newTaskData.isReading = true;
        if( (newTaskData.threadID = Thread_Start( AsyncReadBuffer, NULL, THREAD_JOINABLE )) == -1 ) loadError = true;
      }
      else loadError = true;
    }
    else loadError = true;
    
    parser.CloseFile( configFileID );
  }
  else
  {
    DEBUG_PRINT( "configuration for signal aquisition task %s not found", configFileName );
    return NULL;
  }
  
  if( loadError )
  {
    UnloadTaskData( &newTaskData );
    return NULL;
  }
  
  return &newTaskData;
}

void UnloadTaskData( SignalAquisitionTask* task )
{
  task->isReading = false;
  Thread_WaitExit( task->threadID, 5000 );
  
  ThreadLock_Discard( task->threadLock );
  
  _PdAInReset( task->handle );
  
  _PdAInEnableConv( task->handle, PDAQ_DISABLE );
  _PdAInSwStopTrig( task->handle );
    
  PdAcquireSubsystem( task->handle, AnalogIn, DAQ_RELEASE );
  
  free( task->samplesList );
  free( task->samplesTable );
  free( task->aquiredSamplesCountList );
}


#endif // SIGNAL_AQUISITION_POWER_DAQ_H
