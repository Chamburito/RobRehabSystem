#ifndef SIGNAL_AQUISITION_POWER_DAQ_H
#define SIGNAL_AQUISITION_POWER_DAQ_H

#include "axis/pci_daq_axis/powerdaq/win_sdk_types.h"
#include "axis/pci_daq_axis/powerdaq/powerdaq.h"
#include "axis/pci_daq_axis/powerdaq/powerdaq32.h"

#include "signal_aquisition/signal_aquisition_interface.h"

#include "klib/khash.h"
#include "file_parsing/json_parser.h"

#include "debug/async_debug.h"

typedef struct _SignalAquisitionTask
{
  Thread_Handle threadID;
  ThreadLock threadLock;
  bool isReading;
  size_t channelsNumber;
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
static double* Read( int, unsigned int );
static size_t GetChannelsNumber( int );
static size_t GetMaxSamplesNumber( int );

const SignalAquisitionOperations PowerDAQOperations = { .InitTask = InitTask, .EndTask = EndTask, .Read = Read, 
                                                        .GetChannelsNumber = GetChannelsNumber, .GetMaxSamplesNumber = GetMaxSamplesNumber };

static void* AsyncReadBuffer( void* );

static SignalAquisitionTask* LoadTaskData( const char*, size_t );
static void UnloadTaskData( SignalAquisitionTask* );

static int InitTask( const char* configFileName, size_t aquisitionSamplesNumber )
{
  SignalAquisitionTask* ref_newTaskData = LoadTaskData( configFileName, aquisitionSamplesNumber );
  if( ref_newTaskData == NULL ) return -1;
  
  if( tasksList == NULL ) tasksList = kh_init( Task );
  
  int insertionStatus;
  khint_t newTaskID = kh_put( Task, tasksList, configFileName, &insertionStatus );
  if( insertionStatus == -1 ) 
  {
    UnloadTaskData( ref_newTaskData );
    return -1;
  }
  
  SignalAquisitionTask* newTask = &(kh_value( tasksList, newTaskID ));
  
  memcpy( newTask, ref_newTaskData, sizeof(SignalAquisitionTask) );
  
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

static double* Read( int taskID, unsigned int channel )
{
  if( !kh_exist( tasksList, (khint_t) taskID ) ) return NULL;
  
  SignalAquisitionTask* task = &(kh_value( tasksList, (khint_t) taskID ));
  
  if( channel >= task->channelsNumber ) return NULL;
  
  if( !task->isReading ) return NULL;
 
  ThreadLock_Aquire( task->threadLock );
    
  double* aquiredSamplesList = (double*) task->samplesTable[ channel ];
  task->samplesTable[ channel ] = NULL;
    
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
  
  int32 aquiredSamplesNumber;
  
  while( task->isReading )
  {
    int errorCode = DAQmxReadAnalogF64( task->handle, task->aquisitionSamplesNumber, DAQmx_Val_WaitInfinitely, DAQmx_Val_GroupByChannel, 
                                        task->samplesList, task->channelsNumber * task->aquisitionSamplesNumber, &aquiredSamplesNumber, NULL );

    if( errorCode < 0 )
    {
      static char errorMessage[ DEBUG_MESSAGE_LENGTH ];
      DAQmxGetErrorString( errorCode, errorMessage, DEBUG_MESSAGE_LENGTH );
      ERROR_EVENT( "error aquiring EMG data: %s", errorMessage );
    
      break;
    }
    
    ThreadLock_Aquire( task->threadLock );
    
    for( size_t channel = 0; channel < task->channelsNumber; channel++ )
      task->samplesTable[ channel ] = task->samplesList + channel * aquiredSamplesNumber;
    
    ThreadLock_Release( task->threadLock );
  }
  
  Thread_Exit( 0 );
  return NULL;
}

SignalAquisitionTask* LoadTaskData( const char* configFileName, size_t aquisitionSamplesNumber )
{
  static SignalAquisitionTask newTaskData;
  
  bool loadError = false;
  
  memset( &newTaskData, 0, sizeof(SignalAquisitionTask) );
  
  FileParser parser = JSONParser;
  int configFileID = parser.OpenFile( configFileName );
  if( configFileID != -1 )
  {
    
  }
  else
  {
    
  }
  
  newTaskData.isReading = true;
  if( (newTaskData.threadID = Thread_Start( AsyncReadBuffer, NULL, THREAD_JOINABLE )) == -1 ) loadError = true;
  
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
  
  DAQmxStopTask( task->handle );
  DAQmxClearTask( task->handle );
  
  free( task->samplesList );
  free( task->samplesTable );
}


#endif // SIGNAL_AQUISITION_POWER_DAQ_H
