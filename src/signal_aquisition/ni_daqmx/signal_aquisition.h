#ifndef SIGNAL_AQUISITION_NI_DAQMX_H
#define SIGNAL_AQUISITION_NI_DAQMX_H

#include <NIDAQmx.h>

#include "signal_aquisition/signal_aquisition_interface.h"

#include "debug/async_debug.h"

typedef struct _SignalAquisitionTask
{
  TaskHandle handle;
  Thread_Handle threadID;
  ThreadLock threadLock;
  bool isReading;
  static uInt32 channelsNumber;
  static size_t aquisitionSamplesNumber;
  static float64* samplesList;
  static float64** samplesTable;
}
SignalAquisitionTask;

KHASH_MAP_INIT_INT( Task, SignalAquisitionTask )
static khash_t( Task ) tasksList = NULL;

static int InitTask( const char*, size_t );
static void EndTask( int );
static double* Read( int, unsigned int );
static size_t GetChannelsNumber( int );

const SignalAquisitionOperations NIDAQmxOperations = { .InitTask = InitTask, .EndTask = EndTask, .Read = Read, .GetChannelsNumber = GetChannelsNumber };

static void* AsyncReadBuffer( void* );

static int InitTask( const char* configFileName, size_t aquisitionSamplesNumber )
{
  static SignalAquisitionTask newTaskData;
  
  memset( &newTaskData, 0, sizeof(SignalAquisitionTask) );
  
  DAQmxLoadTask( configFileName, &(newTaskData.handle) );
  
  DAQmxGetTaskAttribute( newTaskData.handle, DAQmx_Task_NumChans, &(newTaskData.channelsNumber) );
  
  DEBUG_PRINT( "%u signal channels found", newTaskData.channelsNumber );
  
  newTaskData.aquisitionSamplesNumber = aquisitionSamplesNumber;
  
  newTaskData.samplesList = (float64*) calloc( newTaskData.channelsNumber * newTaskData.aquisitionSamplesNumber, sizeof(float64) );
  newTaskData.samplesTable = (float64**) calloc( newTaskData.channelsNumber, sizeof(float64*) );
  
  newTaskData.threadLock = ThreadLock_Create();
  
  DAQmxStartTask( newTaskData.handle );
  
  newTaskData.isReading = true;
  newTaskData.threadID = Thread_Start( AsyncReadBuffer, NULL, THREAD_JOINABLE );
  
  if( tasksList == NULL ) tasksList = kh_init( Task );
  
  int insertionStatus;
  khint_t newTaskID = kh_put( Task, tasksList, (int) configFileName, &insertionStatus );
  if( insertionStatus == -1 ) return -1;
  
  SignalAquisitionTask* newTask = &(kh_value( tasksList, newTaskID ));
  
  memcpy( newTask, &newTaskData, sizeof(SignalAquisitionTask) );
  
  return (int) newTaskID;
}

static void EndTask( int taskID )
{
  if( !kh_exist( tasksList, (khint_t) taskID ) ) return;
  
  SignalAquisitionTask* task = &(kh_value( tasksList, (khint_t) taskID ));
  
  task->isReading = false;
  Thread_WaitExit( task->threadID, 5000 );
  
  ThreadLock_Discard( task->threadLock );
  
  DAQmxStopTask( task->handle );
  DAQmxClearTask( task->handle );
  
  free( task->samplesList );
  free( task->samplesTable );
  
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
  
  return (size_t) kh_value( tasksList, (khint_t) taskID ).channelsNumber;
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

#endif // SIGNAL_AQUISITION_NI_DAQMX_H
