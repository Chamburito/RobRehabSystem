#ifndef SIGNAL_AQUISITION_NI_DAQMX_H
#define SIGNAL_AQUISITION_NI_DAQMX_H

#ifdef _CVI_
  #include <NIDAQmx.h>
#endif

#include "signal_aquisition/signal_aquisition_interface.h"

#include "klib/khash.h"

#include "debug/async_debug.h"

static const size_t AQUISITION_BUFFER_LENGTH = 10;

typedef struct _SignalAquisitionTask
{
  TaskHandle handle;
  Thread_Handle threadID;
  ThreadLock threadLock;
  bool isReading;
  uInt32 channelsNumber;
  float64* samplesList;
  float64** samplesTable;
  int32* aquiredSamplesCountList;
}
SignalAquisitionTask;

KHASH_MAP_INIT_STR( Task, SignalAquisitionTask )
static khash_t( Task )* tasksList = NULL;

static int InitTask( const char* );
static void EndTask( int );
static double* Read( int, unsigned int, size_t* );
static size_t GetChannelsNumber( int );
static size_t GetMaxSamplesNumber( int );

const SignalAquisitionOperations NIDAQmxOperations = { .InitTask = InitTask, .EndTask = EndTask, .Read = Read, 
                                                       .GetChannelsNumber = GetChannelsNumber, .GetMaxSamplesNumber = GetMaxSamplesNumber };
#ifdef _CVI_
static void* AsyncReadBuffer( void* );

static SignalAquisitionTask* LoadTaskData( const char* );
static void UnloadTaskData( SignalAquisitionTask* );

static int InitTask( const char* configFileName )
{
  static SignalAquisitionTask newTaskData;
  
  memset( &newTaskData, 0, sizeof(SignalAquisitionTask) );
  
  DAQmxLoadTask( configFileName, &(newTaskData.handle) );
  
  DAQmxGetTaskAttribute( newTaskData.handle, DAQmx_Task_NumChans, &(newTaskData.channelsNumber) );
  
  DEBUG_PRINT( "%u signal channels found", newTaskData.channelsNumber );
  
  newTaskData.samplesList = (float64*) calloc( newTaskData.channelsNumber * AQUISITION_BUFFER_LENGTH, sizeof(float64) );
  newTaskData.samplesTable = (float64**) calloc( newTaskData.channelsNumber, sizeof(float64*) );
  newTaskData.aquiredSamplesCountList = (int32*) calloc( newTaskData.channelsNumber, sizeof(int32) );
  
  newTaskData.threadLock = ThreadLock_Create();
  
  DAQmxStartTask( newTaskData.handle );
  
  newTaskData.isReading = true;
  newTaskData.threadID = Thread_Start( AsyncReadBuffer, NULL, THREAD_JOINABLE );
  
  if( tasksList == NULL ) tasksList = kh_init( Task );
  
  int insertionStatus;
  khint_t newTaskID = kh_put( Task, tasksList, configFileName, &insertionStatus );
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
  free( task->aquiredSamplesCountList );
  
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
    
  double* aquiredSamplesList = (double*) task->samplesTable[ channel ];
  task->samplesTable[ channel ] = NULL;
  
  *ref_aquiredSamplesCount = (size_t) task->aquiredSamplesCountList[ channel ];
  task->aquiredSamplesCountList[ channel ] = 0;
    
  ThreadLock_Release( task->threadLock );
  
  return aquiredSamplesList;
}

static size_t GetChannelsNumber( int taskID )
{
  if( !kh_exist( tasksList, (khint_t) taskID ) ) return 0;
  
  return (size_t) kh_value( tasksList, (khint_t) taskID ).channelsNumber;
}

static size_t GetMaxSamplesNumber( int taskID )
{
  if( !kh_exist( tasksList, (khint_t) taskID ) ) return 0;
  
  return AQUISITION_BUFFER_LENGTH;
}


static void* AsyncReadBuffer( void* callbackData )
{
  SignalAquisitionTask* task = (SignalAquisitionTask*) callbackData;
  
  int32 aquiredSamplesCount;
  
  while( task->isReading )
  {
    int errorCode = DAQmxReadAnalogF64( task->handle, AQUISITION_BUFFER_LENGTH, DAQmx_Val_WaitInfinitely, DAQmx_Val_GroupByChannel, 
                                        task->samplesList, task->channelsNumber * AQUISITION_BUFFER_LENGTH, &aquiredSamplesCount, NULL );

    if( errorCode < 0 )
    {
      static char errorMessage[ DEBUG_MESSAGE_LENGTH ];
      DAQmxGetErrorString( errorCode, errorMessage, DEBUG_MESSAGE_LENGTH );
      ERROR_EVENT( "error aquiring analog signal: %s", errorMessage );
    
      break;
    }
    
    ThreadLock_Aquire( task->threadLock );
    
    for( size_t channel = 0; channel < task->channelsNumber; channel++ )
    {
      task->samplesTable[ channel ] = task->samplesList + channel * aquiredSamplesCount;
      task->aquiredSamplesCountList[ channel ] = aquiredSamplesCount;
    }
    
    ThreadLock_Release( task->threadLock );
  }
  
  Thread_Exit( 0 );
  return NULL;
}
#endif


#endif // SIGNAL_AQUISITION_NI_DAQMX_H
