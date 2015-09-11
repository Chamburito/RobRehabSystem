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
  bool* channelUsedList;
  uInt32 channelsNumber;
  float64* samplesList;
  float64** samplesTable;
  int32* aquiredSamplesCountList;
}
SignalAquisitionTask;

KHASH_MAP_INIT_INT( TaskInt, SignalAquisitionTask* )
static khash_t( TaskInt )* tasksList = NULL;

static int InitTask( const char* );
static void EndTask( int );
static double* Read( int, unsigned int, size_t* );
static bool AquireChannel( int, unsigned int );
static void ReleaseChannel( int, unsigned int );
static size_t GetMaxSamplesNumber( int );

const SignalAquisitionOperations NIDAQmxOperations = { .InitTask = InitTask, .EndTask = EndTask, .Read = Read, .AquireChannel = AquireChannel,
                                                       .ReleaseChannel = ReleaseChannel, .GetMaxSamplesNumber = GetMaxSamplesNumber };
#ifdef _CVI_
static void* AsyncReadBuffer( void* );

static SignalAquisitionTask* LoadTaskData( const char* );
static void UnloadTaskData( SignalAquisitionTask* );

static int InitTask( const char* taskName )
{
  if( tasksList == NULL ) tasksList = kh_init( TaskInt );
  
  int taskKey = (int) kh_str_hash_func( taskName );
  
  int insertionStatus;
  khint_t newTaskID = kh_put( TaskInt, tasksList, taskKey, &insertionStatus );
  
  if( insertionStatus > 0 )
  {
    kh_value( tasksList, newTaskID ) = LoadTaskData( taskName );
    if( kh_value( tasksList, newTaskID ) == NULL )
    {
      DEBUG_PRINT( "loading task %s failed", taskName );
      EndTask( (int) newTaskID ); 
      return -1;
    }
        
    DEBUG_PRINT( "new key %d inserted (iterator: %u - total: %u)", kh_key( tasksList, newTaskID ), newTaskID, kh_size( tasksList ) );
  }
  else if( insertionStatus == 0 ) { DEBUG_PRINT( "task key %d already exists (iterator %u)", taskKey, newTaskID ); }
  
  return (int) newTaskID;
}

static void EndTask( int taskID )
{
  if( !kh_exist( tasksList, (khint_t) taskID ) ) return;
  
  SignalAquisitionTask* task = kh_value( tasksList, (khint_t) taskID );
  
  UnloadTaskData( task );
  
  kh_del( TaskInt, tasksList, (khint_t) taskID );
  
  if( kh_size( tasksList ) == 0 )
  {
    kh_destroy( TaskInt, tasksList );
    tasksList = NULL;
  }
}

static double* Read( int taskID, unsigned int channel, size_t* ref_aquiredSamplesCount )
{
  if( !kh_exist( tasksList, (khint_t) taskID ) ) return NULL;
  
  SignalAquisitionTask* task = kh_value( tasksList, (khint_t) taskID );
  
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

static bool AquireChannel( int taskID, unsigned int channel )
{
  if( !kh_exist( tasksList, (khint_t) taskID ) ) return false;
  
  DEBUG_PRINT( "aquiring channel %u from task %d", channel, kh_key( tasksList, (khint_t) taskID ) );
  
  SignalAquisitionTask* task = kh_value( tasksList, (khint_t) taskID );
  
  if( channel > task->channelsNumber ) return false;
  
  if( task->channelUsedList[ channel ] ) return false;
  
  task->channelUsedList[ channel ] = true;
  
  return true;
}

static void ReleaseChannel( int taskID, unsigned int channel )
{
  if( !kh_exist( tasksList, (khint_t) taskID ) ) return;
  
  SignalAquisitionTask* task = kh_value( tasksList, (khint_t) taskID );
  
  if( channel > task->channelsNumber ) return;
  
  task->channelUsedList[ channel ] = false;
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
      DEBUG_PRINT( "error aquiring analog signal: %s", errorMessage );
    
      //break;
    }
    
    ThreadLock_Aquire( task->threadLock );
    
    for( size_t channel = 0; channel < task->channelsNumber; channel++ )
    {
      task->samplesTable[ channel ] = task->samplesList + channel * aquiredSamplesCount;
      task->aquiredSamplesCountList[ channel ] = aquiredSamplesCount;
    }
    
    ThreadLock_Release( task->threadLock );
  }
  
  DEBUG_PRINT( "ending aquisition thread %x", THREAD_ID );
  
  Thread_Exit( 0 );
  return NULL;
}


SignalAquisitionTask* LoadTaskData( const char* taskName )
{
  bool loadError = false;
  
  SignalAquisitionTask* newTask = (SignalAquisitionTask*) malloc( sizeof(SignalAquisitionTask) );
  memset( newTask, 0, sizeof(SignalAquisitionTask) );
  
  if( DAQmxLoadTask( taskName, &(newTask->handle) ) >= 0 )
  {
    if( DAQmxGetTaskAttribute( newTask->handle, DAQmx_Task_NumChans, &(newTask->channelsNumber) ) >= 0 )
    {
      DEBUG_PRINT( "%u signal channels found", newTask->channelsNumber );
  
      newTask->channelUsedList = (bool*) calloc( newTask->channelsNumber, sizeof(bool) );
      
      newTask->samplesList = (float64*) calloc( newTask->channelsNumber * AQUISITION_BUFFER_LENGTH, sizeof(float64) );
      newTask->samplesTable = (float64**) calloc( newTask->channelsNumber, sizeof(float64*) );
      newTask->aquiredSamplesCountList = (int32*) calloc( newTask->channelsNumber, sizeof(int32) );
  
      if( DAQmxStartTask( newTask->handle ) >= 0 )
      {
        newTask->threadLock = ThreadLock_Create();
  
        newTask->isReading = true;
        if( (newTask->threadID = Thread_Start( AsyncReadBuffer, newTask, THREAD_JOINABLE )) == INVALID_THREAD_HANDLE ) loadError = true;
      }
      else
      {
        DEBUG_PRINT( "error starting task %s", taskName );
        loadError = true;
      }
    }
    else 
    {
      DEBUG_PRINT( "error getting task %s attribute", taskName );
      loadError = true;
    }
  }
  else 
  {
    DEBUG_PRINT( "error loading task %s", taskName );
    loadError = true;
  }
  
  if( loadError )
  {
    UnloadTaskData( newTask );
    return NULL;
  }
  
  return newTask;
}

void UnloadTaskData( SignalAquisitionTask* task )
{
  if( task == NULL ) return;
  
  bool stillUsed = false;
  if( task->channelUsedList != NULL )
  {
    for( size_t channel = 0; channel < task->channelsNumber; channel++ )
    {
      if( task->channelUsedList[ channel ] )
      {
        stillUsed = true;
        break;
      }
    }
  }
  
  if( stillUsed )
  {
    DEBUG_PRINT( "ending task with handle %d", task->handle );
    
    task->isReading = false;
    if( task->threadID != INVALID_THREAD_HANDLE ) Thread_WaitExit( task->threadID, 5000 );
  
    if( task->threadLock != -1 ) ThreadLock_Discard( task->threadLock );
  
    DAQmxStopTask( task->handle );
    DAQmxClearTask( task->handle );
  
    if( task->channelUsedList != NULL ) free( task->channelUsedList );
    
    if( task->samplesList != NULL ) free( task->samplesList );
    if( task->samplesTable != NULL ) free( task->samplesTable );
    if( task->aquiredSamplesCountList != NULL ) free( task->aquiredSamplesCountList );
  }
  
  free( task );
}

#endif


#endif // SIGNAL_AQUISITION_NI_DAQMX_H
