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

KHASH_MAP_INIT_STR( Task, SignalAquisitionTask )
static khash_t( Task )* tasksList = NULL;

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
  DEBUG_PRINT( "list pointer: %p", tasksList );
  if( tasksList != NULL )
  {
    khint_t taskID = kh_get( Task, tasksList, taskName );
    DEBUG_PRINT( "task iterator %u returned", taskID );
    if( taskID != kh_end( tasksList ) ) 
    {
      DEBUG_PRINT( "task key %s already exists", taskName );
      return (int) taskID;
    }
  }
  else
  {
    DEBUG_PRINT( "creating hash table for first key %s", taskName );
    tasksList = kh_init( Task );
  }
  
  SignalAquisitionTask* ref_newTaskData = LoadTaskData( taskName );
  if( ref_newTaskData == NULL ) return -1;
  
  int insertionStatus;
  khint_t newTaskID = kh_put( Task, tasksList, taskName, &insertionStatus );
  if( insertionStatus > 0 ) 
  {
    SignalAquisitionTask* newTask = &(kh_value( tasksList, newTaskID ));
    memcpy( newTask, ref_newTaskData, sizeof(SignalAquisitionTask) );
    
    DEBUG_PRINT( "new key %s inserted (total: %u)", taskName, kh_size( tasksList ) );
  }
  else if( insertionStatus == -1 ) 
  {
    UnloadTaskData( &(kh_value( tasksList, newTaskID )) );
    if( kh_size( tasksList ) == 0 )
    {
      kh_destroy( Task, tasksList );
      tasksList = NULL;
    }
    return -1;
  }
  
  DEBUG_PRINT( "list pointer: %p", tasksList );
  
  return (int) newTaskID;
}

static void EndTask( int taskID )
{
  if( !kh_exist( tasksList, (khint_t) taskID ) ) return;
  
  SignalAquisitionTask* task = &(kh_value( tasksList, (khint_t) taskID ));
  
  UnloadTaskData( task );
  
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
 
  //ThreadLock_Aquire( task->threadLock );
    
  double* aquiredSamplesList = (double*) task->samplesTable[ channel ];
  task->samplesTable[ channel ] = NULL;
  
  *ref_aquiredSamplesCount = (size_t) task->aquiredSamplesCountList[ channel ];
  task->aquiredSamplesCountList[ channel ] = 0;
    
  //ThreadLock_Release( task->threadLock );
  
  return aquiredSamplesList;
}

static bool AquireChannel( int taskID, unsigned int channel )
{
  if( !kh_exist( tasksList, (khint_t) taskID ) ) return false;
  
  SignalAquisitionTask* task = &(kh_value( tasksList, (khint_t) taskID ));
  
  if( channel > task->channelsNumber ) return false;
  
  if( task->channelUsedList[ channel ] ) return false;
  
  task->channelUsedList[ channel ] = true;
  
  return true;
}

static void ReleaseChannel( int taskID, unsigned int channel )
{
  if( !kh_exist( tasksList, (khint_t) taskID ) ) return;
  
  SignalAquisitionTask* task = &(kh_value( tasksList, (khint_t) taskID ));
  
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
      ERROR_EVENT( "error aquiring analog signal: %s", errorMessage );
    
      break;
    }
    
    //ThreadLock_Aquire( task->threadLock );
    
    for( size_t channel = 0; channel < task->channelsNumber; channel++ )
    {
      task->samplesTable[ channel ] = task->samplesList + channel * aquiredSamplesCount;
      task->aquiredSamplesCountList[ channel ] = aquiredSamplesCount;
    }
    
    //ThreadLock_Release( task->threadLock );
  }
  
  Thread_Exit( 0 );
  return NULL;
}

SignalAquisitionTask* LoadTaskData( const char* taskName )
{
  static SignalAquisitionTask newTaskData;
  
  bool loadError = false;
  
  memset( &newTaskData, 0, sizeof(SignalAquisitionTask) );
  
  if( DAQmxLoadTask( taskName, &(newTaskData.handle) ) >= 0 )
  {
    if( DAQmxGetTaskAttribute( newTaskData.handle, DAQmx_Task_NumChans, &(newTaskData.channelsNumber) ) >= 0 )
    {
      DEBUG_PRINT( "%u signal channels found", newTaskData.channelsNumber );
  
      newTaskData.channelUsedList = (bool*) calloc( newTaskData.channelsNumber, sizeof(bool) );
      
      newTaskData.samplesList = (float64*) calloc( newTaskData.channelsNumber * AQUISITION_BUFFER_LENGTH, sizeof(float64) );
      newTaskData.samplesTable = (float64**) calloc( newTaskData.channelsNumber, sizeof(float64*) );
      newTaskData.aquiredSamplesCountList = (int32*) calloc( newTaskData.channelsNumber, sizeof(int32) );
  
      if( DAQmxStartTask( newTaskData.handle ) >= 0 )
      {
        if( (newTaskData.threadLock = ThreadLock_Create()) != -1 )
        {
          newTaskData.isReading = true;
          //if( (newTaskData.threadID = Thread_Start( AsyncReadBuffer, &newTaskData, THREAD_JOINABLE )) == -1 ) loadError = true;
        }
        else loadError = true;
      }
      else loadError = true;
    }
    else loadError = true;
  }
  else loadError = true;          
  
  if( loadError )
  {
    DEBUG_PRINT( "loading task %s failed", taskName );
    UnloadTaskData( &newTaskData );
    return NULL;
  }
  
  return &newTaskData;
}

void UnloadTaskData( SignalAquisitionTask* task )
{
  if( task == NULL ) return;
  
  bool stillUsed = false;
  for( size_t channel = 0; channel < task->channelsNumber; channel++ )
  {
    if( task->channelUsedList[ channel ] )
    {
      stillUsed = true;
      break;
    }
  }
  
  if( stillUsed )
  {
    task->isReading = false;
    //if( task->threadID != -1 ) Thread_WaitExit( task->threadID, 5000 );
  
    if( task->threadLock != -1 ) ThreadLock_Discard( task->threadLock );
  
    DAQmxStopTask( task->handle );
    DAQmxClearTask( task->handle );
  
    if( task->channelUsedList != NULL ) free( task->channelUsedList );
    
    if( task->samplesList != NULL ) free( task->samplesList );
    if( task->samplesTable != NULL ) free( task->samplesTable );
    if( task->aquiredSamplesCountList != NULL ) free( task->aquiredSamplesCountList );
  }
}

#endif


#endif // SIGNAL_AQUISITION_NI_DAQMX_H
