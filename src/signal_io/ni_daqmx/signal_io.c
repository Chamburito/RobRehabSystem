#include "signal_io/interface.h"

#include "klib/khash.h"

#include "debug/async_debug.h"

#include <NIDAQmx.h>

const size_t AQUISITION_BUFFER_LENGTH = 10;

const bool READ = true;
const bool WRITE = false;

typedef struct _SignalIOTaskData
{
  TaskHandle handle;
  Thread threadID;
  bool isRunning, mode;
  unsigned int* channelUsesList;
  Semaphore* channelLocksList;
  uInt32 channelsNumber;
  float64* samplesList;
  double* channelValuesList;
}
SignalIOTaskData;

typedef SignalIOTaskData* SignalIOTask;  

KHASH_MAP_INIT_INT( TaskInt, SignalIOTask )
static khash_t( TaskInt )* tasksList = NULL;

IMPLEMENT_INTERFACE( SIGNAL_IO_FUNCTIONS ) 

static void* AsyncReadBuffer( void* );
static void* AsyncWriteBuffer( void* );

static SignalIOTask LoadTaskData( const char* );
static void UnloadTaskData( SignalIOTask );

static bool CheckTask( SignalIOTask );

int InitTask( const char* taskName )
{
  if( tasksList == NULL ) tasksList = kh_init( TaskInt );
  
  int taskKey = (int) kh_str_hash_func( taskName );
  
  int insertionStatus;
  khint_t newTaskIndex = kh_put( TaskInt, tasksList, taskKey, &insertionStatus );
  if( insertionStatus > 0 )
  {
    kh_value( tasksList, newTaskIndex ) = LoadTaskData( taskName );
    if( kh_value( tasksList, newTaskIndex ) == NULL )
    {
      DEBUG_PRINT( "loading task %s failed", taskName );
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
  
  if( CheckTask( task ) ) return;
  
  UnloadTaskData( task );
  
  kh_del( TaskInt, tasksList, (khint_t) taskID );
  
  if( kh_size( tasksList ) == 0 )
  {
    kh_destroy( TaskInt, tasksList );
    tasksList = NULL;
  }
}

void Reset( int taskID )
{
  return;
}

bool HasError( int taskID )
{
  return false;
}

bool Read( int taskID, unsigned int channel, double* ref_value )
{
  khint_t taskIndex = kh_get( TaskInt, tasksList, (khint_t) taskID );
  if( taskIndex == kh_end( tasksList ) ) return false;
  
  SignalIOTask task = kh_value( tasksList, taskIndex );
  
  if( channel >= task->channelsNumber ) return false;
  
  if( !task->isRunning ) return false;
  
  if( task->mode == WRITE ) return false;
 
  DEBUG_PRINT( "%lu reads left of %u", Semaphores.GetCount( task->channelLocksList[ channel ] ), task->channelUsesList[ channel ] );
  
  Semaphores.Decrement( task->channelLocksList[ channel ] );
  
  DEBUG_PRINT( "%lu reads left of %u", Semaphores.GetCount( task->channelLocksList[ channel ] ), task->channelUsesList[ channel ] );
  
  *ref_value = task->channelValuesList[ channel ];
  
  return true;
}

bool AquireInputChannel( int taskID, unsigned int channel )
{
  khint_t taskIndex = kh_get( TaskInt, tasksList, (khint_t) taskID );
  if( taskIndex == kh_end( tasksList ) ) return false;
  
  SignalIOTask task = kh_value( tasksList, taskIndex );
  
  if( task->mode == WRITE ) return false;
  
  if( channel > task->channelsNumber ) return false;
  
  if( task->channelUsesList[ channel ] >= SIGNAL_INPUT_CHANNEL_MAX_USES ) return false;
  
  task->channelUsesList[ channel ]++;
  
  if( !task->isRunning ) task->threadID = Threading.StartThread( AsyncReadBuffer, task, THREAD_JOINABLE );
  
  return true;
}

void ReleaseInputChannel( int taskID, unsigned int channel )
{
  khint_t taskIndex = kh_get( TaskInt, tasksList, (khint_t) taskID );
  if( taskIndex == kh_end( tasksList ) ) return;
  
  SignalIOTask task = kh_value( tasksList, taskIndex );
  
  if( channel > task->channelsNumber ) return;
  
  if( task->channelUsesList[ channel ] > 0 ) task->channelUsesList[ channel ]--;
  
  (void) CheckTask( task );
}

void EnableOutput( int taskID, bool enable )
{
  return;
}

bool IsOutputEnabled( int taskID )
{
  return true;
}

bool Write( int taskID, unsigned int channel, double value )
{
  khint_t taskIndex = kh_get( TaskInt, tasksList, (khint_t) taskID );
  if( taskIndex == kh_end( tasksList ) ) return false;
  
  SignalIOTask task = kh_value( tasksList, taskIndex );
  
  if( !task->isRunning ) return false;
  
  if( task->mode == READ ) return false;
  
  if( channel > task->channelsNumber ) return false;
  
  Semaphores.Decrement( task->channelLocksList[ 0 ] );
  
  task->channelValuesList[ channel ] = value;
  
  Semaphores.SetCount( task->channelLocksList[ 0 ], 1 );
  
  return true;
}

bool AquireOutputChannel( int taskID, unsigned int channel )
{
  khint_t taskIndex = kh_get( TaskInt, tasksList, (khint_t) taskID );
  if( taskIndex == kh_end( tasksList ) ) return false;
  
  //DEBUG_PRINT( "aquiring channel %u from task %d", channel, taskID );
  
  SignalIOTask task = kh_value( tasksList, taskIndex );
  
  if( task->mode == READ ) return false;
  
  if( channel > task->channelsNumber ) return false;
  
  if( task->channelUsesList[ channel ] == 1 ) return false;
  
  task->channelUsesList[ channel ] = 1;
  
  if( !task->isRunning ) task->threadID = Threading.StartThread( AsyncWriteBuffer, task, THREAD_JOINABLE );
  
  return true;
}

void ReleaseOutputChannel( int taskID, unsigned int channel )
{
  khint_t taskIndex = kh_get( TaskInt, tasksList, (khint_t) taskID );
  if( taskIndex == kh_end( tasksList ) ) return;
  
  SignalIOTask task = kh_value( tasksList, taskIndex );
  
  if( !task->isRunning ) return;
  
  if( channel > task->channelsNumber ) return;
  
  task->channelUsesList[ channel ] = 0;
  
  (void) CheckTask( task );
}



static void* AsyncReadBuffer( void* callbackData )
{
  SignalIOTask task = (SignalIOTask) callbackData;
  
  int32 aquiredSamplesCount;
  
  task->isRunning = true;
  
  DEBUG_PRINT( "initializing read thread %lx", THREAD_ID );
  
  while( task->isRunning )
  {
    int errorCode = DAQmxReadAnalogF64( task->handle, AQUISITION_BUFFER_LENGTH, DAQmx_Val_WaitInfinitely, DAQmx_Val_GroupByChannel, 
                                        task->samplesList, task->channelsNumber * AQUISITION_BUFFER_LENGTH, &aquiredSamplesCount, NULL );

    if( errorCode < 0 )
    {
      static char errorMessage[ DEBUG_MESSAGE_LENGTH ];
      DAQmxGetErrorString( errorCode, errorMessage, DEBUG_MESSAGE_LENGTH );
    }
    else
    {
      for( unsigned int channel = 0; channel < task->channelsNumber; channel++ )
      {
        double* channelSamplesList = task->samplesList + channel * aquiredSamplesCount;
        double channelSamplesSum = 0.0;
        for( size_t sampleIndex = 0; sampleIndex < (size_t) aquiredSamplesCount; sampleIndex++ )
          channelSamplesSum += channelSamplesList[ sampleIndex ]; 
        task->channelValuesList[ channel ] = channelSamplesSum / aquiredSamplesCount;
        
        Semaphores.SetCount( task->channelLocksList[ channel ], task->channelUsesList[ channel ] );
      }
    }
  }
  
  DEBUG_PRINT( "ending aquisition thread %x", THREAD_ID );
  
  return NULL;
}

static void* AsyncWriteBuffer( void* callbackData )
{
  SignalIOTask task = (SignalIOTask) callbackData;
  
  int32 writtenSamplesCount;
  
  task->isRunning = true;
  
  while( task->isRunning )
  {
    Semaphores.Decrement( task->channelLocksList[ 0 ] );
    
    int errorCode = DAQmxWriteAnalogF64( task->handle, 1, 0, 0.1, DAQmx_Val_GroupByChannel, task->channelValuesList, &writtenSamplesCount, NULL );
    if( errorCode < 0 )
    {
      static char errorMessage[ DEBUG_MESSAGE_LENGTH ];
      DAQmxGetErrorString( errorCode, errorMessage, DEBUG_MESSAGE_LENGTH );
      DEBUG_PRINT( "error aquiring analog signal: %s", errorMessage );
    }
    
    Semaphores.SetCount( task->channelLocksList[ 0 ], 1 );
  }
  
  return NULL;
}

bool CheckTask( SignalIOTask task )
{
  bool isStillUsed = false;
  if( task->channelUsesList != NULL )
  {
    for( size_t channel = 0; channel < task->channelsNumber; channel++ )
    {
      if( task->channelUsesList[ channel ] > 0 )
      {
        isStillUsed = true;
        break;
      }
    }
  }
  
  if( !isStillUsed )
  {
    task->isRunning = false;
    if( task->threadID != INVALID_THREAD_HANDLE ) Threading.WaitExit( task->threadID, 5000 );
  }
  
  return isStillUsed;
}

SignalIOTask LoadTaskData( const char* taskName )
{
  bool loadError = false;
  
  SignalIOTask newTask = (SignalIOTask) malloc( sizeof(SignalIOTaskData) );
  memset( newTask, 0, sizeof(SignalIOTask) );
  
  if( DAQmxLoadTask( taskName, &(newTask->handle) ) >= 0 )
  {
    if( DAQmxGetTaskAttribute( newTask->handle, DAQmx_Task_NumChans, &(newTask->channelsNumber) ) >= 0 )
    {
      DEBUG_PRINT( "%u signal channels found", newTask->channelsNumber );
  
      newTask->channelUsesList = (unsigned int*) calloc( newTask->channelsNumber, sizeof(unsigned int) );
      memset( newTask->channelUsesList, 0, newTask->channelsNumber * sizeof(unsigned int) );
      
      newTask->samplesList = (float64*) calloc( newTask->channelsNumber * AQUISITION_BUFFER_LENGTH, sizeof(float64) );
      newTask->channelValuesList = (double*) calloc( newTask->channelsNumber, sizeof(double) );
  
      if( DAQmxStartTask( newTask->handle ) >= 0 )
      {
        uInt32 readChannelsNumber;
        DAQmxGetReadAttribute( newTask->handle, DAQmx_Read_NumChans, &readChannelsNumber );
        if( readChannelsNumber > 0 ) 
        {
          newTask->channelLocksList = (Semaphore*) calloc( newTask->channelsNumber, sizeof(double) );
          for( unsigned int channel = 0; channel < newTask->channelsNumber; channel++ )
            newTask->channelLocksList[ channel ] = Semaphores.Create( 0, SIGNAL_INPUT_CHANNEL_MAX_USES );
          
          newTask->mode = READ;
        }
        else 
        {
          newTask->channelLocksList = (Semaphore*) calloc( 1, sizeof(double) );
          newTask->channelLocksList[ 0 ] = Semaphores.Create( 0, SIGNAL_INPUT_CHANNEL_MAX_USES );
          
          newTask->mode = WRITE;
        }
        
        newTask->isRunning = false;
        newTask->threadID = INVALID_THREAD_HANDLE;
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

void UnloadTaskData( SignalIOTask task )
{
  if( task == NULL ) return;
  
  DEBUG_PRINT( "ending task with handle %d", task->handle );

  if( task->mode == READ )
  {
    for( unsigned int channel = 0; channel < task->channelsNumber; channel++ )
      Semaphores.Discard( task->channelLocksList[ channel ] );
  }
  else
    Semaphores.Discard( task->channelLocksList[ 0 ] );

  DAQmxStopTask( task->handle );
  DAQmxClearTask( task->handle );

  if( task->channelUsesList != NULL ) free( task->channelUsesList );

  if( task->samplesList != NULL ) free( task->samplesList );
  if( task->channelValuesList != NULL ) free( task->channelValuesList );
  if( task->channelLocksList != NULL ) free ( task->channelLocksList );
  
  free( task );
}
