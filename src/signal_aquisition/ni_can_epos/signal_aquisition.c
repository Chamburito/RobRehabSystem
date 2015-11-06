enum { AXIS_ENCODER, AXIS_RPS, AXIS_CURRENT, AXIS_ANALOG, AXIS_CHANNELS_NUMBER };

enum States { READY_2_SWITCH_ON = 1, SWITCHED_ON = 2, OPERATION_ENABLED = 4, FAULT = 8, VOLTAGE_ENABLED = 16, 
              QUICK_STOPPED = 32, SWITCH_ON_DISABLE = 64, REMOTE_NMT = 512, TARGET_REACHED = 1024, SETPOINT_ACK = 4096 };

enum Controls { SWITCH_ON = 1, ENABLE_VOLTAGE = 2, QUICK_STOP = 4, ENABLE_OPERATION = 8, 
                NEW_SETPOINT = 16, CHANGE_IMMEDIATEDLY = 32, ABS_REL = 64, FAULT_RESET = 128, HALT = 256 };

enum { SDO_RX, SDO_TX };
enum { PDO01, PDO02 };

IMPLEMENT_INTERFACE( SIGNAL_AQUISITION_FUNCTIONS )

#include "signal_aquisition/signal_aquisition_interface.h"
#include "ni_can_epos_axis/can_network.h"

#include "klib/khash.h"

#include "debug/async_debug.h"

static const size_t AQUISITION_BUFFER_LENGTH = 1;

typedef struct _SignalAquisitionTask
{
  CANFrame* readFramesList[ 2 ];
  CANFrame* controlFramesList[ 2 ];
  uint16_t statusWord, controlWord;
  Thread threadID;
  ThreadLock threadLock;
  bool isReading;
  unsigned int channelUsesList[ AXIS_CHANNELS_NUMBER ];
  double measuresList[ AXIS_CHANNELS_NUMBER ];
  uint8_t payload[ 8 ];
}
SignalAquisitionTask;

KHASH_MAP_INIT_INT( TaskInt, SignalAquisitionTask* )
static khash_t( TaskInt )* tasksList = NULL;

IMPLEMENT_INTERFACE( SIGNAL_AQUISITION_FUNCTIONS ) 

static SignalAquisitionTask* LoadTaskData( const char* );
static void UnloadTaskData( SignalAquisitionTask* );

int InitTask( const char* taskConfig )
{
  if( tasksList == NULL ) tasksList = kh_init( TaskInt );
  
  int taskKey = (int) kh_str_hash_func( taskConfig );
  
  int insertionStatus;
  khint_t newTaskIndex = kh_put( TaskInt, tasksList, taskKey, &insertionStatus );
  if( insertionStatus > 0 )
  {
    kh_value( tasksList, newTaskIndex ) = LoadTaskData( taskConfig );
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
  
  SignalAquisitionTask* task = kh_value( tasksList, taskIndex );
  
  UnloadTaskData( task );
  
  kh_del( TaskInt, tasksList, (khint_t) taskID );
  
  if( kh_size( tasksList ) == 0 )
  {
    kh_destroy( TaskInt, tasksList );
    tasksList = NULL;
  }
}

double* Read( int taskID, unsigned int channel, size_t* ref_aquiredSamplesCount )
{
  khint_t taskIndex = kh_get( TaskInt, tasksList, (khint_t) taskID );
  if( taskIndex == kh_end( tasksList ) ) return NULL;
  
  SignalAquisitionTask* task = kh_value( tasksList, taskIndex );
  
  if( channel >= AXIS_CHANNELS_NUMBER ) return NULL;
  
  if( !task->isReading ) return NULL;
 
  ThreadLocks.Aquire( task->threadLock );
    
  double* aquiredSamplesList = (double*) task->samplesTable[ channel ];
  *ref_aquiredSamplesCount = 1;
    
  ThreadLocks.Release( task->threadLock );
  
  return aquiredSamplesList;
}

bool AquireChannel( int taskID, unsigned int channel )
{
  khint_t taskIndex = kh_get( TaskInt, tasksList, (khint_t) taskID );
  if( taskIndex == kh_end( tasksList ) ) return false;
  
  //DEBUG_PRINT( "aquiring channel %u from task %d", channel, taskID );
  
  SignalAquisitionTask* task = kh_value( tasksList, taskIndex );
  
  if( channel >= AXIS_CHANNELS_NUMBER ) return false;
  
  task->channelUsesList[ channel ]++;
  
  return true;
}

void ReleaseChannel( int taskID, unsigned int channel )
{
  khint_t taskIndex = kh_get( TaskInt, tasksList, (khint_t) taskID );
  if( taskIndex == kh_end( tasksList ) ) return;
  
  SignalAquisitionTask* task = kh_value( tasksList, taskIndex );
  
  if( channel >= AXIS_CHANNELS_NUMBER ) return;
  
  task->channelUsesList[ channel ]--;
}

static size_t GetMaxSamplesNumber( int taskID )
{
  khint_t taskIndex = kh_get( TaskInt, tasksList, (khint_t) taskID );
  if( taskIndex == kh_end( tasksList ) ) return 0;
  
  return 1;
}


static void* AsyncReadBuffer( void* callbackData )
{
  SignalAquisitionTask* task = (SignalAquisitionTask*) callbackData;
  
  int32 aquiredSamplesCount;
  
  while( task->isReading )
  {
    ThreadLocks.Aquire( task->threadLock ); // Trying to make Read call blocking
    
    CANNetwork_Sync();
  
    // Read values from PDO01 (Position, Current and Status Word) to buffer
    CANFrame_Read( task->readFramesList[ PDO01 ], task->payload );  
    // Update values from PDO01
    taks->measuresList[ AXIS_ENCODER ] = task->payload[ 3 ] * 0x1000000 + task->payload[ 2 ] * 0x10000 + task->payload[ 1 ] * 0x100 + task->payload[ 0 ];
    int currentHEX = task->payload[ 5 ] * 0x100 + task->payload[ 4 ];
    double currentMA = currentHEX - ( ( currentHEX >= 0x8000 ) ? 0xFFFF : 0 );
    task->measuresList[ AXIS_CURRENT ] = currentMA / 1000.0;
  
    //interface->statusWord = payload[ 7 ] * 0x100 + payload[ 6 ];
  
    // Read values from PDO02 (Velocity and Tension) to buffer
    CANFrame_Read( task->readFramesList[ PDO02 ], task->payload );  
    // Update values from PDO02
    double velocityRPM = task->payload[ 3 ] * 0x1000000 + task->payload[ 2 ] * 0x10000 + task->payload[ 1 ] * 0x100 + task->payload[ 0 ];
    task->measuresList[ AXIS_RPS ] = velocityRPM * 60.0;
    double analog = task->payload[ 5 ] * 0x100 + task->payload[ 4 ];
    task->measuresList[ AXIS_ANALOG ] = analog;
    
    ThreadLocks.Release( task->threadLock );
  }
  
  DEBUG_PRINT( "ending aquisition thread %x", THREAD_ID );
  
  return NULL;
}



SignalAquisitionTask* LoadTaskData( const char* taskConfig )
{
  bool loadError = false;
  
  SignalAquisitionTask* newTask = (SignalAquisitionTask*) malloc( sizeof(SignalAquisitionTask) );
  memset( newTask, 0, sizeof(SignalAquisitionTask) );
  
  unsigned int nodeID = (unsigned int) strtoul( taskConfig, NULL, 0 );
  
  DEBUG_PRINT( "trying to load CAN interface for node %u", nodeID );
  
  if( (newInterface->readFramesList[ frameID ] = CANNetwork_InitFrame( SDO, FRAME_OUT, nodeID )) == NULL ) loadError = true;
  if( (newInterface->readFramesList[ frameID ] = CANNetwork_InitFrame( SDO, FRAME_IN, nodeID )) == NULL ) loadError = true;
  if( (newInterface->readFramesList[ frameID ] = CANNetwork_InitFrame( SDO, FRAME_IN, nodeID )) == NULL ) loadError = true;
  if( (newInterface->readFramesList[ frameID ] = CANNetwork_InitFrame( SDO, FRAME_IN, nodeID )) == NULL ) loadError = true;
    
    sprintf( networkAddress, "%s_TX_%02u", CAN_FRAME_NAMES[ frameID ], nodeID );
    newInterface->writeFramesList[ frameID ] = CANNetwork_InitFrame( FRAME_OUT, "CAN2", networkAddress );
    
    if( newInterface->writeFramesList[ frameID ] == NULL )
      DEBUG_PRINT( "error creating frame %s for CAN interface %u", networkAddress, nodeID );
  }

  DEBUG_PRINT( "loaded CAN interface for node %u", nodeID );
  
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
        newTask->threadLock = ThreadLocks.Create();
  
        newTask->isReading = true;
        if( (newTask->threadID = Threading.StartThread( AsyncReadBuffer, newTask, THREAD_JOINABLE )) == INVALID_THREAD_HANDLE ) loadError = true;
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
  if( task->channelUsesList != NULL )
  {
    for( size_t channel = 0; channel < task->channelsNumber; channel++ )
    {
      if( task->channelUsesList[ channel ] > 0 )
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
    if( task->threadID != INVALID_THREAD_HANDLE ) Threading.WaitExit( task->threadID, 5000 );
  
    if( task->threadLock != -1 ) ThreadLocks.Discard( task->threadLock );
  
    DAQmxStopTask( task->handle );
    DAQmxClearTask( task->handle );
  
    if( task->channelUsedList != NULL ) free( task->channelUsedList );
    
    if( task->samplesList != NULL ) free( task->samplesList );
    if( task->samplesTable != NULL ) free( task->samplesTable );
    if( task->aquiredSamplesCountList != NULL ) free( task->aquiredSamplesCountList );
  }
  
  free( task );
}
