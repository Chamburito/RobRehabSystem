#include "signal_aquisition/signal_aquisition_interface.h"
#include "axis/ni_can_epos_axis/can_network.h"

#include "klib/khash.h"

#include "debug/async_debug.h"

enum { AXIS_ENCODER, AXIS_RPS, AXIS_CURRENT, AXIS_ANALOG, AXIS_CHANNELS_NUMBER };

enum States { READY_2_SWITCH_ON = 1, SWITCHED_ON = 2, OPERATION_ENABLED = 4, FAULT = 8, VOLTAGE_ENABLED = 16, 
              QUICK_STOPPED = 32, SWITCH_ON_DISABLE = 64, REMOTE_NMT = 512, TARGET_REACHED = 1024, SETPOINT_ACK = 4096 };

enum Controls { SWITCH_ON = 1, ENABLE_VOLTAGE = 2, QUICK_STOP = 4, ENABLE_OPERATION = 8, 
                NEW_SETPOINT = 16, CHANGE_IMMEDIATEDLY = 32, ABS_REL = 64, FAULT_RESET = 128, HALT = 256 };

//enum { SDO_RX, SDO_TX };
//enum { PDO01, PDO02 };

static const size_t AQUISITION_BUFFER_LENGTH = 1;

typedef struct _SignalAquisitionTaskData
{
  CANFrame readFramesList[ 2 ];
  CANFrame controlFramesList[ 2 ];
  uint16_t statusWord, controlWord;
  Thread threadID;
  bool isReading;
  unsigned int channelUsesList[ AXIS_CHANNELS_NUMBER ];
  Semaphore channelLocksList[ AXIS_CHANNELS_NUMBER ];
  double measuresList[ AXIS_CHANNELS_NUMBER ];
  uint8_t payload[ 8 ];
}
SignalAquisitionTaskData;

typedef SignalAquisitionTaskData* SignalAquisitionTask;

KHASH_MAP_INIT_INT( TaskInt, SignalAquisitionTask )
static khash_t( TaskInt )* tasksList = NULL;

IMPLEMENT_INTERFACE( SIGNAL_AQUISITION_FUNCTIONS ) 

static SignalAquisitionTask LoadTaskData( const char* );
static void UnloadTaskData( SignalAquisitionTask );

static void* AsyncReadBuffer( void* );
static inline bool IsTaskStillUsed( SignalAquisitionTask );

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
  
  SignalAquisitionTask task = kh_value( tasksList, taskIndex );
  
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
  
  SignalAquisitionTask task = kh_value( tasksList, taskIndex );
  
  if( channel >= AXIS_CHANNELS_NUMBER ) return NULL;
  
  if( !task->isReading ) return NULL;
 
  Semaphores.Decrement( task->channelLocksList[ channel ] );
    
  double* aquiredSamplesList = &(task->measuresList[ channel ]);
  *ref_aquiredSamplesCount = 1;
  
  return aquiredSamplesList;
}

bool HasError( int taskID )
{
  khint_t taskIndex = kh_get( TaskInt, tasksList, (khint_t) taskID );
  if( taskIndex == kh_end( tasksList ) ) return false;
  
  SignalAquisitionTask task = kh_value( tasksList, taskIndex );
  
  uint16_t statusWord = (uint16_t) CANNetwork_ReadSingleValue( task->controlFramesList[ 1 ], task->controlFramesList[ 0 ], 0xFFFF, 0xFF );

  return (bool) ( statusWord & FAULT );
}

void Reset( int taskID )
{
  khint_t taskIndex = kh_get( TaskInt, tasksList, (khint_t) taskID );
  if( taskIndex == kh_end( tasksList ) ) return;
  
  SignalAquisitionTask task = kh_value( tasksList, taskIndex );
  
  CANNetwork_WriteSingleValue( task->controlFramesList[ 1 ], 0xFFFF, 0xFF, FAULT_RESET );
}

bool AquireChannel( int taskID, unsigned int channel )
{
  khint_t taskIndex = kh_get( TaskInt, tasksList, (khint_t) taskID );
  if( taskIndex == kh_end( tasksList ) ) return false;
  
  //DEBUG_PRINT( "aquiring channel %u from task %d", channel, taskID );
  
  SignalAquisitionTask task = kh_value( tasksList, taskIndex );
  
  if( channel >= AXIS_CHANNELS_NUMBER ) return false;
  
  if( !task->isReading ) task->threadID = Threading.StartThread( AsyncReadBuffer, task, THREAD_JOINABLE );
  
  if( task->channelUsesList[ channel ] >= SIGNAL_AQUISITION_CHANNEL_MAX_USES ) return false;
  
  task->channelUsesList[ channel ]++;
  
  return true;
}

void ReleaseChannel( int taskID, unsigned int channel )
{
  khint_t taskIndex = kh_get( TaskInt, tasksList, (khint_t) taskID );
  if( taskIndex == kh_end( tasksList ) ) return;
  
  SignalAquisitionTask task = kh_value( tasksList, taskIndex );
  
  if( channel >= AXIS_CHANNELS_NUMBER ) return;
  
  if( task->channelUsesList[ channel ] > 0 ) task->channelUsesList[ channel ]--;
  
  if( IsTaskStillUsed( task ) )
  {
    if( task->isReading )
    {
      task->isReading = false;
      Threading.WaitExit( task->threadID, 5000 );
    }
    else
      EndTask( taskID );
  }
}

size_t GetMaxSamplesNumber( int taskID )
{
  khint_t taskIndex = kh_get( TaskInt, tasksList, (khint_t) taskID );
  if( taskIndex == kh_end( tasksList ) ) return 0;
  
  return 1;
}


static void* AsyncReadBuffer( void* callbackData )
{
  SignalAquisitionTask task = (SignalAquisitionTask) callbackData;
  
  task->isReading = true;
  
  while( task->isReading )
  { 
    CANNetwork_Sync();
  
    // Read values from PDO01 (Position, Current and Status Word) to buffer
    CANFrame_Read( task->readFramesList[ 0 ], task->payload );  
    // Update values from PDO01
    task->measuresList[ AXIS_ENCODER ] = task->payload[ 3 ] * 0x1000000 + task->payload[ 2 ] * 0x10000 + task->payload[ 1 ] * 0x100 + task->payload[ 0 ];
    int currentHEX = task->payload[ 5 ] * 0x100 + task->payload[ 4 ];
    double currentMA = currentHEX - ( ( currentHEX >= 0x8000 ) ? 0xFFFF : 0 );
    task->measuresList[ AXIS_CURRENT ] = currentMA / 1000.0;
  
    //interface->statusWord = payload[ 7 ] * 0x100 + payload[ 6 ];
  
    // Read values from PDO02 (Velocity and Tension) to buffer
    CANFrame_Read( task->readFramesList[ 1 ], task->payload );  
    // Update values from PDO02
    double velocityRPM = task->payload[ 3 ] * 0x1000000 + task->payload[ 2 ] * 0x10000 + task->payload[ 1 ] * 0x100 + task->payload[ 0 ];
    task->measuresList[ AXIS_RPS ] = velocityRPM * 60.0;
    double analog = task->payload[ 5 ] * 0x100 + task->payload[ 4 ];
    task->measuresList[ AXIS_ANALOG ] = analog;
    
    for( unsigned int channel = 0; channel < AXIS_CHANNELS_NUMBER; channel++ )
      Semaphores.SetCount( task->channelLocksList[ channel ], task->channelUsesList[ channel ] );
  }
  
  DEBUG_PRINT( "ending aquisition thread %lx", THREAD_ID );
  
  return NULL;
}

bool IsTaskStillUsed( SignalAquisitionTask task )
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
  
  return isStillUsed;
}

SignalAquisitionTask LoadTaskData( const char* taskConfig )
{
  bool loadError = false;
  
  SignalAquisitionTask newTask = (SignalAquisitionTask) malloc( sizeof(SignalAquisitionTaskData) );
  memset( newTask, 0, sizeof(SignalAquisitionTaskData) );
  
  unsigned int nodeID = (unsigned int) strtoul( taskConfig, NULL, 0 );
  
  DEBUG_PRINT( "trying to load CAN interface for node %u", nodeID );
  
  if( (newTask->controlFramesList[ 0 ] = CANNetwork_InitFrame( SDO, FRAME_OUT, nodeID )) == NULL ) loadError = true;
  if( (newTask->controlFramesList[ 1 ] = CANNetwork_InitFrame( SDO, FRAME_IN, nodeID )) == NULL ) loadError = true;
  if( (newTask->readFramesList[ 0 ] = CANNetwork_InitFrame( PDO01, FRAME_IN, nodeID )) == NULL ) loadError = true;
  if( (newTask->readFramesList[ 1 ] = CANNetwork_InitFrame( PDO02, FRAME_IN, nodeID )) == NULL ) loadError = true;
  
  for( unsigned int channel = 0; channel < AXIS_CHANNELS_NUMBER; channel++ )
    newTask->channelLocksList[ channel ] = Semaphores.Create( 0, SIGNAL_AQUISITION_CHANNEL_MAX_USES );
  
  if( loadError )
  {
    UnloadTaskData( newTask );
    return NULL;
  }
  
  return newTask;
}

void UnloadTaskData( SignalAquisitionTask task )
{
  if( task == NULL ) return;
  
  DEBUG_PRINT( "ending task %p", task );
  
  if( task->isReading )
  {
    task->isReading = false;
    Threading.WaitExit( task->threadID, 5000 );
  }
  
  for( unsigned int channel = 0; channel < AXIS_CHANNELS_NUMBER; channel++ )
    Semaphores.Discard( task->channelLocksList[ channel ] );
  
  CANNetwork_EndFrame( task->controlFramesList[ 0 ] );
  CANNetwork_EndFrame( task->controlFramesList[ 1 ] );
  CANNetwork_EndFrame( task->readFramesList[ 0 ] );
  CANNetwork_EndFrame( task->readFramesList[ 1 ] );
  
  free( task );
}
