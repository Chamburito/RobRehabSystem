#include "signal_io/power_daq/win_sdk_types.h"
#include "signal_io/power_daq/powerdaq.h"
#include "signal_io/power_daq/powerdaq32.h"

#include "signal_io/interface.h"

#include "klib/khash.h"

#include "debug/async_debug.h"

#define INPUT_CHANNELS_NUMBER 16
#define OUTPUT_CHANNELS_NUMBER 2

static const int DAQ_ACQUIRE = 1;
static const int DAQ_RELEASE = 0;
static const int PDAQ_ENABLE = 1;
static const int PDAQ_DISABLE = 0;

static const int IO_RANGE = 10;

typedef struct _SignalIOTaskData
{
  int inputHandle, outputHandle;
  uint16_t inputSamplesList[ INPUT_CHANNELS_NUMBER ];
  uint32_t aquiredSamplesCountList[ INPUT_CHANNELS_NUMBER ];
  int inputChannelUsesList[ INPUT_CHANNELS_NUMBER ];
  Semaphore inputChannelLocksList[ INPUT_CHANNELS_NUMBER ];
  uint32_t outputsList[ OUTPUT_CHANNELS_NUMBER ];
  bool isReading, isWriting;
}
SignalIOTaskData;

typedef SignalIOTaskData* SignalIOTask;

static SignalIOTask adaptersList = NULL;
static size_t adaptersNumber = 0;

static Thread updateThreadID;

DECLARE_MODULE_INTERFACE( SIGNAL_IO_INTERFACE ) 


static void* AsyncReadBuffer( void* );

int InitTask( const char* adapterConfig )
{
  if( PdGetNumberAdapters() <= 0 ) return SIGNAL_IO_TASK_INVALID_ID;
  
  size_t adapterIndex = strtoul( adapterConfig, NULL, 0 );
  if( adapterIndex >= (size_t) PdGetNumberAdapters() ) return SIGNAL_IO_TASK_INVALID_ID;
  
  Adapter_Info adapterInfo;
  _PdGetAdapterInfo( adapterIndex, &adapterInfo );
  if( !( adapterInfo.atType & atMF ) ) return SIGNAL_IO_TASK_INVALID_ID;
  
  if( adaptersNumber == 0 )
  {
    adaptersNumber = (size_t) PdGetNumberAdapters();
    adaptersList = (SignalIOTask) calloc( adaptersNumber, sizeof(SignalIOTaskData) );
    memset( adaptersList, 0, adaptersNumber * sizeof(SignalIOTaskData) );
    
    updateThreadID = Threading.StartThread( AsyncReadBuffer, NULL, THREAD_JOINABLE );
  }
  
  SignalIOTask newAdapter = (SignalIOTask) &(adaptersList[ adapterIndex ]);
  if( !newAdapter->isReading )
  {
    newAdapter->inputHandle = PdAcquireSubsystem( adapterIndex, AnalogIn, DAQ_ACQUIRE );
    newAdapter->outputHandle = PdAcquireSubsystem( adapterIndex, AnalogOut, DAQ_ACQUIRE );    
        
    Reset( adapterIndex );
    
    for( size_t channel = 0; channel < INPUT_CHANNELS_NUMBER; channel++ )
      newAdapter->inputChannelLocksList[ channel ] = Semaphores.Create( 0, SIGNAL_INPUT_CHANNEL_MAX_USES );
    
    newAdapter->isReading = true;
  }
  
  return (int) adapterIndex;
}

void EndTask( int adapterIndex )
{
  if( (size_t) adapterIndex >= adaptersNumber ) return;
  
  SignalIOTask adapter = (SignalIOTask) &(adaptersList[ adapterIndex ]);
  
  for( size_t channel = 0; channel < INPUT_CHANNELS_NUMBER; channel++ )
  {
    if( adapter->inputChannelUsesList[ channel ] > 0 ) return;
  }
  
  for( size_t channel = 0; channel < INPUT_CHANNELS_NUMBER; channel++ )
    Semaphores.Discard( adapter->inputChannelLocksList[ channel ] );
  
  _PdAInReset( adapter->inputHandle );
  _PdAOutReset( adapter->outputHandle );
  PdAcquireSubsystem( adapter->inputHandle, AnalogIn, DAQ_RELEASE );
  PdAcquireSubsystem( adapter->outputHandle, AnalogOut, DAQ_RELEASE );
  
  adapter->isReading = false;
  
  for( adapterIndex = 0; adapterIndex < (int) adaptersNumber; adapterIndex++ )
  {
    if( adaptersList[ adapterIndex ].isReading ) return;
  }
  
  free( adaptersList );
  adaptersList = NULL;
  adaptersNumber = 0;
}

size_t GetMaxInputSamplesNumber( int adapterIndex )
{
  if( (size_t) adapterIndex >= adaptersNumber ) return 0;
  
  return 1;
}

size_t Read( int adapterIndex, unsigned int channel, double* ref_value )
{
  if( (size_t) adapterIndex >= adaptersNumber ) return 0;
  
  SignalIOTask adapter = (SignalIOTask) &(adaptersList[ adapterIndex ]);
  
  if( channel >= INPUT_CHANNELS_NUMBER ) return 0;
  
  if( !adapter->isReading ) return 0;
 
  Semaphores.Decrement( adapter->inputChannelLocksList[ channel ] );
  
  size_t aquiredSamplesCount = adapter->aquiredSamplesCountList[ channel ];
  if( aquiredSamplesCount > 0 && ref_value != NULL )
  {
    uint16_t rawValue = adapter->inputSamplesList[ channel ];
    *ref_value = rawValue * IO_RANGE / 0x8000;
    if( rawValue > 0x8000 ) *ref_value -= ( 2.0 * IO_RANGE );
  }
  
  return aquiredSamplesCount;
}

bool HasError( int adapterIndex )
{
  if( (size_t) adapterIndex >= adaptersNumber ) return false;
  
  // Error detection

  return false;
}

void Reset( int adapterIndex )
{
  if( (size_t) adapterIndex >= adaptersNumber ) return;
  
  SignalIOTask adapter = (SignalIOTask) &(adaptersList[ adapterIndex ]);
  
  _PdAInReset( adapter->inputHandle );
  DWORD analogInputConfig = (AIN_BIPOLAR | AIN_CL_CLOCK_SW | AIN_CV_CLOCK_CONTINUOUS | AIN_SINGLE_ENDED | AIN_RANGE_10V);
  _PdAInSetCfg( adapter->inputHandle, analogInputConfig, 0, 0 );
  DWORD channelsList[ INPUT_CHANNELS_NUMBER ];
  for( size_t channel = 0; channel < INPUT_CHANNELS_NUMBER; channel++ )
    channelsList[ channel ] = CHLIST_ENT( channel % 16, 1, 1 );//( channel % 16 ) | SLOW;
  _PdAInSetChList( adapter->inputHandle, INPUT_CHANNELS_NUMBER, channelsList );
  _PdAInEnableConv( adapter->inputHandle, PDAQ_ENABLE );
  _PdAInSwStartTrig( adapter->inputHandle );
  _PdAInSwClStart( adapter->inputHandle );
        
  _PdAOutReset( adapter->outputHandle );
}

bool AquireInputChannel( int adapterIndex, unsigned int channel )
{
  if( (size_t) adapterIndex >= adaptersNumber ) return false;
  
  SignalIOTask adapter = (SignalIOTask) &(adaptersList[ adapterIndex ]);
  
  if( channel >= INPUT_CHANNELS_NUMBER ) return false;
  
  if( adapter->inputChannelUsesList[ channel ] >= SIGNAL_INPUT_CHANNEL_MAX_USES ) return false;
  
  adapter->inputChannelUsesList[ channel ]++;
  
  return true;
}

void ReleaseInputChannel( int adapterIndex, unsigned int channel )
{
  if( (size_t) adapterIndex >= adaptersNumber ) return;
  
  SignalIOTask adapter = (SignalIOTask) &(adaptersList[ adapterIndex ]);
  
  if( channel >= INPUT_CHANNELS_NUMBER ) return;
  
  if( adapter->inputChannelUsesList[ channel ] > 0 ) adapter->inputChannelUsesList[ channel ]--;
  
  //if( !IsTaskStillUsed( board ) && !board->isReading ) EndTask( boardID );
}

void EnableOutput( int adapterIndex, bool enable )
{
  if( (size_t) adapterIndex >= adaptersNumber ) return;
  
  adaptersList[ adapterIndex ].isWriting = enable;
}

bool IsOutputEnabled( int adapterIndex )
{
  if( (size_t) adapterIndex >= adaptersNumber ) return false;
  
  return adaptersList[ adapterIndex ].isWriting;
}

bool Write( int adapterIndex, unsigned int channel, double value )
{
  if( (size_t) adapterIndex >= adaptersNumber ) return false;
  
  if( channel >= OUTPUT_CHANNELS_NUMBER ) return false;
  
  SignalIOTask adapter = (SignalIOTask) &(adaptersList[ adapterIndex ]);
  
  if( value < -IO_RANGE ) value = -IO_RANGE;
  else if( value < IO_RANGE ) value = IO_RANGE;
  
  adapter->outputsList[ channel ] = (uint32_t) ( ( value + IO_RANGE ) / ( 2 * IO_RANGE ) ) * 0xFFF;
  
  _PdAOutPutValue( adapter->outputHandle, adapter->outputsList[ 0 ] << 12 | adapter->outputsList[ 1 ] );
  
  return false;
}

bool AquireOutputChannel( int adapterIndex, unsigned int channel )
{  
  if( (size_t) adapterIndex >= adaptersNumber ) return false;
  
  return true;
}

void ReleaseOutputChannel( int adapterIndex, unsigned int channel )
{
  return;
}

size_t GetChannelsNumber( int adapterIndex )
{
  if( (size_t) adapterIndex >= adaptersNumber ) return 0;
  
  return INPUT_CHANNELS_NUMBER;
}


static void* AsyncReadBuffer( void* callbackData )
{
  SignalIOTask adapter = (SignalIOTask) callbackData;
  
  uint32_t aquiredSamplesCount;
  
  while( adapter->isReading )
  {
    int readStatus = _PdAInGetSamples( adapter->inputHandle, INPUT_CHANNELS_NUMBER, adapter->inputSamplesList, &aquiredSamplesCount );
  
    if( readStatus <= 0 )
    {
      /*ERROR_EVENT*/DEBUG_PRINT( "error reading from board %d analog input %d: error code %d", 0, task->handle, readStatus );
      break;
    }
    
    ThreadLocks.Aquire( task->threadLock );
    
    for( size_t channel = 0; channel < task->inputChannelsNumber; channel++ )
    {
      for( size_t sampleIndex = 0; sampleIndex < aquiredSamplesCount; sampleIndex++ )
        task->inputSamplesTable[ channel ][ sampleIndex ] = task->inputSamplesList[ channel * aquiredSamplesCount + sampleIndex ];
    }
    
    ThreadLocks.Release( task->threadLock );
  }
  
  return NULL;
}
