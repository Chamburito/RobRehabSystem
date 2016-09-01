////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  Copyright (c) 2016 Leonardo José Consoni                                  //
//                                                                            //
//  This file is part of RobRehabSystem.                                      //
//                                                                            //
//  RobRehabSystem is free software: you can redistribute it and/or modify    //
//  it under the terms of the GNU Lesser General Public License as published  //
//  by the Free Software Foundation, either version 3 of the License, or      //
//  (at your option) any later version.                                       //
//                                                                            //
//  RobRehabSystem is distributed in the hope that it will be useful,         //
//  but WITHOUT ANY WARRANTY; without even the implied warranty of            //
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the              //
//  GNU Lesser General Public License for more details.                       //
//                                                                            //
//  You should have received a copy of the GNU Lesser General Public License  //
//  along with RobRehabSystem. If not, see <http://www.gnu.org/licenses/>.    //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////


#include <signal.h>
#include <stdint.h>

#include "signal_io/power_daq/win_sdk_types.h"
#include "signal_io/power_daq/powerdaq.h"
#include "signal_io/power_daq/powerdaq32.h"

#include "signal_io/interface.h"

#include "klib/khash.h"

#include "debug/async_debug.h"

#define INPUT_CHANNELS_NUMBER 8
#define RAW_INPUT_CHANNELS_NUMBER ( 2 * INPUT_CHANNELS_NUMBER )
#define OUTPUT_CHANNELS_NUMBER 2

static const int DAQ_ACQUIRE = 1;
static const int DAQ_RELEASE = 0;
static const int PDAQ_ENABLE = 1;
static const int PDAQ_DISABLE = 0;

static const double IO_RANGE = 10.0;

static const DWORD ANALOG_INPUT_CONFIG = (AIN_BIPOLAR | AIN_CL_CLOCK_SW | AIN_CV_CLOCK_CONTINUOUS | AIN_SINGLE_ENDED | AIN_RANGE_10V);

typedef struct _IOAdapterData
{
  int inputHandle, outputHandle;
  double inputSamplesList[ INPUT_CHANNELS_NUMBER ];
  uint32_t aquiredSamplesCountList[ INPUT_CHANNELS_NUMBER ];
  int inputChannelUsesList[ INPUT_CHANNELS_NUMBER ];
  Semaphore inputChannelLocksList[ INPUT_CHANNELS_NUMBER ];
  DWORD outputsList[ OUTPUT_CHANNELS_NUMBER ];
  bool isReading, isWriting;
}
IOAdapterData;

typedef IOAdapterData* IOAdapter;

static IOAdapter adaptersList = NULL;
static size_t adaptersNumber = 0;

static Thread updateThreadID = NULL;
static volatile bool isUpdating = false;

DECLARE_MODULE_INTERFACE( SIGNAL_IO_INTERFACE ) 


static void* AsyncReadBuffer( void* );

int InitTask( const char* adapterConfig )
{
  if( PdGetNumberAdapters() <= 0 ) return SIGNAL_IO_TASK_INVALID_ID;
  
  size_t adapterIndex = strtoul( adapterConfig, NULL, 0 );
  if( adapterIndex >= (size_t) PdGetNumberAdapters() ) return SIGNAL_IO_TASK_INVALID_ID;
  
  Adapter_Info adapterInfo;
  _PdGetAdapterInfo( adapterIndex, &adapterInfo );
  DEBUG_PRINT( "PD2-MFx board ?: %s", ( adapterInfo.atType & atMF ) ? "true" : "false" );
  if( !( adapterInfo.atType & atMF ) ) return SIGNAL_IO_TASK_INVALID_ID;
  
  if( adaptersNumber == 0 )
  {
    adaptersNumber = (size_t) PdGetNumberAdapters();
    adaptersList = (IOAdapter) calloc( adaptersNumber, sizeof(IOAdapterData) );
    memset( adaptersList, 0, adaptersNumber * sizeof(IOAdapterData) );
    
    updateThreadID = Threading.StartThread( AsyncReadBuffer, NULL, THREAD_JOINABLE );
  }
  
  IOAdapter newAdapter = (IOAdapter) &(adaptersList[ adapterIndex ]);
  if( !newAdapter->isReading )
  {
    DEBUG_PRINT( "aquiring subsystems from board: %d", adapterIndex );
    newAdapter->inputHandle = PdAcquireSubsystem( (int) adapterIndex, AnalogIn, DAQ_ACQUIRE );
    DEBUG_PRINT( "error aquiring input subsystem: %d", newAdapter->inputHandle );
    newAdapter->outputHandle = PdAcquireSubsystem( (int) adapterIndex, AnalogOut, DAQ_ACQUIRE ); 
    DEBUG_PRINT( "error aquiring output subsystem: %d", newAdapter->outputHandle );   
        
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
  
  IOAdapter adapter = (IOAdapter) &(adaptersList[ adapterIndex ]);
  
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
  
  isUpdating = false;
  Threading.WaitExit( updateThreadID, 5000 );
  updateThreadID = NULL;
  
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
  
  IOAdapter adapter = (IOAdapter) &(adaptersList[ adapterIndex ]);
  
  if( channel >= INPUT_CHANNELS_NUMBER ) return 0;
  
  if( !adapter->isReading ) return 0;
 
  Semaphores.Decrement( adapter->inputChannelLocksList[ channel ] );
  
  size_t aquiredSamplesCount = adapter->aquiredSamplesCountList[ channel ];
  if( aquiredSamplesCount > 0 && ref_value != NULL ) *ref_value = adapter->inputSamplesList[ channel ];
  
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
  
  IOAdapter adapter = (IOAdapter) &(adaptersList[ adapterIndex ]);
  
  _PdAInReset( adapter->inputHandle );
  _PdAInSetCfg( adapter->inputHandle, ANALOG_INPUT_CONFIG, 0, 0 );
  DWORD channelsList[ RAW_INPUT_CHANNELS_NUMBER ] = { 0 };
  for( size_t channel = 0; channel < RAW_INPUT_CHANNELS_NUMBER; channel++ )
    channelsList[ channel ] = ( CHAN(channel) | SLOW );
  _PdAInSetChList( adapter->inputHandle, RAW_INPUT_CHANNELS_NUMBER, channelsList );
  _PdAInEnableConv( adapter->inputHandle, PDAQ_ENABLE );
  _PdAInSwStartTrig( adapter->inputHandle );
  _PdAInSwClStart( adapter->inputHandle );
        
  _PdAOutReset( adapter->outputHandle );
}

bool AquireInputChannel( int adapterIndex, unsigned int channel )
{
  if( (size_t) adapterIndex >= adaptersNumber ) return false;
  
  IOAdapter adapter = (IOAdapter) &(adaptersList[ adapterIndex ]);
  
  if( channel >= INPUT_CHANNELS_NUMBER ) return false;
  
  if( adapter->inputChannelUsesList[ channel ] >= SIGNAL_INPUT_CHANNEL_MAX_USES ) return false;
  
  adapter->inputChannelUsesList[ channel ]++;
  
  return true;
}

void ReleaseInputChannel( int adapterIndex, unsigned int channel )
{
  if( (size_t) adapterIndex >= adaptersNumber ) return;
  
  IOAdapter adapter = (IOAdapter) &(adaptersList[ adapterIndex ]);
  
  if( channel >= INPUT_CHANNELS_NUMBER ) return;
  
  if( adapter->inputChannelUsesList[ channel ] > 0 ) adapter->inputChannelUsesList[ channel ]--;
  
  //EndTask( boardID );
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
  const double TENSION_2_TORQUE_RATIO = 0.027;
  
  if( (size_t) adapterIndex >= adaptersNumber ) return false;
  
  if( channel >= OUTPUT_CHANNELS_NUMBER ) return false;
  
  IOAdapter adapter = (IOAdapter) &(adaptersList[ adapterIndex ]);
  
  if( !adapter->isWriting ) return false;
  
  value /= TENSION_2_TORQUE_RATIO;
  if( value < (double) -IO_RANGE ) value = -IO_RANGE;
  else if( value < (double) IO_RANGE ) value = IO_RANGE;
  
  adapter->outputsList[ channel ] = ( value + IO_RANGE ) / ( 2 * IO_RANGE ) * 0xFFF;
  
  //if( _PdAOutPutValue( adapter->outputHandle, adapter->outputsList[ 0 ] << 12 | adapter->outputsList[ 1 ] ) < 0 )
  //  return false;
  
  return true;
}

bool AquireOutputChannel( int adapterIndex, unsigned int channel )
{  
  if( (size_t) adapterIndex >= adaptersNumber ) return false;
  
  if( channel >= OUTPUT_CHANNELS_NUMBER ) return false;
  
  return true;
}

void ReleaseOutputChannel( int adapterIndex, unsigned int channel )
{
  return;
}



static void* AsyncReadBuffer( void* callbackData )
{
  const double TENSION_2_CURRENT_RATIO = 2.35;
  //const double TENSION_2_CURRENT_OFFSET = 0.1;
  const double CURRENT_2_TORQUE_RATIO = 0.068;
  
  uint16_t rawInputSamplesList[ RAW_INPUT_CHANNELS_NUMBER ];
  double tensionsList[ RAW_INPUT_CHANNELS_NUMBER ];
  DWORD aquiredSamplesCount;
  
  isUpdating = true;
  while( isUpdating )
  {
    for( size_t adapterIndex = 0; adapterIndex < adaptersNumber; adapterIndex++ )
    {
      IOAdapter adapter = (IOAdapter) &(adaptersList[ adapterIndex ]);
      
      if( !adapter->isReading ) continue;
      
      int readStatus = _PdAInGetSamples( adapter->inputHandle, RAW_INPUT_CHANNELS_NUMBER, rawInputSamplesList, &aquiredSamplesCount );
      readStatus = PdAInRawToVolts( adapterIndex, ANALOG_INPUT_CONFIG, rawInputSamplesList, tensionsList, RAW_INPUT_CHANNELS_NUMBER );
      if( readStatus < 0 )
      {
        ERROR_EVENT( "error reading from input handle %d: error code %d", adapter->inputHandle, readStatus );
        aquiredSamplesCount = 0;
      }
      
      for( size_t rawChannel = 0; rawChannel < RAW_INPUT_CHANNELS_NUMBER; rawChannel += 2 )
      {
        double currentPair[ 2 ];
        for( size_t sampleIndex = 0; sampleIndex < 2; sampleIndex++ )
        {
          //WORD rawValue = rawInputSamplesList[ rawChannel + sampleIndex ];
          //currentPair[ sampleIndex ] = ( (double) rawValue ) / 0x8000 * IO_RANGE;
          //currentPair[ sampleIndex ] = ( (double) rawValue ) / 0x8000 * IO_RANGE;
          //if( rawValue >= 0x8000 ) currentPair[ sampleIndex ] -= ( 2.0 * IO_RANGE );
          currentPair[ sampleIndex ] = tensionsList[ rawChannel + sampleIndex ];
          //currentPair[ sampleIndex ] -= TENSION_2_CURRENT_OFFSET;
          currentPair[ sampleIndex ] /= TENSION_2_CURRENT_RATIO;
        }
        
        double alpha = atan2( currentPair[ 0 ] * cos( 4.0 / 3.0 * M_PI ) - currentPair[ 1 ], currentPair[ 0 ] * sin( 4.0 / 3.0 * M_PI ) );
        double current = ( fabs( cos( alpha ) ) < 0.1 ) ? currentPair[ 1 ] / ( cos( alpha + ( 4.0 / 3.0 * M_PI ) ) ) : currentPair[ 0 ] / cos( alpha );
        
        size_t channel = rawChannel / 2;        
        adapter->inputSamplesList[ channel ] = -current * CURRENT_2_TORQUE_RATIO;
        adapter->aquiredSamplesCountList[ channel ] = ( channel < aquiredSamplesCount / 2 ) ? 1 : 0;
        Semaphores.SetCount( adapter->inputChannelLocksList[ channel ], adapter->inputChannelUsesList[ channel ] );
      }

      _PdAInSwClStart( adapter->inputHandle );
    }
    
    Timing.Delay( 1 );
  }
  
  return NULL;
}
