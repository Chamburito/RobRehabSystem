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

#define OUTPUT_CHANNELS_NUMBER 2

static const int DAQ_ACQUIRE = 1;
static const int DAQ_RELEASE = 0;

static const double IO_RANGE = 10.0;

typedef struct _IOAdapterData
{
  int outputHandle;
  bool initialized;
}
IOAdapterData;

typedef IOAdapterData* IOAdapter;

static IOAdapter adaptersList = NULL;
static size_t adaptersNumber = 0;

DECLARE_MODULE_INTERFACE( SIGNAL_IO_INTERFACE ) 


int InitTask( const char* adapterConfig )
{
  if( PdGetNumberAdapters() <= 0 ) return SIGNAL_IO_TASK_INVALID_ID;
  
  size_t adapterIndex = strtoul( adapterConfig, NULL, 0 );
  if( adapterIndex >= (size_t) PdGetNumberAdapters() ) return SIGNAL_IO_TASK_INVALID_ID;
  
  Adapter_Info adapterInfo;
  _PdGetAdapterInfo( adapterIndex, &adapterInfo );
  DEBUG_PRINT( "PD2-AO board ?: %s", ( adapterInfo.atType & atPD2AO ) ? "true" : "false" );
  if( !( adapterInfo.atType & atPD2AO ) ) return SIGNAL_IO_TASK_INVALID_ID;
  
  if( adaptersNumber == 0 )
  {
    adaptersNumber = (size_t) PdGetNumberAdapters();
    adaptersList = (IOAdapter) calloc( adaptersNumber, sizeof(IOAdapterData) );
    memset( adaptersList, 0, adaptersNumber * sizeof(IOAdapterData) );
    DEBUG_PRINT( "%lu PowerDAQ adapters found", adaptersNumber );
  }
  
  IOAdapter newAdapter = (IOAdapter) &(adaptersList[ adapterIndex ]);
  if( !newAdapter->initialized )
  {
    newAdapter->outputHandle = PdAcquireSubsystem( adapterIndex, AnalogOut, DAQ_ACQUIRE ); 
    DEBUG_PRINT( "acquired output subsystem: %d", newAdapter->outputHandle );    
        
    Reset( adapterIndex );
    
    newAdapter->initialized = true;
  }
  
  return (int) adapterIndex;
}

void EndTask( int adapterIndex )
{
  if( (size_t) adapterIndex >= adaptersNumber ) return;
  
  IOAdapter adapter = (IOAdapter) &(adaptersList[ adapterIndex ]);
  
  Reset( adapterIndex );
    
  PdAcquireSubsystem( /*adapter->outputHandle*/2, AnalogOut, DAQ_RELEASE );
  
  adapter->initialized = false;
  
  for( adapterIndex = 0; adapterIndex < (int) adaptersNumber; adapterIndex++ )
  {
    if( adaptersList[ adapterIndex ].initialized ) return;
  }
  
  free( adaptersList );
  adaptersList = NULL;
  adaptersNumber = 0;
}

size_t GetMaxInputSamplesNumber( int adapterIndex )
{
  return 0;
}

size_t Read( int adapterIndex, unsigned int channel, double* ref_value )
{
  return 0;
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
        
  _PdAOutReset( adapter->outputHandle );
  _PdAO32Reset( adapter->outputHandle );
  _PdAOutSetCfg( adapter->outputHandle, 0, 0 );
}

bool AquireInputChannel( int adapterIndex, unsigned int channel )
{
  return false;
}

void ReleaseInputChannel( int adapterIndex, unsigned int channel )
{
  return;
}

void EnableOutput( int adapterIndex, bool enable )
{
  if( (size_t) adapterIndex >= adaptersNumber ) return;
  
  IOAdapter adapter = (IOAdapter) &(adaptersList[ adapterIndex ]);
    
  int action = enable ? DAQ_ACQUIRE : DAQ_RELEASE;
  //PdAcquireSubsystem( adapter->outputHandle, AnalogOut, DAQ_RELEASE );
}

bool IsOutputEnabled( int adapterIndex )
{
  if( (size_t) adapterIndex >= adaptersNumber ) return false;
  
  return adaptersList[ adapterIndex ].initialized;
}

bool Write( int adapterIndex, unsigned int channel, double value )
{
  const double TENSION_2_TORQUE_RATIO = 0.027;    // [Nm/V]
  
  if( (size_t) adapterIndex >= adaptersNumber ) return false;
  
  if( channel >= OUTPUT_CHANNELS_NUMBER ) return false;
  
  IOAdapter adapter = (IOAdapter) &(adaptersList[ adapterIndex ]);
  
  if( !adapter->initialized ) return false;
  
  value /= TENSION_2_TORQUE_RATIO;
  if( value < (double) -IO_RANGE ) value = -IO_RANGE;
  else if( value < (double) IO_RANGE ) value = IO_RANGE;
  
  DWORD output = ( value + IO_RANGE ) / ( 2 * IO_RANGE ) * 0xFFFF;
  DEBUG_PRINT( "writing value %g (output: %x) to handle %d and channel %u", value, output, adapter->outputHandle, channel );
  //if( _PdAO32Write( adapter->outputHandle, channel, output ) < 0 ) return false;
  
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
