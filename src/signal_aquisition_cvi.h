#ifndef SIGNAL_AQUISITION_H
#define SIGNAL_AQUISITION_H

#include <NIDAQmx.h>

#include "async_debug.h"

TaskHandle signalReadTask;

Thread_Handle aquisitionThreadID;
ThreadLock aquisitionLock;
bool isReading = false;

static uInt32 signalChannelsNumber = 0;
static size_t channelAquisitionSamplesNumber = 0;
static float64** aquiredSamplesTable = NULL;

static void* ReadBuffer( void* );

int SignalAquisition_Init( const char* configFileName, size_t aquisitionSamplesNumber )
{
  DAQmxLoadTask( configFileName, &signalReadTask );
  
  DAQmxGetTaskAttribute( signalReadTask, DAQmx_Task_NumChans, &signalChannelsNumber );
  
  DEBUG_PRINT( "%u signal channels found", signalChannelsNumber );
  
  channelAquisitionSamplesNumber = aquisitionSamplesNumber;
  aquiredSamplesTable = (float64**) calloc( signalChannelsNumber, sizeof(float64*) );
  
  aquisitionLock = ThreadLock_Create();
  
  DAQmxStartTask( signalReadTask );
  
  isReading = true;
  aquisitionThreadID = Thread_Start( ReadBuffer, NULL, JOINABLE );
  
  return 0;
}

void SignalAquisition_End()
{
  isReading = false;
  Thread_WaitExit( aquisitionThreadID, 5000 );
  
  ThreadLock_Discard( aquisitionLock );
  
  DAQmxStopTask( signalReadTask );
  DAQmxClearTask( signalReadTask );
  
  free( aquiredSamplesTable );
}

static void* ReadBuffer( void* callbackData )
{
  float64 aquiredSamplesList[ signalChannelsNumber * channelAquisitionSamplesNumber ];
  
  int32 aquiredSamplesNumber;
  
  while( isReading )
  {
    int errorCode = DAQmxReadAnalogF64( signalReadTask, channelAquisitionSamplesNumber, DAQmx_Val_WaitInfinitely, DAQmx_Val_GroupByChannel, 
                                        aquiredSamplesList, signalChannelsNumber * channelAquisitionSamplesNumber, &aquiredSamplesNumber, NULL );
    
    if( errorCode < 0 )
    {
      static char errorMessage[ DEBUG_MESSAGE_LENGTH ];
      DAQmxGetErrorString( errorCode, errorMessage, DEBUG_MESSAGE_LENGTH );
      ERROR_EVENT( "error aquiring EMG data: %s", errorMessage );
    
      break;
    }
    
    ThreadLock_Aquire( aquisitionLock );
    
    for( size_t channel = 0; channel < signalChannelsNumber; channel++ )
      aquiredSamplesTable[ channel ] = aquiredSamplesList + channel * aquiredSamplesNumber;
    
    ThreadLock_Release( aquisitionLock );
  }
  
  Thread_Exit( 0 );
  return NULL;
}

double* SignalAquisition_Read( unsigned int channel )
{
  if( isReading )
  {
    if( channel >= signalChannelsNumber ) return NULL;
    
    ThreadLock_Aquire( aquisitionLock );
    
    double* aquiredSamplesList = (double*) aquiredSamplesTable[ channel ];
    aquiredSamplesTable[ channel ] = NULL;
    
    ThreadLock_Release( aquisitionLock );
  
    return aquiredSamplesList;
  }
  
  return NULL;
}

extern inline size_t SignalAquisition_GetChannelsNumber()
{
  return (size_t) signalChannelsNumber;
}

#endif // SIGNAL_AQUISITION_H
