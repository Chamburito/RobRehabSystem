#ifndef SIGNAL_AQUISITION_H
#define SIGNAL_AQUISITION_H

#include <NIDAQmx.h>

#include "async_debug.h"

TaskHandle signalReadTask;

static uInt32 signalChannelsNumber = 0;

static float64* aquiredSamplesList = NULL;
static float64** aquiredSamplesTable = NULL;
static size_t channelAquisitionSamplesNumber = 0;

int SignalAquisition_Init( size_t aquisitionSamplesNumber )
{
  DAQmxLoadTask( "EMGReadTask", &signalReadTask );
  
  DAQmxGetTaskAttribute( signalReadTask, DAQmx_Task_NumChans, &signalChannelsNumber );
  
  DEBUG_PRINT( "%u signal channels found", signalChannelsNumber );
  
  channelAquisitionSamplesNumber = aquisitionSamplesNumber;
  aquiredSamplesList = (float64*) calloc( signalChannelsNumber * channelAquisitionSamplesNumber, sizeof(float64) );
  aquiredSamplesTable = (double**) calloc( signalChannelsNumber, sizeof(float64*) );
  
  DAQmxStartTask( signalReadTask );
  
  return 0;
}

void SignalAquisition_End()
{
  DAQmxStopTask( signalReadTask );
  DAQmxClearTask( signalReadTask );
  
  free( aquiredSamplesList );
  free( aquiredSamplesTable );
}

const unsigned int SIGNAL_AQUISITION_INFINITE_TIMEOUT = 0xFFFFFFFF;

double** SignalAquisition_Read( unsigned int timeout, int* ref_aquiredSamplesNumber )
{
  static int32 aquiredSamplesNumber;
  
  float64 timeoutSeconds = ( timeout == SIGNAL_AQUISITION_INFINITE_TIMEOUT ) ? DAQmx_Val_WaitInfinitely : ( (float64) timeout ) / 1000.0;
  
  int errorCode = DAQmxReadAnalogF64( signalReadTask, channelAquisitionSamplesNumber, timeoutSeconds, DAQmx_Val_GroupByChannel, 
                                      aquiredSamplesList, signalChannelsNumber * channelAquisitionSamplesNumber, &aquiredSamplesNumber, NULL );
  
  if( errorCode < 0 ) 
  {
    static char errorMessage[ DEBUG_MESSAGE_LENGTH ];
    DAQmxGetErrorString( errorCode, errorMessage, DEBUG_MESSAGE_LENGTH );
    ERROR_EVENT( "error aquiring EMG data: %s", errorMessage );
    
    return NULL;
  }
  
  for( size_t channel = 0; channel < signalChannelsNumber; channel++ )
    aquiredSamplesTable[ channel ] = aquiredSamplesList + channel * aquiredSamplesNumber;
  
  if( ref_aquiredSamplesNumber != NULL ) *ref_aquiredSamplesNumber = (int) aquiredSamplesNumber;
  
  return (double**) aquiredSamplesTable;
}

extern inline size_t SignalAquisition_GetChannelsNumber()
{
  return (size_t) signalChannelsNumber;
}

#endif // SIGNAL_AQUISITION_H
