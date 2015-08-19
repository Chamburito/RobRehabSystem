#ifndef AXIS_CAN_EPOS_INTERFACE_H
#define AXIS_CAN_EPOS_INTERFACE_H

#include "axis_interface.h"

#include "pci4e/pci4e.h"
#include "pci4e/pci4eHelper.h"

#include "powerdaq/win_sdk_types.h"
#include "powerdaq/powerdaq.h"
#include "powerdaq/powerdaq32.h"

#ifdef WIN32
  #include "timing_windows.h"
#else
  #include "timing_unix.h"
#endif

#include <string.h>
#include <stdlib.h>
#include <malloc.h>
#include <stdbool.h>

#include "klib/khash.h"

#include "async_debug.h"

#define DAQ_ACQUIRE 1
#define DAQ_RELEASE 0
#define PDAQ_ENABLE 1

#define DAQ_SAMPLES_NUMBER 16        // treat each board the same, 16 samples.
#define DAQ_CHANNELS_NUMBER 16
#define DAQ_TENSION_INPUT_RANGE 10.0
#define DAQ_TENSION_CROSSTALK_BIAS 0.018

#define PCI_ENCODER_COUNT_LIMIT ( 1 << 24 )

static const uint8_t operationModes[ AXIS_DIMENSIONS_NUMBER ] = { 0xFF, 0xFE };

typedef struct _PCIDAQInterface
{
  int analogInputHandle, analogOutputHandle, digitalInputHandle, digitalOutputHandle;
  int analogInputChannel;
  int encoderChannel;
}
PCIDAQInterface;

KHASH_MAP_INIT_INT( PCIDAQ, PCIDAQInterface );
static khash_t( PCIDAQ )* interfacesList = NULL;

static int Connect( int );
static void Disconnect( int );
static void Enable( int );
static void Disable( int );
static void Reset( int );
static bool IsEnabled( int );
static bool HasError( int );
static bool ReadMeasures( int, double[ AXIS_DIMENSIONS_NUMBER ] );
static void WriteControl( int, double[ AXIS_DIMENSIONS_NUMBER ] );
static void SetOperationMode( int, enum AxisDimensions );

static double DAQAnalogRead( PCIDAQInterface* );
//static double DAQDigitalRead( PCIDAQInterface* );
static double PCIEncoderRead( PCIDAQInterface* );

static inline void SetControl( CANInterface*, enum Controls, bool );
static inline bool CheckState( CANInterface* interface, enum States stateValue );
static inline double ReadSingleValue( CANInterface*, uint16_t, uint8_t );
static inline void WriteSingleValue( CANInterface*, uint16_t, uint8_t, int );
static inline void EnableDigitalOutput( CANInterface*, bool );

const AxisInterface AxisPCIDAQInterface = { .Connect = Connect,
                                            .Disconnect = Disconnect,
                                            .Enable = Enable,
                                            .Disable = Disable,
                                            .Reset = Reset,
                                            .IsEnabled = IsEnabled,
                                            .HasError = HasError,
                                            .ReadMeasures = ReadMeasures,
                                            .WriteControl = WriteControl,
                                            .SetOperationMode = SetOperationMode };
                                                    
const size_t ADDRESS_MAX_LENGTH = 16;

// Create CAN controlled DC motor handle

static int Connect( int channel )
{
  DEBUG_EVENT( 0, "connecting PCIDAQ interface to channel %d", channel );
  
  short pci4eCardsNumber;
  if( PCI4E_Initialize( &pci4eCardsNumber ) == NO_CARDS_DETECTED )
  {
    //DEBUG_PRINT( "No PCI4E boards available" );
    return -1;
  }
  //else if( pci4eCardsNumber > channel )
  //{
  //  
  //}
  if( PdGetNumberAdapters() <= 0 )
  {
    //DEBUG_PRINT( "No DAQ boards available" );
    return -1;
  }
  else
  {
    Adapter_Info adapterInfo;
    _PdGetAdapterInfo( 0, &adapterInfo );
    if( !(adapterInfo.atType & atMF) )
    {
      DEBUG_PRINT( "invalid board %d type (not MFx)", 0 ); 
      return -1;
    }
    
    DEBUG_PRINT( "found DAQ board model %s", adapterInfo.lpBoardName );
  }
  
  if( interfacesList == NULL )
  {
    //CANNetwork_Start( CAN_DATABASE, CAN_CLUSTER );
    interfacesList = kh_init( PCIDAQ );
  }
  
  int insertionStatus;
  khiter_t interfaceID = kh_put( PCIDAQ, interfacesList, networkNode, &insertionStatus );
  if( insertionStatus > 0 )
  {
    PCIDAQInterface* interface = &(kh_value( interfacesList, interfaceID ));
    
    interface->encoderChannel = channel;
    
    interface->analogInputHandle = PdAcquireSubsystem( 0, AnalogIn, DAQ_ACQUIRE );
    interface->analogOutputHandle = PdAcquireSubsystem( 0, AnalogOut, DAQ_ACQUIRE );
    interface->digitalInputHandle = PdAcquireSubsystem( 0, DigitalIn, DAQ_ACQUIRE );
    interface->digitalOutputHandle = PdAcquireSubsystem( 0, DigitalOut, DAQ_ACQUIRE );
    
    _PdAInReset( interface->analogInputHandle );
    _PdAOutReset( interface->analogOutputHandle );
    _PdDInReset( interface->digitalInputHandle );
    _PdDOutReset( interface->digitalOutputHandle );

    uint32_t analogInputConfig = (AIN_BIPOLAR | AIN_CL_CLOCK_SW | AIN_CV_CLOCK_CONTINUOUS | AIN_SINGLE_ENDED | AIN_RANGE_10V);
    _PdAInSetCfg( interface->analogInputHandle, analogInputConfig, 0, 0 );

    // we can't init uei_board here because this init happens AFTER
    // imt2.cal has been read.  In the 99% case, there is just
    // one board, and daq->uei_board[0] is 0, and all is well.
    // if you have more than one board, set it up in the cal file.
    // phandle == daq->uei_board[lhandle]
    // if you want the 3rd uei board (in position 2) to stick
    // adc voltages in slots 0-15, do:
    // s ueiboard lhandle phandle, i.e.:
    // s uei_board 0 2

    // fill CL with "slow bit" flag
    // if > 1 boards, this is all the same.
    uint32_t channelsList[ DAQ_CHANNELS_NUMBER ];
    for( size_t channelIndex = 0; channelIndex < DAQ_CHANNELS_NUMBER; channelIndex++ )
        channelsList[ channelIndex ] = channelIndex | SLOW;
    _PdAInSetChList( interface->analogInputHandle, DAQ_CHANNELS_NUMBER, channelsList );
    _PdAInEnableConv( interface->analogInputHandle, PDAQ_ENABLE);
    _PdAInSwStartTrig( interface->analogInputHandle );
    _PdAInSwClStart( interface->analogInputHandle );
    
    interface->analogInputChannel = 0;
    
    Disable( interfaceID );
    
    DEBUG_EVENT( 0, "created interface on network node %d", networkNode );
    
    return interfaceID;
  }
  
  if( insertionStatus == -1 )
    ERROR_EVENT( "failed creating interface on network node %d", networkNode );
  else if( insertionStatus == 0 )
    ERROR_EVENT( "interface on network node %d already exists", networkNode );
    
  return -1;
}

static void Disconnect( int interfaceID )
{
  if( kh_exist( interfacesList, interfaceID ) )
  {
    PCIDAQInterface* interface = &(kh_value( interfacesList, interfaceID ));
    
    _PdAInReset( interface->analogInputHandle );
    _PdAOutReset( interface->analogOutputHandle );
    _PdDInReset( interface->digitalInputHandle );
    _PdDOutReset( interface->digitalOutputHandle );
            
    PdAcquireSubsystem( interface->analogInputHandle, AnalogIn, DAQ_RELEASE );
    PdAcquireSubsystem( interface->analogOutputHandle, AnalogOut, DAQ_RELEASE );
    PdAcquireSubsystem( interface->digitalInputHandle, DigitalIn, DAQ_RELEASE );
    PdAcquireSubsystem( interface->digitalOutputHandle, DigitalOut, DAQ_RELEASE );
    
    kh_del( PCIDAQ, interfacesList, interfaceID );
    
    if( kh_size( interfacesList ) == 0 )
    {
      kh_destroy( PCIDAQ, interfacesList );
      interfacesList = NULL;
    }
  }
}

static void Enable( int interfaceID )
{
  if( kh_exist( interfacesList, interfaceID ) )
  {
    PCIDAQInterface* interface = &(kh_value( interfacesList, interfaceID ));
    
    
  }
}

static void Disable( int interfaceID )
{
  if( kh_exist( interfacesList, interfaceID ) )
  {
    PCIDAQInterface* interface = &(kh_value( interfacesList, interfaceID ));
    
    
  }
}

void Reset( int interfaceID )
{
  if( kh_exist( interfacesList, interfaceID ) )
  {
    PCIDAQInterface* interface = &(kh_value( interfacesList, interfaceID ));
    
    
  }
}

static bool IsEnabled( int interfaceID )
{
  if( kh_exist( interfacesList, interfaceID ) )
  {
    PCIDAQInterface* interface = &(kh_value( interfacesList, interfaceID ));
    
    
  }
  
  return false;
}

static bool HasError( int interfaceID )
{
  if( kh_exist( interfacesList, interfaceID ) )
  {
    PCIDAQInterface* interface = &(kh_value( interfacesList, interfaceID ));
    
    
  }
  
  return false;
}

static bool ReadMeasures( int interfaceID, double measuresList[ AXIS_DIMENSIONS_NUMBER ] )
{
  // Create reading buffer

  if( kh_exist( interfacesList, interfaceID ) )
  {
    PCIDAQInterface* interface = &(kh_value( interfacesList, interfaceID ));
    
    measuresList[ AXIS_POSITION ] = PCIEncoderRead( interface );
    measuresList[ AXIS_TORQUE ] = DAQAnalogRead( interface );
    
    return true;
  }
  
  return false;
}

void WriteControl( int interfaceID, double setpointsList[ AXIS_DIMENSIONS_NUMBER ] )
{
  // Create writing buffer
  static uint8_t payload[ 8 ];
  
  if( kh_exist( interfacesList, interfaceID ) )
  {
    PCIDAQInterface* interface = &(kh_value( interfacesList, interfaceID ));
    
    
  }
}

static void SetOperationMode( int interfaceID, enum AxisOperationModes mode )
{
  if( kh_exist( interfacesList, interfaceID ) )
  {
    PCIDAQInterface* interface = &(kh_value( interfacesList, interfaceID ));
    
    
  }
}

static inline double DAQAnalogRead( PCIDAQInterface* interface )
{
  static uint16_t analogSamplesBuffer[ DAQ_CHANNELS_NUMBER ][ DAQ_SAMPLES_NUMBER ];
  static uint16_t tensionMeasuresList[ DAQ_SAMPLES_NUMBER ];
  static size_t readSamplesCount;
  int readStatus = _PdAInGetSamples( interface->analogInputHandle, DAQ_SAMPLES_NUMBER, analogSamplesBuffer, &readSamplesCount );
  
  if( readStatus <= 0 )
  {
    DEBUG_PRINT( "error reading from board %d analog input %d: error code %d", 0, interface->analogInputHandle, readStatus );
    return false;
  }
  
  DEBUG_PRINT( "%u samples read", readSamplesCount );
  
  for( size_t sampleIndex = 0; sampleIndex < readSamplesCount; sampleIndex++ )
  {
    tensionMeasuresList[ sampleIndex ] = analogSamplesBuffer[ 0 ][ sampleIndex ] * DAQ_TENSION_INPUT_RANGE / 0x8000;
    if( analogSamplesBuffer[ 0 ][ sampleIndex ] >= 0x8000 )
      tensionMeasuresList[ sampleIndex ] -= 2.0 * DAQ_TENSION_INPUT_RANGE;
  }
  for( size_t sampleIndex = 1; sampleIndex <= readSamplesCount; sampleIndex++ )
  {
    tensionMeasuresList[ sampleIndex - 1 ] -= tensionMeasuresList[ sampleIndex % readSamplesCount ] * DAQ_TENSION_CROSSTALK_BIAS;
    if( tensionMeasuresList[ sampleIndex - 1 ]  > DAQ_TENSION_INPUT_RANGE ) tensionMeasuresList[ sampleIndex - 1 ]  = DAQ_TENSION_INPUT_RANGE;
    else if( tensionMeasuresList[ sampleIndex - 1 ]  < -DAQ_TENSION_INPUT_RANGE ) tensionMeasuresList[ sampleIndex - 1 ]  = -DAQ_TENSION_INPUT_RANGE;
  }
  
  _PdAInSwClStart( interface->analogInputHandle );
  
  return 0.0;
}

static inline double PCIEncoderRead( PCIDAQInterface* interface )
{
  long encoderCount;
  if( PCI4E_GetCount( 0, interface->encoderChannel, &encoderCount ) < 0 )
  {
    DEBUG_PRINT( "error reading from channel %d encoder", interface->encoderChannel );\
    return 0.0;
  }
  
  if( encoderCount > PCI_ENCODER_COUNT_LIMIT / 2 ) encoderCount -= PCI_ENCODER_COUNT_LIMIT;
  
  return (double) encoderCount;
}


#endif  /* AXIS_CAN_EPOS_INTERFACE_H */
