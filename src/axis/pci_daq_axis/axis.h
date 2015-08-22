#ifndef AXIS_CAN_EPOS_INTERFACE_H
#define AXIS_CAN_EPOS_INTERFACE_H

#include "axis_interface.h"

#include "pci_daq_axis/pci4e/pci4e.h"
#include "pci_daq_axis/pci4e/pci4eHelper.h"

#include "pci_daq_axis/powerdaq/win_sdk_types.h"
#include "pci_daq_axis/powerdaq/powerdaq.h"
#include "pci_daq_axis/powerdaq/powerdaq32.h"

#ifdef WIN32
  #include "time/timing_windows.h"
#else
  #include "time/timing_unix.h"
#endif

#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <malloc.h>
#include <stdbool.h>

#include "klib/khash.h"
#include "file_parsing/json_parser.h"

#include "debug/async_debug.h"

#define DAQ_ACQUIRE 1
#define DAQ_RELEASE 0
#define PDAQ_ENABLE 1
#define PDAQ_DISABLE 0

#define DAQ_SAMPLES_NUMBER 16                   // treat each board the same, 16 samples.
#define DAQ_TENSION_INPUT_RANGE 20.0            // from -10.0 to +10.0
#define DAQ_TENSION_INPUT_AMPLITUDE ( DAQ_TENSION_INPUT_RANGE / 2.0 )
#define DAQ_TENSION_INPUT_OFFSET ( -DAQ_TENSION_INPUT_AMPLITUDE )
#define DAQ_TENSION_CROSSTALK_BIAS 0.018

#define PCI_ENCODER_COUNT_LIMIT ( 1 << 24 )

static const uint8_t operationModes[ AXIS_DIMENSIONS_NUMBER ] = { 0xFF, 0xFE };

typedef struct _PCIDAQInterface
{
  int daqBoardID;
  int analogInputHandle, analogOutputHandle;
  unsigned int analogInputChannels[ 2 ], analogOutputChannel;
  double analogMultiplier;
  int pciCardID;
  unsigned int encoderChannel;
  unsigned int encoderResolution;
}
PCIDAQInterface;

KHASH_MAP_INIT_INT( PCIDAQ, PCIDAQInterface );
static khash_t( PCIDAQ )* interfacesList = NULL;

static uint16_t* analogReadBuffer;
static size_t usedChannelsCount = 0;
static size_t analogInputChannelsNumber;
static DWORD analogWriteBuffer;

static int Connect( const char* );
static void Disconnect( int );
static void Enable( int );
static void Disable( int );
static void Reset( int );
static bool IsEnabled( int );
static bool HasError( int );
static bool ReadMeasures( int, double[ AXIS_DIMENSIONS_NUMBER ] );
static void WriteControl( int, double );

static inline PCIDAQInterface* LoadInterfaceData( const char* );
static inline double DAQAnalogRead( PCIDAQInterface* );
static inline void DAQAnalogWrite( PCIDAQInterface*, double ); 
static inline double PCIEncoderRead( PCIDAQInterface* );

const AxisMethods AxisPCIDAQMethods = { .Connect = Connect, .Disconnect = Disconnect, .Enable = Enable, .Disable = Disable, .Reset = Reset,
                                        .IsEnabled = IsEnabled, .HasError = HasError, .ReadMeasures = ReadMeasures, .WriteControl = WriteControl };

// Create CAN controlled DC motor handle

static int Connect( const char* configKey )
{
  short pciCardsNumber;
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "%d PCI cards available", PCI4E_Initialize( &pciCardsNumber ) );
  
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "%d DAQ boards available", PdGetNumberAdapters() );
  
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "trying to connect PCI DAQ interface %s", configKey );
  
  PCIDAQInterface* ref_interfaceData = LoadInterfaceData( configKey );
  
  if( ref_interfaceData == NULL ) return -1;
  
  if( interfacesList == NULL ) interfacesList = kh_init( PCIDAQ );
  
  int insertionStatus;
  khiter_t interfaceID = kh_put( PCIDAQ, interfacesList, pciCardID + daqBoardID + encoderChannel + analogOutputChannel, &insertionStatus );
  if( insertionStatus > 0 )
  {
    PCIDAQInterface* interface = &(kh_value( interfacesList, interfaceID ));
    
    memcpy( interface, ref_interfaceData, sizeof(PCIDAQInterface) );
    
    Adapter_Info adapterInfo;
    _PdGetAdapterInfo( interface->daqBoardID, &adapterInfo );
    analogInputChannelsNumber = adapterInfo.SSI[ AnalogIn ].dwChannels;
    
    interface->analogInputHandle = PdAcquireSubsystem( 0, AnalogIn, DAQ_ACQUIRE );
    interface->analogOutputHandle = PdAcquireSubsystem( 0, AnalogOut, DAQ_ACQUIRE );
    
    Reset( interfaceID );
    
    _PdAInEnableConv( interface->analogInputHandle, PDAQ_ENABLE );
    _PdAInSwStartTrig( interface->analogInputHandle );
    _PdAInSwClStart( interface->analogInputHandle );
    
    if( usedChannelsCount++ == 0 ) analogReadBuffer = (uint16_t*) calloc( DAQ_SAMPLES_NUMBER * analogInputChannelsNumber, sizeof(uint16_t) );
    
    Disable( interfaceID );
    
    /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "created interface for device %s", configKey );
    
    return interfaceID;
  }
  else if( insertionStatus == -1 )
    ERROR_EVENT( "failed creating interface for device %s", configKey );
  else if( insertionStatus == 0 )
    ERROR_EVENT( "interface for device %s already exists", configKey );
    
  return -1;
}

static void Disconnect( int interfaceID )
{
  if( kh_exist( interfacesList, interfaceID ) )
  {
    PCIDAQInterface* interface = &(kh_value( interfacesList, interfaceID ));
    
    _PdAInEnableConv( interface->analogInputHandle, PDAQ_DISABLE );
    _PdAInSwStopTrig( interface->analogInputHandle );
    
    PdAcquireSubsystem( interface->analogInputHandle, AnalogIn, DAQ_RELEASE );
    PdAcquireSubsystem( interface->analogOutputHandle, AnalogOut, DAQ_RELEASE );
    
    if( --usedChannelsCount == 0 ) free( analogReadBuffer );
    
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
    
    _PdClearUserEvents( 0, interface->analogInputHandle, eBufferError | eConvError | eScanError | eDataError );
    
    _PdAInReset( interface->analogInputHandle );
    _PdAOutReset( interface->analogOutputHandle );
    
    DWORD analogInputConfig = (AIN_BIPOLAR | AIN_CL_CLOCK_SW | AIN_CV_CLOCK_CONTINUOUS | AIN_SINGLE_ENDED | AIN_RANGE_10V);
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
    DWORD channelsList[ analogInputChannelsNumber ];
    for( size_t channelIndex = 0; channelIndex < analogInputChannelsNumber; channelIndex++ )
        channelsList[ channelIndex ] = CHLIST_ENT( channelIndex, 1, 1 );//channelIndex | SLOW;
    _PdAInSetChList( interface->analogInputHandle, analogInputChannelsNumber, channelsList );
    
    _PdSetUserEvents( 0, interface->analogInputHandle, eStartTrig | eBufferError | eConvError | eScanError | eDataError );
  }
}

static bool IsEnabled( int interfaceID )
{
  static tEvents boardStatus;
  
  if( kh_exist( interfacesList, interfaceID ) )
  {
    PCIDAQInterface* interface = &(kh_value( interfacesList, interfaceID ));
    
    _PdAdapterGetBoardStatus( 0, &boardStatus );
    
    //return( (boardStatus.AOutIntr & AOB_Enabled ) || (boardStatus.AOutIntr & AOB_Active ) );
  }
  
  return false;
}

static bool HasError( int interfaceID )
{
  static DWORD eventsList;
  
  if( kh_exist( interfacesList, interfaceID ) )
  {
    PCIDAQInterface* interface = &(kh_value( interfacesList, interfaceID ));
    
    _PdGetUserEvents( 0, interface->analogInputHandle, &eventsList );
    
    //if( (eventsList & eBufferError) || (eventsList & eScanError) || (eventsList & eConvError) || (eventsList & eDataError) )
    //  return true;
  }
  
  return false;
}

static bool ReadMeasures( int interfaceID, double measuresList[ AXIS_DIMENSIONS_NUMBER ] )
{
  if( kh_exist( interfacesList, interfaceID ) )
  {
    PCIDAQInterface* interface = &(kh_value( interfacesList, interfaceID ));
    
    measuresList[ AXIS_POSITION ] = PCIEncoderRead( interface ) / interface->encoderResolution;
    measuresList[ AXIS_FORCE ] = DAQAnalogRead( interface ) * interface->analogMultiplier;
    
    return true;
  }
  
  return false;
}

void WriteControl( int interfaceID, double setpointValue )
{
  // Create writing buffer
  static uint8_t payload[ 8 ];
  
  if( kh_exist( interfacesList, interfaceID ) )
  {
    PCIDAQInterface* interface = &(kh_value( interfacesList, interfaceID ));
    
    DAQAnalogWrite( interface, setpointValue / interface->analogMultiplier );
  }
}




static inline PCIDAQInterface* LoadInterface( const char* configKey )
{
  static PCIDAQInterface interfaceData;
  
  interfaceData.daqBoardID = interfaceData.pciCardID = -1;
  
  FileParser parser = JSONParser;
  int configfileID = parser.OpenFile( "pci_daq_devices" );
  if( configfileID != -1 )
  {
    if( parser.HasKey( configKey ) )
    {
      parser.SetBaseKey( configKey );
      
      if( parser.HasKey( "daq_board" ) )
      {
        interfaceData.daqBoardID = (int) parser.GetIntegerValue( "daq_board.id" );
        interfaceData.analogInputChannels[ 0 ] = (unsigned int) parser.GetIntegerValue( "daq_board.input_channels.0" );
        interfaceData.analogInputChannels[ 1 ] = (unsigned int) parser.GetIntegerValue( "daq_board.input_channels.1" );
        interfaceData.analogOutputChannel = (unsigned int) parser.GetIntegerValue( "daq_board.output_channel" );
        interfaceData.analogMultiplier = (double) parser.GetRealValue( "daq_board.signal_multiplier" );
      }
      
      if( parser.HasKey( "pci_card" ) )
      {
        interfaceData.pciCardID = (int) parser.GetIntegerValue( "pci_card.id" );
        interfaceData.encoderChannel = (unsigned int) parser.GetIntegerValue( "pci_card.channel" );
        interfaceData.encoderResolution = (unsigned int) parser.GetIntegerValue( "pci_card.resolution" );
      }
    }
    else
    {
      DEBUG_PRINT( "configuration for PCI DAQ device %s not found in file", configKey );
      return NULL;
    }
  }
  else
  {
    DEBUG_PRINT( "configuration file for PCI DAQ device %s not found", configKey );
    return NULL;
  }
  
  if( interfaceData.pciCardID < 0 || interfaceData.pciCardID >= 1/*pciCardsNumber*/ )
  {
    DEBUG_PRINT( "invalid PCI card ID for device %s: %d", configKey, interfaceData.pciCardID );
    return NULL;
  }
  
  if( interfaceData.daqBoardID > 0 && interfaceData.daqBoardID < 1/*PdGetNumberAdapters()*/ )
  {
    Adapter_Info adapterInfo;
    _PdGetAdapterInfo( interfaceData.daqBoardID, &adapterInfo );
    if( !( (adapterInfo.atType & atPD2MF) || (adapterInfo.atType & atPD2MFS) ) )
    {
      DEBUG_PRINT( "invalid board %d type (not PD2-MFx): %x", 0, adapterInfo.atType ); 
      return NULL;
    }
    
    DEBUG_PRINT( "found DAQ board model %s", adapterInfo.lpBoardName );
  }
  else
  {
    DEBUG_PRINT( "invalid DAQ board ID for device %s: %d", configKey, interfaceData.daqBoardID );
    return NULL;
  }
}

static inline double DAQAnalogRead( PCIDAQInterface* interface )
{
  static double tensionMeasuresList[ 2/*DAQ_SAMPLES_NUMBER*/ ];
  static size_t readSamplesCount;
  int readStatus = _PdAInGetSamples( interface->analogInputHandle, DAQ_SAMPLES_NUMBER, analogReadBuffer, &readSamplesCount );
  
  if( readStatus <= 0 )
  {
    DEBUG_PRINT( "error reading from board %d analog input %d: error code %d", 0, interface->analogInputHandle, readStatus );
    return 0.0;
  }
  
  DEBUG_PRINT( "%u samples read", readSamplesCount );
  
  for( size_t sampleIndex = 0; sampleIndex < 2/*readSamplesCount*/; sampleIndex++ )
  {
    /*tensionMeasuresList[ sampleIndex ] = analogSamplesBuffer[ 0 ][ sampleIndex ] * DAQ_TENSION_INPUT_RANGE / 0xFFFF;
    if( analogSamplesBuffer[ 0 ][ sampleIndex ] >= 0x8000 )
      tensionMeasuresList[ sampleIndex ] -= 2.0 * DAQ_TENSION_INPUT_RANGE;*/
    //tensionMeasuresList[ sampleIndex ] = (double) ( analogReadBuffer[ interface->analogInputChannel + sampleIndex * DAQ_SAMPLES_NUMBER ] ^ 0x8000 );
    tensionMeasuresList[ sampleIndex ] = (double) ( analogReadBuffer[ interface->analogInputChannels[ sampleIndex ] ] ^ 0x8000 );
    tensionMeasuresList[ sampleIndex ] *= DAQ_TENSION_INPUT_RANGE / 0xFFFF; // 20.0 / 65535
    tensionMeasuresList[ sampleIndex ] += DAQ_TENSION_INPUT_OFFSET;
  }
  //for( size_t sampleIndex = 1; sampleIndex <= DAQ_SAMPLES_NUMBER/*readSamplesCount*/; sampleIndex++ )
  //{
  //  tensionMeasuresList[ sampleIndex - 1 ] -= tensionMeasuresList[ sampleIndex % readSamplesCount ] * DAQ_TENSION_CROSSTALK_BIAS;
  //  if( tensionMeasuresList[ sampleIndex - 1 ]  > DAQ_TENSION_INPUT_AMPLITUDE ) tensionMeasuresList[ sampleIndex - 1 ]  = DAQ_TENSION_INPUT_AMPLITUDE;
  //  else if( tensionMeasuresList[ sampleIndex - 1 ]  < -DAQ_TENSION_INPUT_AMPLITUDE ) tensionMeasuresList[ sampleIndex - 1 ]  = -DAQ_TENSION_INPUT_AMPLITUDE;
  //}
  
  double alpha = atan2( tensionMeasuresList[ 0 ] * cos( 4 * M_PI / 3 ) - tensionMeasuresList[ 1 ], tensionMeasuresList[ 0 ] * sin( 4 * M_PI / 3 ) );
  double current = ( fabs( alpha ) < 0.1 ) ? tensionMeasuresList[ 1 ] / cos( alpha + ( 4 / 3 * M_PI ) ) : tensionMeasuresList[ 0 ] / cos( alpha ); 
  
  _PdAInSwClStart( interface->analogInputHandle );
  
  return current;
}

static inline void DAQAnalogWrite( PCIDAQInterface* interface, double tensionValue )
{
    if( tensionValue  > DAQ_TENSION_INPUT_AMPLITUDE ) tensionValue  = DAQ_TENSION_INPUT_AMPLITUDE;
    else if( tensionValue  < -DAQ_TENSION_INPUT_AMPLITUDE ) tensionValue  = -DAQ_TENSION_INPUT_AMPLITUDE;
    
    DWORD writeValue = ( (DWORD) ( ( tensionValue - DAQ_TENSION_INPUT_OFFSET ) / DAQ_TENSION_INPUT_RANGE ) ) * 0xFFF;
    
    // 2 12bits output channel values per board: 0x00BBBAAA
    analogWriteBuffer &= ( 0x00FFF000 >> 12 * interface->analogOutputChannel );
    analogWriteBuffer |= ( writeValue << 12 * interface->analogOutputChannel );

    _PdAOutPutValue( 0, analogWriteBuffer );
}

static inline double PCIEncoderRead( PCIDAQInterface* interface )
{
  long encoderCount;
  if( PCI4E_GetCount( interface->pciCardID, interface->encoderChannel, &encoderCount ) < 0 )
  {
    DEBUG_PRINT( "error reading from channel %d encoder", interface->encoderChannel );\
    return 0.0;
  }
  
  if( encoderCount > PCI_ENCODER_COUNT_LIMIT / 2 ) encoderCount -= PCI_ENCODER_COUNT_LIMIT;
  
  return (double) encoderCount;
}


#endif  /* AXIS_CAN_EPOS_INTERFACE_H */
