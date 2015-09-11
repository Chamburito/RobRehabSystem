 /* 
 * File:   AXIS.h
 * Author: GUILHERME FERNANDES
 *
 * ESTA FUNÇÃO REALIZA O CONTROLE DE UM EIXO DA EPOS
 * 
 * Created on 26 de Janeiro de 2012, 19:34
 */

#ifndef AXIS_CAN_EPOS_INTERFACE_H
#define AXIS_CAN_EPOS_INTERFACE_H

#include "axis_interface.h"
#include "ni_can_epos_axis/can_network.h"

#ifdef WIN32
  #include "time/timing_windows.h"
#else
  #include "time/timing_unix.h"
#endif

#ifdef _CVI_
  #include <ansi_c.h>
#else
  #include <stdio.h>
  #include <string.h>
  #include <stdlib.h>
  #include <malloc.h>
#endif

#include "klib/khash.h"
#include "file_parsing/json_parser.h"

#include <stdbool.h>

#include "debug/async_debug.h"

/*enum OperationMode { HOMMING_MODE, PROFILE_VELOCITY_MODE, PROFILE_POSITION_MODE, POSITION_MODE, 
                     VELOCITY_MODE, CURRENT_MODE, MASTER_ENCODER_MODE, STEP_MODE, AXIS_MODES_NUMBER };
static const uint8_t operationModes[ AXIS_MODES_NUMBER ] = { 0x06, 0x03, 0x01, 0xFF, 0xFE, 0xFD, 0xFB, 0xFA };*/

static const uint8_t operationModes[ AXIS_DIMENSIONS_NUMBER ] = { 0xFF, 0xFE, 0x00 };

enum States { READY_2_SWITCH_ON = 1, SWITCHED_ON = 2, OPERATION_ENABLED = 4, FAULT = 8, VOLTAGE_ENABLED = 16, 
              QUICK_STOPPED = 32, SWITCH_ON_DISABLE = 64, REMOTE_NMT = 512, TARGET_REACHED = 1024, SETPOINT_ACK = 4096 };

enum Controls { SWITCH_ON = 1, ENABLE_VOLTAGE = 2, QUICK_STOP = 4, ENABLE_OPERATION = 8, 
                NEW_SETPOINT = 16, CHANGE_IMMEDIATEDLY = 32, ABS_REL = 64, FAULT_RESET = 128, HALT = 256 };

enum FrameTypes { SDO, PDO01, PDO02, AXIS_FRAME_TYPES_NUMBER };

//CAN database addressing
const char* CAN_DATABASE = "database";
const char* CAN_CLUSTER = "NETCAN";

typedef struct _CANInterface
{
  CANFrame* readFramesList[ AXIS_FRAME_TYPES_NUMBER ];
  CANFrame* writeFramesList[ AXIS_FRAME_TYPES_NUMBER ];
  size_t setpointIndex;
  unsigned int encoderResolution;
  double currentToForceRatio;
  uint16_t statusWord, controlWord;
  uint16_t digitalOutput;
  double currentSetpoint;
}
CANInterface;

KHASH_MAP_INIT_INT( CAN, CANInterface* )
static khash_t( CAN )* interfacesList = NULL;

static int CANEPOS_Connect( const char* );
static void CANEPOS_Disconnect( int );
static void CANEPOS_Enable( int );
static void CANEPOS_Disable( int );
static void CANEPOS_Reset( int );
static bool CANEPOS_IsEnabled( int );
static bool CANEPOS_HasError( int );
static bool CANEPOS_ReadMeasures( int, double[ AXIS_DIMENSIONS_NUMBER ] );
static void CANEPOS_WriteControl( int, double );

static inline CANInterface* LoadInterfaceData( const char* );
static inline void UnloadInterfaceData( CANInterface* );

static inline void SetOperationMode( CANInterface*, enum AxisDimensions );
static inline void SetControl( CANInterface*, enum Controls, bool );
static inline bool CheckState( CANInterface*, enum States );
static inline double ReadSingleValue( CANInterface*, uint16_t, uint8_t );
static inline void WriteSingleValue( CANInterface*, uint16_t, uint8_t, int );
static inline void EnableDigitalOutput( CANInterface*, bool );

const AxisOperations AxisCANEPOSOperations = { .Connect = CANEPOS_Connect, .Disconnect = CANEPOS_Disconnect, .Enable = CANEPOS_Enable, .Disable = CANEPOS_Disable, .Reset = CANEPOS_Reset,
                                               .IsEnabled = CANEPOS_IsEnabled, .HasError = CANEPOS_HasError, .ReadMeasures = CANEPOS_ReadMeasures, .WriteControl = CANEPOS_WriteControl };
                                                    
const size_t ADDRESS_MAX_LENGTH = 16;

// Create CAN controlled DC motor handle
static int CANEPOS_Connect( const char* configFileName )
{
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "trying to connect CAN interface %s", configFileName );
  
  if( interfacesList == NULL )
  {
    CANNetwork_Start( CAN_DATABASE, CAN_CLUSTER );
    interfacesList = kh_init( CAN );
  }
  
  int configKey = (int) kh_str_hash_func( configFileName ); 
  
  int insertionStatus;
  khiter_t newInterfaceID = kh_put( CAN, interfacesList, configKey, &insertionStatus );
  if( insertionStatus > 0 )
  {
    kh_value( interfacesList, newInterfaceID ) = LoadInterfaceData( configFileName );
    if( kh_value( interfacesList, newInterfaceID ) == NULL )
    {
      CANEPOS_Disconnect( (int) newInterfaceID );
      return -1;
    }

    CANInterface* newInterface = kh_value( interfacesList, newInterfaceID );
    
    EnableDigitalOutput( newInterface, true );
    
    SetControl( newInterface, ENABLE_VOLTAGE, true );
    SetControl( newInterface, QUICK_STOP, true );
    
    CANEPOS_Disable( newInterfaceID );
    
    /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "created CAN EPOS interface %s", configFileName );
  }
  if( insertionStatus == 0 )
    ERROR_EVENT( "CAN EPOS interface %s already exists", configFileName );
    
  return (int) newInterfaceID;
}

static void CANEPOS_Disconnect( int interfaceID )
{
  if( kh_exist( interfacesList, interfaceID ) )
  {
    CANInterface* interface = kh_value( interfacesList, interfaceID );
    
    UnloadInterfaceData( interface );
    
    kh_del( CAN, interfacesList, interfaceID );
    
    if( kh_size( interfacesList ) == 0 )
    {
      kh_destroy( CAN, interfacesList );
      interfacesList = NULL;
    }
  }
}

static void CANEPOS_Enable( int interfaceID )
{
  if( kh_exist( interfacesList, interfaceID ) )
  {
    CANInterface* interface = kh_value( interfacesList, interfaceID );
    
    SetControl( interface, SWITCH_ON, false );
    SetControl( interface, ENABLE_OPERATION, false );
    CANEPOS_WriteControl( interfaceID, interface->currentSetpoint );
          
    Timing_Delay( 200 );
          
    SetControl( interface, SWITCH_ON, true );
    CANEPOS_WriteControl( interfaceID, interface->currentSetpoint );

    Timing_Delay( 200 );
          
    SetControl( interface, ENABLE_OPERATION, true );
    CANEPOS_WriteControl( interfaceID, interface->currentSetpoint );
  }
}

static void CANEPOS_Disable( int interfaceID )
{
  if( kh_exist( interfacesList, interfaceID ) )
  {
    CANInterface* interface = kh_value( interfacesList, interfaceID );
    
    SetControl( interface, SWITCH_ON, true );
    SetControl( interface, ENABLE_OPERATION, false );
    CANEPOS_WriteControl( interfaceID, interface->currentSetpoint );
          
    Timing_Delay( 200 );
          
    SetControl( interface, SWITCH_ON, false );
    CANEPOS_WriteControl( interfaceID, interface->currentSetpoint );
  }
}

void CANEPOS_Reset( int interfaceID )
{
  if( kh_exist( interfacesList, interfaceID ) )
  {
    CANInterface* interface = kh_value( interfacesList, interfaceID );
    
    SetControl( interface, FAULT_RESET, true );
    CANEPOS_WriteControl( interfaceID, interface->currentSetpoint );
    
    Timing_Delay( 200 );
    
    SetControl( interface, FAULT_RESET, false );
    CANEPOS_WriteControl( interfaceID, interface->currentSetpoint );
  }
}

static bool CANEPOS_IsEnabled( int interfaceID )
{
  if( kh_exist( interfacesList, interfaceID ) )
  {
    CANInterface* interface = kh_value( interfacesList, interfaceID );
    
    return ( CheckState( interface, SWITCHED_ON ) && CheckState( interface, OPERATION_ENABLED ) );
  }
  
  return false;
}

static bool CANEPOS_HasError( int interfaceID )
{
  if( kh_exist( interfacesList, interfaceID ) )
  {
    CANInterface* interface = kh_value( interfacesList, interfaceID );
    
    return ( CheckState( interface, FAULT ) );
  }
  
  return false;
}

static bool CANEPOS_ReadMeasures( int interfaceID, double measuresList[ AXIS_DIMENSIONS_NUMBER ] )
{
  // Create reading buffer
  static uint8_t payload[ 8 ];

  if( kh_exist( interfacesList, interfaceID ) )
  {
    CANInterface* interface = kh_value( interfacesList, interfaceID );
    
    CANNetwork_Sync();
    
    // Read values from PDO01 (Position, Current and Status Word) to buffer
    CANFrame_Read( interface->readFramesList[ PDO01 ], payload );  
    // Update values from PDO01
    double rawPosition = payload[ 3 ] * 0x1000000 + payload[ 2 ] * 0x10000 + payload[ 1 ] * 0x100 + payload[ 0 ];
    measuresList[ AXIS_POSITION ] = rawPosition / interface->encoderResolution;
    int rawCurrent = payload[ 5 ] * 0x100 + payload[ 4 ];
    if( rawCurrent >= 0x8000 ) rawCurrent = - ( 0xFFFF - rawCurrent );
    double current = rawCurrent / 1000.0;
    measuresList[ AXIS_FORCE ] = rawCurrent * interface->currentToForceRatio;
    
    //DEBUG_PRINT( "current: %.3f - force: %.3f", current, 151.8 * current );
    
    interface->statusWord = payload[ 7 ] * 0x100 + payload[ 6 ];

    // Read values from PDO02 (Velocity and Tension) to buffer
    CANFrame_Read( interface->readFramesList[ PDO02 ], payload );  
    // Update values from PDO02
    double rawVelocity = payload[ 3 ] * 0x1000000 + payload[ 2 ] * 0x10000 + payload[ 1 ] * 0x100 + payload[ 0 ];
    measuresList[ AXIS_VELOCITY ] = rawVelocity;
    
    double tension = payload[ 5 ] * 0x100 + payload[ 4 ];
    measuresList[ AXIS_FORCE ] = tension;
    
    ///DEBUG_PRINT( "p: %+.3f - v: %+.3f - t: %+.3f", rawPosition, rawVelocity, tension );
  }
  
  return false;
}

void CANEPOS_WriteControl( int interfaceID, double setpoint )
{
  // Create writing buffer
  static uint8_t payload[ 8 ];
  
  if( kh_exist( interfacesList, interfaceID ) )
  {
    CANInterface* interface = kh_value( interfacesList, interfaceID );
  
    interface->currentSetpoint = setpoint;
    
    int rawPositionSetpoint = (int) setpoint;
    
    // Set values for PDO01 (Position Setpoint and Control Word)
    payload[ 0 ] = (uint8_t) ( rawPositionSetpoint & 0x000000ff );
    payload[ 1 ] = (uint8_t) ( ( rawPositionSetpoint & 0x0000ff00 ) / 0x100 );
    payload[ 2 ] = (uint8_t) ( ( rawPositionSetpoint & 0x00ff0000 ) / 0x10000 );
    payload[ 3 ] = (uint8_t) ( ( rawPositionSetpoint & 0xff000000 ) / 0x1000000 );
    payload[ 4 ] = ( 0 & 0x000000ff );
    payload[ 5 ] = ( 0 & 0x0000ff00 ) / 0x100; 
    payload[ 6 ] = (uint8_t) ( interface->controlWord & 0x000000ff );
    payload[ 7 ] = (uint8_t) ( ( interface->controlWord & 0x0000ff00 ) / 0x100 ); 

    // Write values from buffer to PDO01 
    CANFrame_Write( interface->writeFramesList[ PDO01 ], payload );

    int rawVelocitySetpoint = (int) setpoint;
    
    // Set values for PDO02 (Velocity Setpoint and Digital Output)
    payload[ 0 ] = (uint8_t) ( rawVelocitySetpoint & 0x000000ff );
    payload[ 1 ] = (uint8_t) ( ( rawVelocitySetpoint & 0x0000ff00 ) / 0x100 );
    payload[ 2 ] = (uint8_t) ( ( rawVelocitySetpoint & 0x00ff0000 ) / 0x10000 );
    payload[ 3 ] = (uint8_t) ( ( rawVelocitySetpoint & 0xff000000 ) / 0x1000000 );
    payload[ 4 ] = (uint8_t) ( interface->digitalOutput & 0x000000ff );
    payload[ 5 ] = (uint8_t) ( ( interface->digitalOutput & 0x0000ff00 ) / 0x100 ); 
    payload[ 6 ] = ( 0 & 0x000000ff );
    payload[ 7 ] = ( 0 & 0x0000ff00 ) / 0x100; 

    // Write values from buffer to PDO01
    CANFrame_Write( interface->writeFramesList[ PDO02 ], payload );
    
    CANNetwork_Sync();
  }
}



static const char* CAN_FRAME_NAMES[ AXIS_FRAME_TYPES_NUMBER ] = { "SDO", "PDO01", "PDO02" };
//static const char* configDirName = "axes/";
static inline CANInterface* LoadInterfaceData( const char* configFileName )
{
  static char networkAddress[ ADDRESS_MAX_LENGTH ];
  
  //bool loadError = false;
  
  CANInterface* newInterface = (CANInterface*) malloc( sizeof(CANInterface) );
  memset( newInterface, 0, sizeof(CANInterface) );
  
  /*char* configFilePath = (char*) calloc( strlen( configDirName ) + strlen( configFileName ) + 1, sizeof(char) );
  sprintf( configFilePath, "%s%s", configDirName, configFileName );
  
  FileParser parser = JSONParser;
  int configfileID = parser.LoadFile( configFilePath );
  
  free( configFilePath );
  
  if( configfileID != -1 )
  {
    unsigned int nodeID = (unsigned int) parser.GetIntegerValue( configfileID, "node_id" );
    if( nodeID != 0 )
    {
      for( size_t frameID = 0; frameID < AXIS_FRAME_TYPES_NUMBER; frameID++ )
      {
        sprintf( networkAddress, "%s_RX_%02u", CAN_FRAME_NAMES[ frameID ], nodeID );
        newInterface->readFramesList[ frameID ] = CANNetwork_InitFrame( FRAME_IN, "CAN1", networkAddress );
        
        sprintf( networkAddress, "%s_TX_%02u", CAN_FRAME_NAMES[ frameID ], nodeID );
        newInterface->writeFramesList[ frameID ] = CANNetwork_InitFrame( FRAME_OUT, "CAN2", networkAddress );
        
        if( newInterface->readFramesList[ frameID ] == NULL || newInterface->writeFramesList[ frameID ] == NULL ) 
        {
          loadError = true;
          //ERROR_EVENT( "failed creating frame %s for interface on network node %d", networkAddress, nodeID );
        }     
      }
    }
    else loadError = true; 
    
    if( (newInterface->encoderResolution = (unsigned int) parser.GetIntegerValue( configfileID, "encoder_resolution" )) == 0 ) loadError = true;
    if( (newInterface->currentToForceRatio = parser.GetRealValue( configfileID, "force_constant" )) == 0.0 ) loadError = true;
    
    parser.UnloadFile( configfileID );
  }
  else
  {
    DEBUG_PRINT( "configuration file for CAN EPOS device %s not found", configFileName );
    loadError = true;
  }
  
  if( loadError )
  {
    UnloadInterfaceData( newInterface );
    return NULL;
  }*/
  
  DEBUG_PRINT( "Trying to load CAN interface %s", configFileName );
  
  unsigned int nodeID = 1;
  for( size_t frameID = 0; frameID < AXIS_FRAME_TYPES_NUMBER; frameID++ )
  {
    sprintf( networkAddress, "%s_RX_%02u", CAN_FRAME_NAMES[ frameID ], nodeID );
    newInterface->readFramesList[ frameID ] = CANNetwork_InitFrame( FRAME_IN, "CAN1", networkAddress );

    if( newInterface->readFramesList[ frameID ] == NULL )
      DEBUG_PRINT( "error creating frame %s for CAN interface %s", networkAddress, configFileName );
    
    sprintf( networkAddress, "%s_TX_%02u", CAN_FRAME_NAMES[ frameID ], nodeID );
    newInterface->writeFramesList[ frameID ] = CANNetwork_InitFrame( FRAME_OUT, "CAN2", networkAddress );
    
    if( newInterface->writeFramesList[ frameID ] == NULL )
      DEBUG_PRINT( "error creating frame %s for CAN interface %s", networkAddress, configFileName );
  }
  
  newInterface->encoderResolution = 1;
  newInterface->currentToForceRatio = 1.0;
  
  SetOperationMode( newInterface, AXIS_POSITION );
  
  DEBUG_PRINT( "loaded CAN interface %s", configFileName );
  
  return newInterface;
}

static inline void UnloadInterfaceData( CANInterface* interface )
{
  if( interface == NULL ) return;
    
  for( size_t frameID = 0; frameID < AXIS_FRAME_TYPES_NUMBER; frameID++ )
  {
    CANFrame_End( interface->readFramesList[ frameID ] );
    CANFrame_End( interface->writeFramesList[ frameID ] );
  }
  
  free( interface );
}

static inline void SetOperationMode( CANInterface* interface, enum AxisDimensions mode )
{
  if( interface != NULL )
  {
    WriteSingleValue( interface, 0x6060, 0x00, operationModes[ mode ] );
  }
}

static inline void SetControl( CANInterface* interface, enum Controls controlValue, bool enabled )
{
  if( enabled ) interface->controlWord |= controlValue;
  else interface->controlWord &= (~controlValue);
}

static inline bool CheckState( CANInterface* interface, enum States stateValue )
{
  if( (interface->statusWord & stateValue) != 0 ) return true;
  else return false;
}

static inline double ReadSingleValue( CANInterface* interface, uint16_t index, uint8_t subIndex )
{
  // Build read requisition buffer for defined value
  static uint8_t payload[8];
  
  payload[0] = 0x40; 
  payload[1] = (uint8_t) ( index & 0x000000ff );
  payload[2] = (uint8_t) ( ( index & 0x0000ff00 ) / 0x100 );//0xff); ?
  payload[3] = subIndex;
  payload[4] = 0x0;
  payload[5] = 0x0;
  payload[6] = 0x0;
  payload[7] = 0x0;

  // Write value requisition to SDO frame 
  CANFrame_Write( interface->writeFramesList[ SDO ], payload );

  Timing_Delay( 100 );

  // Read requested value from SDO frame
  CANFrame_Read( interface->readFramesList[ SDO ], payload );

  double value = payload[ 7 ] * 0x1000000 + payload[ 6 ] * 0x10000 + payload[ 5 ] * 0x100 + payload[4];

  return value; 
}

static inline void WriteSingleValue( CANInterface* interface, uint16_t index, uint8_t subIndex, int value )
{
  // Build write buffer
  static uint8_t payload[8];
  
  payload[0] = 0x22; 
  payload[1] = (uint8_t) ( index & 0x000000ff );
  payload[2] = (uint8_t) ( ( index & 0x0000ff00 ) / 0x100 );
  payload[3] = subIndex;
  payload[4] = (uint8_t) ( value & 0x000000ff );
  payload[5] = (uint8_t) ( ( value & 0x0000ff00 ) / 0x100 );
  payload[6] = (uint8_t) ( ( value & 0x00ff0000 ) / 0x10000 );
  payload[7] = (uint8_t) ( ( value & 0xff000000 ) / 0x1000000 );

  // Write value to SDO frame 
  CANFrame_Write( interface->writeFramesList[ SDO ], payload );
}

static inline void EnableDigitalOutput( CANInterface* interface, bool enabled )
{
  if( enabled ) WriteSingleValue( interface, 0x6060, 0x00, 0xff );
  else WriteSingleValue( interface, 0x6060, 0x00, 0x00 );
}

static inline void MotorDrive_SetDigitalOutput( CANInterface* interface, uint16_t output )
{
  interface->digitalOutput = output;
}


#endif  /* AXIS_CAN_EPOS_INTERFACE_H */
