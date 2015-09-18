 /* 
 * File:   AXIS.h
 * Author: GUILHERME FERNANDES
 *
 * ESTA FUN√á√ÉO REALIZA O CONTROLE DE UM EIXO DA EPOS
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

static const int operationModes[ AXIS_SETPOINTS_NUMBER ] = { 0xFF, 0xFE, 0xFD };

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
  uint16_t statusWord, controlWord;
  uint16_t digitalOutput;
  double currentSetpoint;
}
CANInterface;

KHASH_MAP_INIT_INT( CANInt, CANInterface* )
static khash_t( CANInt )* interfacesList = NULL;

static int CANEPOS_Connect( const char* );
static void CANEPOS_Disconnect( int );
static void CANEPOS_Enable( int );
static void CANEPOS_Disable( int );
static void CANEPOS_Reset( int );
static bool CANEPOS_IsEnabled( int );
static bool CANEPOS_HasError( int );
static bool CANEPOS_ReadMeasures( int, double[ AXIS_MEASURES_NUMBER ] );
static void CANEPOS_WriteSetpoints( int, double[ AXIS_SETPOINTS_NUMBER ] );
static void CANEPOS_SetOperationMode( CANInterface*, enum AxisSetpoints );

static inline CANInterface* LoadInterfaceData( const char* );
static inline void UnloadInterfaceData( CANInterface* );

static inline void SetOperationMode( CANInterface*, enum AxisDimensions );
static inline void SetControl( CANInterface*, enum Controls, bool );
static inline bool CheckState( CANInterface*, enum States );
static inline int ReadSingleValue( CANInterface*, uint16_t, uint8_t );
static inline void WriteSingleValue( CANInterface*, uint16_t, uint8_t, int );
static inline void EnableDigitalOutput( CANInterface*, bool );

const AxisOperations AxisCANEPOSOperations = { .Connect = CANEPOS_Connect, .Disconnect = CANEPOS_Disconnect, .Enable = CANEPOS_Enable, .Disable = CANEPOS_Disable, .Reset = CANEPOS_Reset,
                                               .IsEnabled = CANEPOS_IsEnabled, .HasError = CANEPOS_HasError, .ReadMeasures = CANEPOS_ReadMeasures, .WriteSetpoints = CANEPOS_WriteSetpoints };
                                                    
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
  
  DEBUG_PRINT( "CAN EPOS list %p created", interfacesList );
  
  int configKey = (int) kh_str_hash_func( configFileName );
  
  DEBUG_PRINT( "generated CAN EPOS config key: %d", configKey );
  
  int insertionStatus;
  khiter_t newInterfaceID = kh_put( CANInt, interfacesList, configKey, &insertionStatus );
  if( insertionStatus > 0 )
  {
    DEBUG_PRINT( "inserting CAN EPOS config key on list: %d", configKey );
    
    kh_value( interfacesList, newInterfaceID ) = LoadInterfaceData( configFileName );
    if( kh_value( interfacesList, newInterfaceID ) == NULL )
    {
      CANEPOS_Disconnect( (int) newInterfaceID );
      return -1;
    }

    CANInterface* newInterface = kh_value( interfacesList, newInterfaceID );
    
    //EnableDigitalOutput( newInterface, true ); // Valor est· errado !! (0x6060 È operation mode)
    
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
  if( kh_exist( interfacesList, (khint_t) interfaceID ) )
  {
    DEBUG_PRINT( "disconnecting CAN EPOS device %d", interfaceID );
    
    CANInterface* interface = kh_value( interfacesList, (khint_t) interfaceID );
    
    UnloadInterfaceData( interface );
    
    kh_del( CANInt, interfacesList, interfaceID );
    
    if( kh_size( interfacesList ) == 0 )
    {
      kh_destroy( CANInt, interfacesList );
      interfacesList = NULL;
    }
  }
}

static void CANEPOS_Enable( int interfaceID )
{
  if( kh_exist( interfacesList, (khint_t) interfaceID ) )
  {
    CANInterface* interface = kh_value( interfacesList, (khint_t) interfaceID );
    
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
  DEBUG_EVENT( 0, "disabling CAN EPOS device %d", interfaceID );
  
  if( kh_exist( interfacesList, (khint_t) interfaceID ) )
  {
    CANInterface* interface = kh_value( interfacesList, (khint_t) interfaceID );
    
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
  if( kh_exist( interfacesList, (khint_t) interfaceID ) )
  {
    CANInterface* interface = kh_value( interfacesList, (khint_t) interfaceID );
    
    SetControl( interface, FAULT_RESET, true );
    CANEPOS_WriteControl( interfaceID, interface->currentSetpoint );
    
    Timing_Delay( 200 );
    
    SetControl( interface, FAULT_RESET, false );
    CANEPOS_WriteControl( interfaceID, interface->currentSetpoint );
  }
}

static bool CANEPOS_IsEnabled( int interfaceID )
{
  if( kh_exist( interfacesList, (khint_t) interfaceID ) )
  {
    CANInterface* interface = kh_value( interfacesList, (khint_t) interfaceID );
    
    return ( CheckState( interface, SWITCHED_ON ) && CheckState( interface, OPERATION_ENABLED ) );
  }
  
  return false;
}

static bool CANEPOS_HasError( int interfaceID )
{
  if( kh_exist( interfacesList, (khint_t) interfaceID ) )
  {
    CANInterface* interface = kh_value( interfacesList, (khint_t) interfaceID );
    
    return ( CheckState( interface, FAULT ) );
  }
  
  return false;
}

static bool CANEPOS_ReadMeasures( int interfaceID, double measuresList[ AXIS_MEASURES_NUMBER ] )
{
  // Create reading buffer
  static uint8_t payload[ 8 ];

  if( kh_exist( interfacesList, (khint_t) interfaceID ) )
  {
    CANInterface* interface = kh_value( interfacesList, (khint_t) interfaceID );
    
    CANNetwork_Sync();
    
    // Read values from PDO01 (Position, Current and Status Word) to buffer
    CANFrame_Read( interface->readFramesList[ PDO01 ], payload );  
    // Update values from PDO01
    measuresList[ AXIS_ENCODER ] = payload[ 3 ] * 0x1000000 + payload[ 2 ] * 0x10000 + payload[ 1 ] * 0x100 + payload[ 0 ];
    int rawCurrent = payload[ 5 ] * 0x100 + payload[ 4 ];
    if( rawCurrent >= 0x8000 ) rawCurrent = - ( 0xFFFF - rawCurrent );
    measuresList[ AXIS_CURRENT ] = rawCurrent / 1000.0;
    
    interface->statusWord = payload[ 7 ] * 0x100 + payload[ 6 ];

    // Read values from PDO02 (Velocity and Tension) to buffer
    CANFrame_Read( interface->readFramesList[ PDO02 ], payload );  
    // Update values from PDO02
    //double rawVelocity = payload[ 3 ] * 0x1000000 + payload[ 2 ] * 0x10000 + payload[ 1 ] * 0x100 + payload[ 0 ];
    //measuresList[ AXIS_VELOCITY ] = rawVelocity * 60.0;
    
    double analog = payload[ 5 ] * 0x100 + payload[ 4 ];
    measuresList[ AXIS_TENSION ] = analog;
  }
  
  return false;
}

void CANEPOS_WriteSetpoint( int interfaceID, double setpoint )
{
  // Create writing buffer
  static uint8_t payload[ 8 ];
  
  if( kh_exist( interfacesList, (khint_t) interfaceID ) )
  {
    CANInterface* interface = kh_value( interfacesList, (khint_t) interfaceID );
  
    interface->currentSetpoint = setpoint;
    
    int rawPositionSetpoint = (int) setpoint;
    int16_t rawCurrentSetpoint = (int16_t) setpoint + ( ( setpoint < 0.0 ) ? 0xFFFF : 0 );
    
    // Set values for PDO01 (Position Setpoint and Control Word)
    payload[ 0 ] = (uint8_t) ( rawPositionSetpoint & 0x000000ff );
    payload[ 1 ] = (uint8_t) ( ( rawPositionSetpoint & 0x0000ff00 ) / 0x100 );
    payload[ 2 ] = (uint8_t) ( ( rawPositionSetpoint & 0x00ff0000 ) / 0x10000 );
    payload[ 3 ] = (uint8_t) ( ( rawPositionSetpoint & 0xff000000 ) / 0x1000000 );
    payload[ 4 ] = (uint8_t) ( rawCurrentSetpoint & 0x000000ff );
    payload[ 5 ] = (uint8_t) ( ( rawCurrentSetpoint & 0x0000ff00 ) / 0x100 ); 
    payload[ 6 ] = (uint8_t) ( interface->controlWord & 0x000000ff );
    payload[ 7 ] = (uint8_t) ( ( interface->controlWord & 0x0000ff00 ) / 0x100 ); 

    // Write values from buffer to PDO01 
    CANFrame_Write( interface->writeFramesList[ PDO01 ], payload );

    int rawVelocitySetpoint = (int) ( setpoint / 60.0 );
    
    //DEBUG_PRINT( "velocity setpoint: %d", rawVelocitySetpoint );
    
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

static inline void CANEPOS_SetOperationMode( int interfaceID, enum AxisSetpoints mode )
{
  if( !kh_exist( interfacesList, (khint_t) interfaceID ) ) return;
  
  WriteSingleValue( kh_value( interfacesList, (khint_t) interfaceID ), 0x6060, 0x00, operationModes[ mode ] );
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
  
  SetOperationMode( newInterface, AXIS_CURRENT );
  
  DEBUG_PRINT( "operation mode: %d", ReadSingleValue( interface, 0x6060, 0x00 ) );
  
  DEBUG_PRINT( "loaded CAN interface %s", configFileName );
  
  return newInterface;
}

static inline void UnloadInterfaceData( CANInterface* interface )
{
  if( interface == NULL ) return;
    
  /*for( size_t frameID = 0; frameID < AXIS_FRAME_TYPES_NUMBER; frameID++ )
  {
    CANFrame_End( interface->readFramesList[ frameID ] );
    CANFrame_End( interface->writeFramesList[ frameID ] );
  }*/
  
  free( interface );
  
  DEBUG_PRINT( "interface %p unloaded", interface );
}

static inline void SetControl( CANInterface* interface, enum Controls controlValue, bool enabled )
{
  if( interface == NULL ) return;
  
  if( enabled ) interface->controlWord |= controlValue;
  else interface->controlWord &= (~controlValue);
}

static inline bool CheckState( CANInterface* interface, enum States stateValue )
{
  if( interface == NULL ) return false;
  
  if( (interface->statusWord & stateValue) != 0 ) return true;
  else return false;
}

static inline int ReadSingleValue( CANInterface* interface, uint16_t index, uint8_t subIndex )
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

  int value = payload[ 7 ] * 0x1000000 + payload[ 6 ] * 0x10000 + payload[ 5 ] * 0x100 + payload[4];

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
