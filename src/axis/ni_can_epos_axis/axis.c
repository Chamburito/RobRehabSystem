 /* 
 * File:   AXIS.h
 * Author: GUILHERME FERNANDES
 *
 * ESTA FUN√á√ÉO REALIZA O CONTROLE DE UM EIXO DA EPOS
 * 
 * Created on 26 de Janeiro de 2012, 19:34
 */
 
#include "axis_interface.h"
#include "ni_can_epos_axis/can_network.h"

#ifdef WIN32
  #include "time/timing_windows.h"
#else
  #include "time/timing_unix.h"
#endif
 
#include "klib/khash.h"
 
#include "debug/async_debug.h"
 
#ifdef _CVI_
  #include <ansi_c.h>
#else
  #include <stdio.h>
  #include <string.h>
  #include <stdlib.h>
  #include <malloc.h>
#endif

#include <stdbool.h>

/*enum OperationMode { HOMMING_MODE, PROFILE_VELOCITY_MODE, PROFILE_POSITION_MODE, POSITION_MODE, 
                     VELOCITY_MODE, CURRENT_MODE, MASTER_ENCODER_MODE, STEP_MODE, AXIS_MODES_NUMBER };
static const uint8_t OPERATION_MODES[ AXIS_MODES_NUMBER ] = { 0x06, 0x03, 0x01, 0xFF, 0xFE, 0xFD, 0xFB, 0xFA };*/

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
  double lastSetpoint;
}
CANInterface;

KHASH_MAP_INIT_INT( CANInt, CANInterface* )
static khash_t( CANInt )* interfacesList = NULL;

IMPLEMENT_INTERFACE( Axis, AXIS_INTERFACE_FUNCTIONS )

static inline CANInterface* LoadInterfaceData( unsigned int );
static inline void UnloadInterfaceData( CANInterface* );

static inline void SetControl( CANInterface*, enum Controls, bool );
static inline bool CheckState( CANInterface*, enum States );
static inline int ReadSingleValue( CANInterface*, uint16_t, uint8_t );
static inline void WriteSingleValue( CANInterface*, uint16_t, uint8_t, int );
static inline void EnableDigitalOutput( CANInterface*, bool );
                                                    
// Create CAN controlled DC motor handle
int Connect( unsigned int nodeID )
{
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "trying to connect CAN interface %u", nodeID );
  
  if( interfacesList == NULL )
  {
    CANNetwork_Start( CAN_DATABASE, CAN_CLUSTER );
    interfacesList = kh_init( CANInt );
  }
  
  DEBUG_PRINT( "CAN EPOS config key: %d", (int) nodeID );
  
  int insertionStatus;
  khiter_t newInterfaceIndex = kh_put( CANInt, interfacesList, (int) nodeID, &insertionStatus );
  if( insertionStatus > 0 )
  {
    DEBUG_PRINT( "inserting CAN EPOS config key on list: %d", (int) nodeID );
    
    kh_value( interfacesList, newInterfaceIndex ) = LoadInterfaceData( nodeID );
    if( kh_value( interfacesList, newInterfaceIndex ) == NULL )
    {
      Disconnect( (int) nodeID );
      return -1;
    }

    CANInterface* newInterface = kh_value( interfacesList, newInterfaceIndex );
    
    //EnableDigitalOutput( newInterface, true ); // Valor est· errado !! (0x6060 È operation mode)
    
    SetControl( newInterface, ENABLE_VOLTAGE, true );
    SetControl( newInterface, QUICK_STOP, true );
    
    Disable( newInterfaceIndex );
    
    /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "created CAN EPOS interface %u", nodeID );
  }
  if( insertionStatus == 0 )
    ERROR_EVENT( "CAN EPOS interface %u already exists", nodeID );
    
  return (int) kh_key( interfacesList, newInterfaceIndex );
}

void Disconnect( int interfaceID )
{
  khint_t interfaceIndex = kh_get( CANInt, interfacesList, (khint_t) interfaceID );
  if( interfaceIndex == kh_end( interfacesList ) ) return;

  DEBUG_PRINT( "disconnecting CAN EPOS device %d", interfaceID );
  
  CANInterface* interface = kh_value( interfacesList, interfaceIndex );
  
  UnloadInterfaceData( interface );
  
  kh_del( CANInt, interfacesList, interfaceID );
  
  if( kh_size( interfacesList ) == 0 )
  {
    CANNetwork_Stop();
    
    kh_destroy( CANInt, interfacesList );
    interfacesList = NULL;
  }
}

void Enable( int interfaceID )
{
  khint_t interfaceIndex = kh_get( CANInt, interfacesList, (khint_t) interfaceID );
  if( interfaceIndex == kh_end( interfacesList ) ) return;
  
  CANInterface* interface = kh_value( interfacesList, interfaceIndex );
    
  SetControl( interface, SWITCH_ON, false );
  SetControl( interface, ENABLE_OPERATION, false );
  WriteSetpoint( interfaceID, interface->lastSetpoint );
  
  Timing_Delay( 200 );
  
  SetControl( interface, SWITCH_ON, true );
  WriteSetpoint( interfaceID, interface->lastSetpoint );
  
  Timing_Delay( 200 );
  
  SetControl( interface, ENABLE_OPERATION, true );
  WriteSetpoint( interfaceID, interface->lastSetpoint );
}

void Disable( int interfaceID )
{
  DEBUG_EVENT( 0, "disabling CAN EPOS device %d", interfaceID );
  
  khint_t interfaceIndex = kh_get( CANInt, interfacesList, (khint_t) interfaceID );
  if( interfaceIndex == kh_end( interfacesList ) ) return;
  
  CANInterface* interface = kh_value( interfacesList, interfaceIndex );
    
  SetControl( interface, SWITCH_ON, true );
  SetControl( interface, ENABLE_OPERATION, false );
  WriteSetpoint( interfaceID, interface->lastSetpoint );
  
  Timing_Delay( 200 );
  
  SetControl( interface, SWITCH_ON, false );
  WriteSetpoint( interfaceID, interface->lastSetpoint );
}

void Reset( int interfaceID )
{
  khint_t interfaceIndex = kh_get( CANInt, interfacesList, (khint_t) interfaceID );
  if( interfaceIndex == kh_end( interfacesList ) ) return;
  
  CANInterface* interface = kh_value( interfacesList, interfaceIndex );
    
  SetControl( interface, FAULT_RESET, true );
  WriteSetpoint( interfaceID, interface->lastSetpoint );
  
  Timing_Delay( 200 );
  
  SetControl( interface, FAULT_RESET, false );
  WriteSetpoint( interfaceID, interface->lastSetpoint );
}

static bool IsEnabled( int interfaceID )
{
  khint_t interfaceIndex = kh_get( CANInt, interfacesList, (khint_t) interfaceID );
  if( interfaceIndex == kh_end( interfacesList ) ) return false;
  
  CANInterface* interface = kh_value( interfacesList, interfaceIndex );
    
  return ( CheckState( interface, SWITCHED_ON ) && CheckState( interface, OPERATION_ENABLED ) );
}

static bool HasError( int interfaceID )
{
  khint_t interfaceIndex = kh_get( CANInt, interfacesList, (khint_t) interfaceID );
  if( interfaceIndex == kh_end( interfacesList ) ) return false;
  
  CANInterface* interface = kh_value( interfacesList, interfaceIndex );
    
  return ( CheckState( interface, FAULT ) );
}

static bool ReadMeasures( int interfaceID, double measuresList[ AXIS_VARS_NUMBER ] )
{
  // Create reading buffer
  static uint8_t payload[ 8 ];

  khint_t interfaceIndex = kh_get( CANInt, interfacesList, (khint_t) interfaceID );
  if( interfaceIndex == kh_end( interfacesList ) ) return false;
  
  CANInterface* interface = kh_value( interfacesList, interfaceIndex );
    
  CANNetwork_Sync();
  
  // Read values from PDO01 (Position, Current and Status Word) to buffer
  CANFrame_Read( interface->readFramesList[ PDO01 ], payload );  
  // Update values from PDO01
  measuresList[ AXIS_ENCODER ] = payload[ 3 ] * 0x1000000 + payload[ 2 ] * 0x10000 + payload[ 1 ] * 0x100 + payload[ 0 ];
  int currentHEX = payload[ 5 ] * 0x100 + payload[ 4 ];
  double currentMA = currentHEX - ( ( currentHEX >= 0x8000 ) ? 0xFFFF : 0 );
  measuresList[ AXIS_CURRENT ] = currentMA / 1000.0;
  
  interface->statusWord = payload[ 7 ] * 0x100 + payload[ 6 ];
  
  // Read values from PDO02 (Velocity and Tension) to buffer
  CANFrame_Read( interface->readFramesList[ PDO02 ], payload );  
  // Update values from PDO02
  double velocityRPM = payload[ 3 ] * 0x1000000 + payload[ 2 ] * 0x10000 + payload[ 1 ] * 0x100 + payload[ 0 ];
  measuresList[ AXIS_RPS ] = velocityRPM * 60.0;
  
  double analog = payload[ 5 ] * 0x100 + payload[ 4 ];
  measuresList[ AXIS_ANALOG ] = analog;
  
  //DEBUG_PRINT( "encoder: %.5f (interface %u)", measuresList[ AXIS_ENCODER ], (khint_t) interfaceID );
  
  return true;
}

bool WriteSetpoint( int interfaceID, double setpoint )
{
  // Create writing buffer
  static uint8_t payload[ 8 ];
  
  khint_t interfaceIndex = kh_get( CANInt, interfacesList, (khint_t) interfaceID );
  if( interfaceIndex == kh_end( interfacesList ) ) return false;
  
  CANInterface* interface = kh_value( interfacesList, interfaceIndex );
  
  interface->lastSetpoint = setpoint;
  
  int encoderSetpoint = (int) setpoint;
  int16_t currentSetpointHEX = (int16_t) setpoint + ( ( setpoint < 0.0 ) ? 0xFFFF : 0 );
  
  // Set values for PDO01 (Position Setpoint and Control Word)
  payload[ 0 ] = (uint8_t) ( encoderSetpoint & 0x000000FF );
  payload[ 1 ] = (uint8_t) ( ( encoderSetpoint & 0x0000FF00 ) / 0x100 );
  payload[ 2 ] = (uint8_t) ( ( encoderSetpoint & 0x00FF0000 ) / 0x10000 );
  payload[ 3 ] = (uint8_t) ( ( encoderSetpoint & 0xFF000000 ) / 0x1000000 );
  payload[ 4 ] = (uint8_t) ( currentSetpointHEX & 0x000000FF );
  payload[ 5 ] = (uint8_t) ( ( currentSetpointHEX & 0x0000FF00 ) / 0x100 ); 
  payload[ 6 ] = (uint8_t) ( interface->controlWord & 0x000000FF );
  payload[ 7 ] = (uint8_t) ( ( interface->controlWord & 0x0000FF00 ) / 0x100 ); 
  
  // Write values from buffer to PDO01 
  CANFrame_Write( interface->writeFramesList[ PDO01 ], payload );
  
  int velocitySetpointRPM = (int) ( setpoint / 60.0 );
  int16_t digitalOutput = (int16_t) setpoint;
  
  //DEBUG_PRINT( "velocity setpoint: %d", rawVelocitySetpoint );
  
  // Set values for PDO02 (Velocity Setpoint and Digital Output)
  payload[ 0 ] = (uint8_t) ( velocitySetpointRPM & 0x000000FF );
  payload[ 1 ] = (uint8_t) ( ( velocitySetpointRPM & 0x0000FF00 ) / 0x100 );
  payload[ 2 ] = (uint8_t) ( ( velocitySetpointRPM & 0x00FF0000 ) / 0x10000 );
  payload[ 3 ] = (uint8_t) ( ( velocitySetpointRPM & 0xFF000000 ) / 0x1000000 );
  payload[ 4 ] = (uint8_t) ( digitalOutput & 0x000000FF );
  payload[ 5 ] = (uint8_t) ( ( digitalOutput & 0x0000FF00 ) / 0x100 ); 
  payload[ 6 ] = ( 0 & 0x000000FF );
  payload[ 7 ] = ( 0 & 0x0000FF00 ) / 0x100; 
  
  // Write values from buffer to PDO01
  CANFrame_Write( interface->writeFramesList[ PDO02 ], payload );
  
  CANNetwork_Sync();
  
  return true;
}

static const int OPERATION_MODES[ AXIS_VARS_NUMBER ] = { 0xFF, 0xFE, 0xFD, 0x00 };
static void SetOperationMode( int interfaceID, enum AxisVariables mode )
{
  khint_t interfaceIndex = kh_get( CANInt, interfacesList, (khint_t) interfaceID );
  if( interfaceIndex == kh_end( interfacesList ) ) return;
  
  if( mode < 0 || mode >= AXIS_VARS_NUMBER ) return;
  
  CANInterface* interface = kh_value( interfacesList, interfaceIndex );
  
  WriteSingleValue( interface, 0x6060, 0x00, OPERATION_MODES[ mode ] );
  
  interface->lastSetpoint = 0.0;
}


const size_t ADDRESS_MAX_LENGTH = 16;
static const char* CAN_FRAME_NAMES[ AXIS_FRAME_TYPES_NUMBER ] = { "SDO", "PDO01", "PDO02" };
static inline CANInterface* LoadInterfaceData( unsigned int nodeID )
{
  static char networkAddress[ ADDRESS_MAX_LENGTH ];
  
  CANInterface* newInterface = (CANInterface*) malloc( sizeof(CANInterface) );
  memset( newInterface, 0, sizeof(CANInterface) );
  
  DEBUG_PRINT( "Trying to load CAN interface %u", nodeID );
  
  for( size_t frameID = 0; frameID < AXIS_FRAME_TYPES_NUMBER; frameID++ )
  {
    sprintf( networkAddress, "%s_RX_%02u", CAN_FRAME_NAMES[ frameID ], nodeID );
    newInterface->readFramesList[ frameID ] = CANNetwork_InitFrame( FRAME_IN, "CAN1", networkAddress );

    if( newInterface->readFramesList[ frameID ] == NULL )
      DEBUG_PRINT( "error creating frame %s for CAN interface %u", networkAddress, nodeID );
    
    sprintf( networkAddress, "%s_TX_%02u", CAN_FRAME_NAMES[ frameID ], nodeID );
    newInterface->writeFramesList[ frameID ] = CANNetwork_InitFrame( FRAME_OUT, "CAN2", networkAddress );
    
    if( newInterface->writeFramesList[ frameID ] == NULL )
      DEBUG_PRINT( "error creating frame %s for CAN interface %u", networkAddress, nodeID );
  }

  DEBUG_PRINT( "loaded CAN interface %u", nodeID );
  
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
