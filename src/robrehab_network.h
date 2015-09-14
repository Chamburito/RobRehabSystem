#ifndef ROBREHAB_NETWORK_H
#define ROBREHAB_NETWORK_H

#ifdef _CVI_
  #include "ip_network/cvirte_ip_connection.h"
#else
  #include "ip_network/async_ip_connection.h"
#endif

#include <cvinetv.h>
#include <utility.h>
#include <toolbox.h>
#include "Common.h"

//#include "network_axis.h"

#include "axis_control.h"

#include "klib/kvec.h"

#include "debug/async_debug.h"

static int infoServerConnectionID;
static int dataServerConnectionID;

static kvec_t( int ) infoClientsList;
static kvec_t( int ) dataClientsList;

typedef struct _NetworkAxisController
{
  int axisID, clientID;
  //TrajectoryPlanner* trajectoryPlanner;
} 
NetworkAxisController;

/*KHASH_MAP_INIT_INT( AxisInt, NetworkAxisController )
static khash_t( AxisInt )* networkControllersList;*/
static kvec_t( NetworkAxisController ) networkControllersList;

static char axesInfoString[ IP_CONNECTION_MSG_LEN ] = ""; // String containing used axes names

enum { ROBOT_POSITION, ROBOT_VELOCITY, ROBOT_SETPOINT, ROBOT_ERROR, ROBOT_STIFFNESS, ROBOT_TORQUE, MAX_STIFFNESS, PATIENT_STIFFNESS, PATIENT_TORQUE, EMG_AGONIST, EMG_ANTAGONIST, DISPLAY_VALUES_NUMBER };

/*static CNVData data = 0;

static double maxStiffness;
static CNVSubscriber gMaxToggleSubscriber, gMinToggleSubscriber;
void CVICALLBACK ChangeStateDataCallback( void*, CNVData, void* );

static CNVBufferedWriter gWavePublisher;*/

int RobRehabNetwork_Init()
{
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "Initializing RobRehab Network on thread %x", CmtGetCurrentThreadID() );
  if( (infoServerConnectionID = AsyncIPConnection_Open( NULL, "50000", TCP )) == -1 )
    return -1;
  if( (dataServerConnectionID = AsyncIPConnection_Open( NULL, "50001", UDP )) == -1 )
  {
    AsyncIPConnection_Close( infoServerConnectionID );
    return -1;
  }
  
  DEBUG_EVENT( 1, "Received server connection IDs: %d (State), %d (Data)", infoServerConnectionID, dataServerConnectionID );
  
  kv_init( infoClientsList );
  kv_init( dataClientsList );
  
  kv_init( networkControllersList );
  
  SET_PATH( "../config/" );
  
  size_t shmAxesNumber = 0;
  
  FileParser parser = JSONParser;
  int configFileID = parser.LoadFile( "shared_axes" );
  if( configFileID != -1 )
  {
    if( parser.HasKey( configFileID, "axes" ) )
    {
      size_t shmAxesNumber = parser.GetListSize( configFileID, "axes" );
      
      DEBUG_PRINT( "List size: %u", shmAxesNumber );
      
      char searchPath[ FILE_PARSER_MAX_PATH_LENGTH ];
      for( size_t shmAxisDataIndex = 0; shmAxisDataIndex < shmAxesNumber; shmAxisDataIndex++ )
      {
        sprintf( searchPath, "axes.%u", shmAxisDataIndex );
        char* deviceName = parser.GetStringValue( configFileID, searchPath );
        
        if( deviceName != NULL )
        {
          DEBUG_PRINT( "found shared axis %s", deviceName );
          
          if( strlen( axesInfoString ) > 0 ) strcat( axesInfoString, "|" );
          snprintf( &(axesInfoString[ strlen( axesInfoString ) ]), IP_CONNECTION_MSG_LEN, "%u:%s", shmAxisDataIndex, deviceName );
          
          int shmAxisControlDataID = SHMAxisControl.InitControllerData( deviceName );
          if( shmAxisControlDataID != -1 )
          {
            NetworkAxisController* newController = kv_pushp( NetworkAxisController, networkControllersList );
            
            newController->axisID = shmAxisControlDataID;
            newController->clientID = -1;
            //newController->trajectoryPlanner = TrajectoryPlanner_Init();
            
            DEBUG_PRINT( "got axis ID %d", newController->axisID );
          }
        }
      }
    }
    
    parser.UnloadFile( configFileID );
  }

  //int status = 0;
  
  // Create network variable connections.
	/*status = CNVCreateSubscriber( "\\\\localhost\\" PROCESS "\\" MAX_TOGGLE_VARIABLE, ChangeStateDataCallback, 0, 0, 10000, 0, &gMaxToggleSubscriber );
	if( status != 0 ) DEBUG_PRINT( "%s", CNVGetErrorDescription( status ) );
	status = CNVCreateSubscriber( "\\\\localhost\\" PROCESS "\\" MIN_TOGGLE_VARIABLE, ChangeStateDataCallback, 0, 0, 10000, 0, &gMinToggleSubscriber );
	if( status != 0 ) DEBUG_PRINT( "%s", CNVGetErrorDescription( status ) );
  
  status = CNVCreateBufferedWriter( "\\\\localhost\\" PROCESS "\\" WAVE_VARIABLE, 0, 0, 0, 10, 10000, 0, &gWavePublisher );
	if( status != 0 ) DEBUG_PRINT( "%s", CNVGetErrorDescription( status ) );*/
  
  DEBUG_EVENT( 0, "RobRehab Network initialized on thread %x", THREAD_ID );
  
  return 0;
}

void RobRehabNetwork_End()
{
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "ending RobRehab Network on thread %x", THREAD_ID );
  
  AsyncIPConnection_Close( infoServerConnectionID );
  AsyncIPConnection_Close( dataServerConnectionID );
  
  kv_destroy( infoClientsList );
  kv_destroy( dataClientsList );
  
  for( size_t controllerID = 0; controllerID < kv_size( networkControllersList ); controllerID++ )
  {
    TrajectoryPlanner_End( networkControllersList[ controllerID ].trajectoryPlanner );
    AxisControl.EndController( networkControllersList[ controllerID ].axisID );
  }
  kv_destroy( networkControllersList );
  
  /*if( data ) CNVDisposeData( data );
  if( gMaxToggleSubscriber ) CNVDispose( gMaxToggleSubscriber );
	if( gMinToggleSubscriber ) CNVDispose( gMinToggleSubscriber );
  if( gWavePublisher ) CNVDispose( gWavePublisher );
	CNVFinish();*/
  
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "RobRehab Network ended on thread %x", THREAD_ID );
}

static void UpdateClientInfo( int );
static void UpdateClientData( int );

void RobRehabNetwork_Update()
{
  static int newInfoClientID, newDataClientID;
  
  if( (newInfoClientID = AsyncIPConnection_GetClient( infoServerConnectionID )) != -1 )
  {
    AsyncIPConnection_WriteMessage( newInfoClientID, axesInfoString );
    kv_push( int, infoClientsList, newInfoClientID );
  }
  
  if( (newDataClientID = AsyncIPConnection_GetClient( dataServerConnectionID )) != -1 )
    kv_push( int, dataClientsList, newDataClientID );
  
  for( size_t clientIndex = 0; clientIndex < kv_size( infoClientsList ); clientIndex++ )
    UpdateClientInfo( infoClientsList[ clientIndex ] );
  
  for( size_t clientIndex = 0; clientIndex < kv_size( dataClientsList ); clientIndex++ )
    UpdateClientData( dataClientsList[ clientIndex ] );
  
  for( size_t controllerID = 0; controllerID < kv_size( networkControllersList ); controllerID++ )
  {
    //double* controlMeasuresList = AxisControl.GetMeasuresList( controllerID );
    double* controlParametersList = AxisControl.GetSetpointsList( controllerID );

    //double* targetList = TrajectoryPlanner_GetTargetList( networkControllersList[ controllerID ].trajectoryPlanner );
    
    //DEBUG_PRINT( "\tnext setpoint: %lf (time: %lf)", targetList[ TRAJECTORY_POSITION ], targetList[ TRAJECTORY_TIME ] );
    
    //AESControl_SetSetpoint( controllerID, targetList[ TRAJECTORY_POSITION ] );
    AxisControl.SetSetpoint( controllerID, /*targetList[ TRAJECTORY_POSITION ]*/controlParametersList[ CONTROL_REFERENCE ] );

    //Gamb
    /*static size_t valuesCount;
    static double dataArray[ DISPLAY_VALUES_NUMBER * NUM_POINTS ];
    static size_t arrayDims = DISPLAY_VALUES_NUMBER * NUM_POINTS;

    dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + ROBOT_POSITION ] = controlMeasuresList[ CONTROL_POSITION ];
    dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + ROBOT_VELOCITY ] = controlMeasuresList[ CONTROL_VELOCITY ];
    dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + ROBOT_SETPOINT ] = controlParametersList[ CONTROL_REFERENCE ];
    dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + ROBOT_ERROR ] = controlMeasuresList[ CONTROL_ERROR ]; //Timing_GetExecTimeSeconds();
    dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + ROBOT_STIFFNESS ] = controlParametersList[ CONTROL_STIFFNESS ];
    dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + ROBOT_TORQUE ] = controlMeasuresList[ CONTROL_FORCE ];
    dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + MAX_STIFFNESS ] = maxStiffness;
    dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + PATIENT_TORQUE ] = 0.0;//EMGAESControl_GetTorque( controllerID );
    dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + PATIENT_STIFFNESS ] = 0.0;//EMGAESControl_GetStiffness( controllerID );
    dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + EMG_AGONIST ] = 0.0;//EMGProcessing_GetMuscleActivation( 0 );
    dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + EMG_ANTAGONIST ] = 0.0;//EMGProcessing_GetMuscleActivation( 1 );

    //DEBUG_PRINT( "EMG activation 1: %.3f - activation 2: %.3f - stiffness: %.3f", dataArray[ valuesCount * CONTROL_VALUES_NUMBER + AXIS_CURRENT ],
    //                                                                              dataArray[ valuesCount * CONTROL_VALUES_NUMBER + AXIS_TENSION ], motorParametersList[ CONTROL_STIFFNESS ] );

    valuesCount++;

    if( valuesCount >= NUM_POINTS )
    {
      CNVCreateArrayDataValue( (CNVData*) &data, CNVDouble, dataArray, 1, &arrayDims );
      CNVPutDataInBuffer( gWavePublisher, data, 1000 );
      valuesCount = 0;
    }*/
  }
}

const size_t CONTROLLER_INFO_SIZE = 2
const uint8_t ENABLE_MASK = ( 1 << 7 );         // b10000000                                    
const uint8_t DISABLE_MASK = ( 1 << 6 );        // b01000000
const uint8_t RESET_MASK = ( 1 << 5 );          // b00100000
const uint8_t CALIBRATE_MASK = ( 1 << 4 );      // b00010000
static void UpdateClientInfo( int clientID )
{
  static char messageOut[ IP_CONNECTION_MSG_LEN ];
  
  char* messageIn = AsyncIPConnection_ReadMessage( clientID );
  
  if( messageIn != NULL ) 
  {
    DEBUG_UPDATE( "received input message: %s", messageIn );
    
    memset( messageOut, 0, sizeof(messageOut) );
    size_t stateBytesCount = 0;
    
    size_t infosNumber = strlen( messageIn ) / CONTROLLER_INFO_SIZE;
    for( size_t infoIndex = 0; infoIndex < infosNumber; infoIndex++ )
    {
      uint8_t controllerID = (uint8_t) messageIn[ infoIndex * CONTROLLER_INFO_SIZE ];
      if( controllerID < 0 || controllerID >= kv_size( networkControllersList ) ) continue;
      
      uint8_t commandData = (uint8_t) messageIn[ infoIndex * CONTROLLER_INFO_SIZE + 2 ];
      
      DEBUG_UPDATE( "parsing axis %u commands: %x", controllerID, commandData );
      
      if( commandData & ENABLE_MASK ) 
      {
        DEBUG_PRINT( "enabling controller %u", controllerID );
        AxisControl.Enable( controllerID );
      }
      if( commandData & DISABLE_MASK )
      {
        DEBUG_PRINT( "disabling controller %u", controllerID );
        AxisControl.Disable( controllerID );
      }
      if( commandData & RESET_MASK )
      {
        DEBUG_PRINT( "reseting controller %u", controllerID );
        AxisControl.Reset( controllerID );
        networkControllersList[ controllerID ].dataClientID = -1;
      }
      if( commandData & CALIBRATE_MASK )
      {
        DEBUG_PRINT( "calibrating controller %u", controllerID );
        AxisControl.Calibrate( controllerID );
      }
      
      messageOut[ stateBytesCount++ ] = controllerID;
      messageOut[ stateBytesCount ] |= ( ENABLE_MASK & (uint8_t) AxisControl.IsEnabled( controllerID ) );
      stateBytesCount++;
    }
  
    if( stateBytesCount > 0 ) 
    {
      DEBUG_UPDATE( "sending message %s to client %d (%u bytes)", messageOut, clientID, stateBytesCount );
      AsyncIPConnection_WriteMessage( clientID, messageOut );
    }
  }
  
  return 0;
}

static void UpdateClientData( int clientID )
{
  static char messageOut[ IP_CONNECTION_MSG_LEN ];
  
  static double setpoint, setpointDerivative, setpointsInterval;
  
  char* messageIn = AsyncIPConnection_ReadMessage( dataClient->clientID );
  
  if( messageIn != NULL ) 
  {
    DEBUG_UPDATE( "received input message: %s", messageIn );
    
    char mask = messageIn[ 0 ];
    
    float setpointPosition;
    memcpy( &setpointPosition, messageIn[ 1 ], 4 );
    
    AxisControl.SetSetpoint( 0, setpointPosition );
    AxisControl.SetImpedance( 0, setpointStiffness, setpointDamping );
  
    for( char* axisCommand = strtok( messageIn, ":" ); axisCommand != NULL; axisCommand = strtok( NULL, ":" ) )
    {
      unsigned int controllerID = (unsigned int) strtoul( axisCommand, &axisCommand, 0 );
    
      if( controllerID < 0 || controllerID >= 1/*AESControl_GetDevicesNumber()*/ ) continue;
      
      NetworkAxisController* networkAxis = &(networkControllersList[ controllerID ]);
      
      if( networkAxis->dataClientID == -1 )
      {
        DEBUG_PRINT( "new client for axis %u: %d", controllerID, clientID );
        networkAxis->dataClientID = clientID;
      }
      else if( networkAxis->dataClientID != clientID ) continue;
      
      DEBUG_UPDATE( "parsing axis %u command \"%s\"", controllerID, axisCommand );
      
      setpoint = strtod( axisCommand, &axisCommand );
      setpointDerivative = strtod( axisCommand, &axisCommand );
      setpointsInterval = strtod( axisCommand, &axisCommand );
      
      //DEBUG_PRINT( "received setpoint: %f", setpoint );
      
      TrajectoryPlanner_SetCurve( networkAxis->trajectoryPlanner, setpoint, setpointDerivative, setpointsInterval );
    }
  }
  
  strcpy( messageOut, "" );
  for( size_t controllerID = 0; controllerID < 1/*AESControl_GetDevicesNumber()*/; controllerID++ )
  {
    double* controlMeasuresList = /*AESControl_GetMeasuresList*/AxisControl.GetMeasuresList( controllerID );
    if( controlMeasuresList != NULL )
    {
      if( strlen( messageOut ) > 0 ) strcat( messageOut, ":" );

      sprintf( &messageOut[ strlen( messageOut ) ], "%u", controllerID );

      for( size_t dimensionIndex = 0; dimensionIndex < AXIS_FORCE; dimensionIndex++ )
        sprintf( &messageOut[ strlen( messageOut ) ], " %f", controlMeasuresList[ dimensionIndex ] );
    }
  }
  
  if( strlen( messageOut ) > 0 ) 
  {
    DEBUG_UPDATE( "sending message %s to client %d", messageOut, dataClient->clientID );
    AsyncIPConnection_WriteMessage( dataClient->clientID, messageOut );
  }
  
  return 0;
}

/*static void WriteAxisControlState( unsigned int controllerID, const char* command )
{
  bool motorEnabled = (bool) strtoul( command, &command, 0 );

  if( motorEnabled ) AESControl_EnableMotor( controllerID );
  else AESControl_DisableMotor( controllerID );

  bool reset = (bool) strtoul( command, NULL, 0 );

  if( reset ) AESControl_Reset( controllerID );
}

static char* ReadAxisControlState( unsigned int controllerID )
{
  const size_t STATE_MAX_LEN = 2;
  static char readout[ STATE_MAX_LEN * AXIS_STATES_NUMBER + 1 ];
  
  bool* motorStatesList = AESControl_GetMotorStatus( controllerID );
  if( motorStatesList != NULL )
  {
    strcpy( readout, "" );
    for( size_t stateIndex = 0; stateIndex < AXIS_STATES_NUMBER; stateIndex++ )
      strcat( readout, ( motorStatesList[ stateIndex ] == true ) ? "1 " : "0 " );
    readout[ strlen( readout ) - 1 ] = '\0';
  }
  
  DEBUG_UPDATE( "measure readout: %s", readout );
  
  return readout;
}

static void WriteAxisControlData( unsigned int controllerID, const char* command )
{
  static double setpointsList[ TRAJECTORY_VALUES_NUMBER ];
  static size_t setpointsCount;
  
  if( clientID != -1 ) return;
  
  networkControllersList[ controllerID ].dataClient = clientID;

  setpointsCount = 0;
  while( *command != '\0' && setpointsCount < TRAJECTORY_VALUES_NUMBER )
  {
    setpointsList[ setpointsCount ] = strtod( command, &command );
    setpointsCount++;
  }

  TrajectoryPlanner_SetCurve( networkControllersList[ controllerID ].trajectoryPlanner, setpointsList );
}

static char* ReadAxisControlData( unsigned int controllerID )
{
  const size_t VALUE_MAX_LEN = 10;
  
  static char readout[ VALUE_MAX_LEN * AXIS_FORCE + 1 ];
  
  double* controlMeasuresList = AESControl_GetMeasuresList( controllerID );
  if( controlMeasuresList != NULL )
  {
    //double* jointMeasuresList = EMGAESControl_ApplyGains( controllerID, maxStiffness );
      
    strcpy( readout, "" );
    for( size_t dimensionIndex = 0; dimensionIndex < AXIS_FORCE; dimensionIndex++ )
      sprintf( &readout[ strlen( readout ) ], "%.3f ", controlMeasuresList[ dimensionIndex ] );
    readout[ strlen( readout ) - 1 ] = '\0';
  }
  
  DEBUG_UPDATE( "measure readout: %s", readout );
  
  return readout;
}*/


void CVICALLBACK ChangeStateDataCallback( void* handle, CNVData data, void* callbackData )
{
	/*int messageCode;
  CNVGetScalarDataValue( data, CNVInt32, &messageCode );
  
  unsigned int controllerID = (unsigned int) ( messageCode & 0x000000ff );
  bool enabled = (bool) ( ( messageCode & 0x0000ff00 ) / 0x100 );
  unsigned int muscleGroup = (unsigned int) ( ( messageCode & 0x00ff0000 ) / 0x10000 ); 

	if( handle == gMaxToggleSubscriber )
	{
    EMGAESControl_ChangeState( controllerID, muscleGroup, enabled ? EMG_CONTRACTION_PHASE : EMG_ACTIVATION_PHASE ); 
    
    //DEBUG_PRINT( "axis %u %s %s EMG contraction phase", controllerID, enabled ? "starting" : "ending", ( muscleGroup == MUSCLE_AGONIST ) ? "agonist" : "antagonist" );
	}
	else if( handle == gMinToggleSubscriber )
	{
    EMGAESControl_ChangeState( controllerID, muscleGroup, enabled ? EMG_RELAXATION_PHASE : EMG_ACTIVATION_PHASE );
    
    //DEBUG_PRINT( "axis %u %s %s EMG relaxation phase", controllerID, enabled ? "starting" : "ending", ( muscleGroup == MUSCLE_AGONIST ) ? "agonist" : "antagonist" );
	}*/
}


#endif //ROBREHAB_NETWORK_H
