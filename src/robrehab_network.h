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

//#include "spline3_interpolation.h"
#include "shm_axis_control.h"
//#include "axis_control.h"

#include "file_parsing/json_parser.h"

#include "klib/kvec.h"

#include "debug/async_debug.h"

static int infoServerConnectionID;
static int dataServerConnectionID;

static kvec_t( int ) infoClientsList;
static char* infoMessageOut;
static size_t infoMessageLength;
const size_t COMMAND_BLOCK_SIZE = 1 + SHM_CONTROL_COMMANDS_NUMBER * sizeof(bool);
const size_t STATE_BLOCK_SIZE = 1 + SHM_CONTROL_STATES_NUMBER * sizeof(bool);

static kvec_t( int ) dataClientsList;
static char* dataMessageOut;
static size_t dataMessageLength;
const size_t PARAMETER_BLOCK_SIZE = 1 + SHM_CONTROL_PARAMETERS_NUMBER * sizeof(float);
const size_t MEASURE_BLOCK_SIZE = 1 + SHM_CONTROL_MEASURES_NUMBER * sizeof(float);

typedef struct _NetworkAxisController
{
  int axisID, clientID;
  //TrajectoryPlanner* trajectoryPlanner;
} 
NetworkAxisController;

/*KHASH_MAP_INIT_INT( AxisInt, NetworkAxisController )
static khash_t( AxisInt )* controllersList;*/
static kvec_t( NetworkAxisController ) controllersList;

static char axesInfoString[ IP_MAX_MESSAGE_LENGTH ] = ""; // String containing used axes names

//enum { ROBOT_POSITION, ROBOT_VELOCITY, ROBOT_SETPOINT, ROBOT_ERROR, ROBOT_STIFFNESS, ROBOT_TORQUE, MAX_STIFFNESS, PATIENT_STIFFNESS, PATIENT_TORQUE, EMG_AGONIST, EMG_ANTAGONIST, DISPLAY_VALUES_NUMBER };

/*static CNVData data = 0;

static double maxStiffness;
static CNVSubscriber gMaxToggleSubscriber, gMinToggleSubscriber;
void CVICALLBACK ChangeStateDataCallback( void*, CNVData, void* );

static CNVBufferedWriter gWavePublisher;*/

int RobRehabNetwork_Init()
{
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "Initializing RobRehab Network on thread %x", THREAD_ID );
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
  
  kv_init( controllersList );
  
  SET_PATH( "../config/" );
  
  FileParser parser = JSONParser;
  int configFileID = parser.LoadFile( "shared_axes" );
  if( configFileID != -1 )
  {
    if( parser.HasKey( configFileID, "axes" ) )
    {
      size_t sharedAxesNumber = parser.GetListSize( configFileID, "axes" );
      
      DEBUG_PRINT( "List size: %u", sharedAxesNumber );
      
      infoMessageLength = sharedAxesNumber * ( ( COMMAND_BLOCK_SIZE > STATE_BLOCK_SIZE ) ? COMMAND_BLOCK_SIZE : STATE_BLOCK_SIZE ); 
      infoMessageOut = (char*) calloc( infoMessageLength, sizeof(char) );
      AsyncIPConnection_SetMessageLength( infoServerConnectionID, infoMessageLength );
      
      dataMessageLength = sharedAxesNumber * ( ( PARAMETER_BLOCK_SIZE > MEASURE_BLOCK_SIZE ) ? PARAMETER_BLOCK_SIZE : MEASURE_BLOCK_SIZE );
      dataMessageOut = (char*) calloc( dataMessageLength, sizeof(char) );
      AsyncIPConnection_SetMessageLength( dataServerConnectionID, dataMessageLength );
      
      char searchPath[ FILE_PARSER_MAX_PATH_LENGTH ];
      for( size_t sharedAxisDataIndex = 0; sharedAxisDataIndex < sharedAxesNumber; sharedAxisDataIndex++ )
      {
        sprintf( searchPath, "axes.%u", sharedAxisDataIndex );
        char* deviceName = parser.GetStringValue( configFileID, searchPath );
        
        if( deviceName != NULL )
        {
          DEBUG_PRINT( "found shared axis %s", deviceName );
          
          if( strlen( axesInfoString ) > 0 ) strcat( axesInfoString, "|" );
          snprintf( &(axesInfoString[ strlen( axesInfoString ) ]), IP_MAX_MESSAGE_LENGTH, "%u:%s", sharedAxisDataIndex, deviceName );
          
          int sharedAxisControlDataID = SHMAxisControl.InitControllerData( deviceName );
          if( sharedAxisControlDataID != -1 )
          {
            NetworkAxisController newController = { .axisID = sharedAxisControlDataID, .clientID = -1 /*, .trajectoryPlanner = TrajectoryPlanner_Init();*/ };
            
            kv_push( NetworkAxisController, controllersList, newController );
            
            DEBUG_PRINT( "got axis ID %d", kv_A( controllersList, kv_size( controllersList ) - 1 ).axisID );
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
  
  for( size_t infoClientIndex = 0; infoClientIndex < kv_size( infoClientsList ); infoClientIndex++ )
    AsyncIPConnection_Close( kv_A( infoClientsList, infoClientIndex ) );
  
  for( size_t dataClientIndex = 0; dataClientIndex < kv_size( dataClientsList ); dataClientIndex++ )
    AsyncIPConnection_Close( kv_A( dataClientsList, dataClientIndex ) );
  
  AsyncIPConnection_Close( infoServerConnectionID );
  AsyncIPConnection_Close( dataServerConnectionID );
  
  kv_destroy( infoClientsList );
  kv_destroy( dataClientsList );
  
  for( size_t controllerID = 0; controllerID < kv_size( controllersList ); controllerID++ )
  {
    //TrajectoryPlanner_End( controllersList[ controllerID ].trajectoryPlanner );
    //AxisControl.EndController( controllersList[ controllerID ].axisID );
    SHMAxisControl.EndControllerData( kv_A( controllersList, controllerID ).axisID );
  }
  kv_destroy( controllersList );
  
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
    UpdateClientInfo( kv_A( infoClientsList, clientIndex ) );
  
  for( size_t clientIndex = 0; clientIndex < kv_size( dataClientsList ); clientIndex++ )
    UpdateClientData( kv_A( dataClientsList, clientIndex ) );
  
  //for( size_t controllerID = 0; controllerID < kv_size( controllersList ); controllerID++ )
  //{
    //double* controlMeasuresList = AxisControl.GetMeasuresList( controllerID );
    //double* controlParametersList = AxisControl.GetSetpointsList( controllerID );

    //double* targetList = TrajectoryPlanner_GetTargetList( controllersList[ controllerID ].trajectoryPlanner );
    
    //DEBUG_PRINT( "\tnext setpoint: %lf (time: %lf)", targetList[ TRAJECTORY_POSITION ], targetList[ TRAJECTORY_TIME ] );
    
    //AESControl_SetSetpoint( controllerID, targetList[ TRAJECTORY_POSITION ] );
    //AxisControl.SetSetpoint( controllerID, /*targetList[ TRAJECTORY_POSITION ]*/controlParametersList[ CONTROL_REFERENCE ] );

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
  //}
}

static void UpdateClientInfo( int clientID )
{
  char* messageIn = AsyncIPConnection_ReadMessage( clientID );
  
  if( messageIn != NULL ) 
  {
    /*DEBUG_UPDATE*/DEBUG_PRINT( "received input message: %s", messageIn );
    
    memset( infoMessageOut, 0, infoMessageLength * sizeof(char) );
    size_t stateBytesCount = 0;
    
    uint8_t commandBlocksNumber = (uint8_t) *(messageIn++);
    for( uint8_t commandBlockIndex = 0; commandBlockIndex < commandBlocksNumber; commandBlockIndex++ )
    {
      uint8_t controllerID = (uint8_t) messageIn[ commandBlockIndex * COMMAND_BLOCK_SIZE ];
      if( controllerID < 0 || controllerID >= kv_size( controllersList ) ) continue;
      
      /*DEBUG_UPDATE*/DEBUG_PRINT( "received axis %u command data", controllerID );
      
      bool* commandsList = (bool*) ( messageIn + commandBlockIndex * COMMAND_BLOCK_SIZE + 1 );
      SHMAxisControl.SetBooleanValues( controllerID, SHM_CONTROL_COMMANDS, commandsList );
      
      bool* statesList = SHMAxisControl.GetBooleanValuesList( controllerID, SHM_CONTROL_STATES, SHM_PEEK, NULL );
      if( statesList != NULL )
      {
        infoMessageOut[ stateBytesCount++ ] = controllerID;
        memcpy( infoMessageOut + stateBytesCount, statesList, sizeof(bool) * SHM_CONTROL_STATES_NUMBER );
        stateBytesCount += sizeof(bool) * SHM_CONTROL_STATES_NUMBER;
      }
    }
  
    if( stateBytesCount > 0 ) 
    {
      /*DEBUG_UPDATE*/DEBUG_PRINT( "sending message %s to client %d (%u bytes)", infoMessageOut, clientID, stateBytesCount );
      AsyncIPConnection_WriteMessage( clientID, infoMessageOut );
    }
  }
}

static void UpdateClientData( int clientID )
{
  char* messageIn = AsyncIPConnection_ReadMessage( clientID );
  
  if( messageIn != NULL ) 
  {
    DEBUG_UPDATE( "received input message: %s", messageIn );
    
    uint8_t parameterBlocksNumber = (uint8_t) *(messageIn++);
    for( uint8_t parameterBlockIndex = 0; parameterBlockIndex < parameterBlocksNumber; parameterBlockIndex++ )
    {
      uint8_t controllerID = (uint8_t) messageIn[ parameterBlockIndex * PARAMETER_BLOCK_SIZE ];
      if( controllerID < 0 || controllerID >= kv_size( controllersList ) ) continue;
      
      NetworkAxisController* networkAxis = &(kv_A( controllersList, controllerID ));
      
      if( networkAxis->clientID == -1 )
      {
        DEBUG_PRINT( "new client for axis %u: %d", controllerID, clientID );
        networkAxis->clientID = clientID;
      }
      else if( networkAxis->clientID != clientID ) continue;
      
      DEBUG_UPDATE( "receiving axis %u parameters", controllerID );
      
      float* parametersList = (float*) ( messageIn + parameterBlockIndex * PARAMETER_BLOCK_SIZE + 1 );
      SHMAxisControl.SetNumericValues( controllerID, SHM_CONTROL_PARAMETERS, parametersList );
      
      //DEBUG_PRINT( "received setpoint: %f", setpoint );
      
      //TrajectoryPlanner_SetCurve( networkAxis->trajectoryPlanner, setpoint, setpointDerivative, setpointsInterval );
    }
  }
  
  memset( dataMessageOut, 0, dataMessageLength * sizeof(char) );
  size_t measureBytesCount = 0;
  for( size_t controllerID = 0; controllerID < kv_size( controllersList ); controllerID++ )
  {
    float* measuresList = SHMAxisControl.GetNumericValuesList( controllerID, SHM_CONTROL_MEASURES, SHM_PEEK, NULL );
    if( measuresList != NULL )
    {
      dataMessageOut[ measureBytesCount++ ] = (uint8_t) controllerID;
      memcpy( dataMessageOut + measureBytesCount, measuresList, sizeof(float) * SHM_CONTROL_MEASURES_NUMBER );
      measureBytesCount += sizeof(float) * SHM_CONTROL_MEASURES_NUMBER;
    }
  }
  
  if( measureBytesCount > 0 ) 
  {
    DEBUG_UPDATE( "sending message %s to client %d (%u bytes)", dataMessageOut, clientID, measureBytesCount );
    AsyncIPConnection_WriteMessage( clientID, dataMessageOut );
  }
}

#endif //ROBREHAB_NETWORK_H
