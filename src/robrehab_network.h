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

#include "network_axis.h"

#include "axis_control.h"

#include "klib/kvec.h"

#include "debug/async_debug.h"

static int infoServerConnectionID;
static int dataServerConnectionID;

static kvec_t( int ) infoClientsList;
static kvec_t( int ) dataClientsList;

typedef struct _NetworkAxis
{
  int axisID;
  int dataClientID;
  TrajectoryPlanner* trajectoryPlanner;
} 
NetworkAxis;

KHASH_MAP_INIT_INT( AxisInt, NetworkAxis )
static khash_t( AxisInt )* networkAxesList;  

static char axesInfoString[ IP_CONNECTION_MSG_LEN ] = ""; // String containing used axes names

enum { ROBOT_POSITION, ROBOT_VELOCITY, ROBOT_SETPOINT, ROBOT_ERROR, ROBOT_STIFFNESS, ROBOT_TORQUE, MAX_STIFFNESS, PATIENT_STIFFNESS, PATIENT_TORQUE, EMG_AGONIST, EMG_ANTAGONIST, DISPLAY_VALUES_NUMBER };

static CNVData data = 0;

static double maxStiffness;
static CNVSubscriber gMaxToggleSubscriber, gMinToggleSubscriber;
void CVICALLBACK ChangeStateDataCallback( void*, CNVData, void* );

static CNVBufferedWriter gWavePublisher;

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
        
        DEBUG_PRINT( "found axis %s", deviceName );
        
        if( kh_get( SHMJointControl, shmJointControlsList, deviceName ) == kh_end( shmJointControlsList ) )
        {
          if( (emgJointControlID = EMGJointControl.InitJoint( deviceName )) != -1 )
          {
            if( (shmAxisControlDataID = SHMAxisControl.InitControllerData( deviceName )) != -1 )
            {
              int insertionStatus;
              khint_t shmJointControlID = kh_put( SHMJointControl, shmJointControlsList, deviceName, &insertionStatus );
              
              DEBUG_PRINT( "Got hash table iterator %u (insertion status: %d)", shmJointControlID, insertionStatus );

              kh_value( shmJointControlsList, shmJointControlID ).shmAxisControlDataID = shmAxisControlDataID;
              kh_value( shmJointControlsList, shmJointControlID ).emgJointControlID = emgJointControlID;
              
              DEBUG_PRINT( "Got joint %d and axis %d", kh_value( shmJointControlsList, shmJointControlID ).emgJointControlID,
                                                       kh_value( shmJointControlsList, shmJointControlID ).shmAxisControlDataID );
            }
            else loadError = true;
          }
          else loadError = true;
        
          if( loadError )
          {
            SHMAxisControl.EndControllerData( shmAxisControlDataID );
            EMGJointControl.EndJoint( emgJointControlID );
            return -1;
          }
        }
      }
    }
    
    parser.UnloadFile( configFileID );
  }
  
  for( size_t deviceID = 0; deviceID < 1/*AESControl_GetDevicesNumber()*/; deviceID++ )
  {
    /*char* deviceName = AESControl_GetDeviceName( deviceID );
    if( deviceName != NULL )
    {
      if( strlen( axesInfoString ) > 0 ) strcat( axesInfoString, "|" );
      snprintf( &axesInfoString[ strlen( axesInfoString ) ], IP_CONNECTION_MSG_LEN, "%u:%s", deviceID, deviceName );
    }*/
    strcpy( axesInfoString, "0:ankle" );
    
    networkAxesList[ deviceID ].axisID = AxisControl.InitController( "ankle" );
    networkAxesList[ deviceID ].dataClientID = -1;
    networkAxesList[ deviceID ].trajectoryPlanner = TrajectoryPlanner_Init();
  }

  int status = 0;
  
  // Create network variable connections.
	status = CNVCreateSubscriber( "\\\\localhost\\" PROCESS "\\" MAX_TOGGLE_VARIABLE, ChangeStateDataCallback, 0, 0, 10000, 0, &gMaxToggleSubscriber );
	if( status != 0 ) DEBUG_PRINT( "%s", CNVGetErrorDescription( status ) );
	status = CNVCreateSubscriber( "\\\\localhost\\" PROCESS "\\" MIN_TOGGLE_VARIABLE, ChangeStateDataCallback, 0, 0, 10000, 0, &gMinToggleSubscriber );
	if( status != 0 ) DEBUG_PRINT( "%s", CNVGetErrorDescription( status ) );
  
  status = CNVCreateBufferedWriter( "\\\\localhost\\" PROCESS "\\" WAVE_VARIABLE, 0, 0, 0, 10, 10000, 0, &gWavePublisher );
	if( status != 0 ) DEBUG_PRINT( "%s", CNVGetErrorDescription( status ) );
  
  DEBUG_EVENT( 0, "RobRehab Network initialized on thread %x", CmtGetCurrentThreadID() );
  
  return 0;
}

void RobRehabNetwork_End()
{
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "ending RobRehab Network on thread %x", CmtGetCurrentThreadID() );
  
  AsyncIPConnection_Close( infoServerConnectionID );
  AsyncIPConnection_Close( dataServerConnectionID );
  
  kv_destroy( infoClientsList );
  kv_destroy( dataClientsList );
  
  for( size_t deviceID = 0; deviceID < 1/*AESControl_GetDevicesNumber()*/; deviceID++ )
  {
    TrajectoryPlanner_End( networkAxesList[ deviceID ].trajectoryPlanner );
    AxisControl.EndController( networkAxesList[ deviceID ].axisID );
  }
  free( networkAxesList );
  
  if( data ) CNVDisposeData( data );
  if( gMaxToggleSubscriber ) CNVDispose( gMaxToggleSubscriber );
	if( gMinToggleSubscriber ) CNVDispose( gMinToggleSubscriber );
  if( gWavePublisher ) CNVDispose( gWavePublisher );
	CNVFinish();
  
  DEBUG_EVENT( 0, "RobRehab Network ended on thread %x", CmtGetCurrentThreadID() );
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
  
  for( size_t deviceID = 0; deviceID < 1/*AESControl_GetDevicesNumber()*/; deviceID++ )
  {
    double* controlMeasuresList = AxisControl.GetMeasuresList( deviceID );
    double* controlParametersList = AxisControl.GetSetpointsList( deviceID );

    double* targetList = TrajectoryPlanner_GetTargetList( networkAxesList[ deviceID ].trajectoryPlanner );
    
    //DEBUG_PRINT( "\tnext setpoint: %lf (time: %lf)", targetList[ TRAJECTORY_POSITION ], targetList[ TRAJECTORY_TIME ] );
    
    //AESControl_SetSetpoint( deviceID, targetList[ TRAJECTORY_POSITION ] );
    AxisControl.SetSetpoint( deviceID, targetList[ TRAJECTORY_POSITION ] );

    //Gamb
    static size_t valuesCount;
    static double dataArray[ DISPLAY_VALUES_NUMBER * NUM_POINTS ];
    static size_t arrayDims = DISPLAY_VALUES_NUMBER * NUM_POINTS;

    dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + ROBOT_POSITION ] = controlMeasuresList[ CONTROL_POSITION ];
    dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + ROBOT_VELOCITY ] = controlMeasuresList[ CONTROL_VELOCITY ];
    dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + ROBOT_SETPOINT ] = /*targetList[ TRAJECTORY_POSITION ];*/controlParametersList[ CONTROL_REFERENCE ];
    dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + ROBOT_ERROR ] = controlMeasuresList[ CONTROL_ERROR ]; //Timing_GetExecTimeSeconds();
    dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + ROBOT_STIFFNESS ] = controlParametersList[ CONTROL_STIFFNESS ];
    dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + ROBOT_TORQUE ] = controlMeasuresList[ CONTROL_FORCE ];
    dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + MAX_STIFFNESS ] = maxStiffness;
    dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + PATIENT_TORQUE ] = 0.0;//EMGAESControl_GetTorque( deviceID );
    dataArray[ valuesCount * DISPLAY_VALUES_NUMBER + PATIENT_STIFFNESS ] = 0.0;//EMGAESControl_GetStiffness( deviceID );
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
    }
  }
}


static void UpdateClientInfo( int clientID )
{
  static char messageOut[ IP_CONNECTION_MSG_LEN ];
  
  char* messageIn = AsyncIPConnection_ReadMessage( clientID );
  
  if( messageIn != NULL ) 
  {
    DEBUG_UPDATE( "received input message: %s", messageIn );
  
    for( char* axisCommand = strtok( messageIn, ":" ); axisCommand != NULL; axisCommand = strtok( NULL, ":" ) )
    {
      unsigned int deviceID = (unsigned int) strtoul( axisCommand, &axisCommand, 0 );
    
      if( deviceID < 0 || deviceID >= 1/*AESControl_GetDevicesNumber()*/ ) continue;
    
      DEBUG_UPDATE( "parsing axis %u command \"%s\"", deviceID, axisCommand );
      
      bool motorEnabled = (bool) strtoul( axisCommand, &axisCommand, 0 );

      if( motorEnabled ) /*AESControl_EnableMotor*/AxisControl.Enable( deviceID );
      else /*AESControl_DisableMotor*/AxisControl.Disable( deviceID );

      bool reset = (bool) strtoul( axisCommand, &axisCommand, 0 );

      if( reset ) 
      {
        /*AESControl_Reset*/AxisControl.Reset( deviceID );
        networkAxesList[ deviceID ].dataClientID = -1;
      }
      
      double impedanceStiffness = strtod( axisCommand, &axisCommand );
      double impedanceDamping = strtod( axisCommand, &axisCommand );
      
      /*AESControl_SetImpedance*/AxisControl.SetImpedance( deviceID, impedanceStiffness, impedanceDamping );
      
      /*double positionOffset =*/(void) strtod( axisCommand, NULL );
      
      //AESControl_SetOffset( deviceID, positionOffset );//AxisControl. SEAReadAxes( deviceID );
      
      maxStiffness = impedanceStiffness;
    }
    
    strcpy( messageOut, "" );
    for( size_t deviceID = 0; deviceID < 1/*AESControl_GetDevicesNumber()*/; deviceID++ )
    {
      //bool* motorStatesList = AESControl_GetMotorStatus( deviceID );
      //if( motorStatesList != NULL )
      //{
        if( strlen( messageOut ) > 0 ) strcat( messageOut, ":" );
      
        sprintf( &messageOut[ strlen( messageOut ) ], "%u", deviceID );
      
        strcat( messageOut, ( AxisControl.IsEnabled( deviceID ) == true ) ? " 1" : " 0" );
        //strcat( messageOut, ( AxisControl.HasError( deviceID ) == true ) ? " 1" : " 0" );
        
        //for( size_t stateIndex = 0; stateIndex < AXIS_STATES_NUMBER; stateIndex++ )
        //  strcat( messageOut, ( motorStatesList[ stateIndex ] == true ) ? " 1" : " 0" );
      //}
    }
  
    if( strlen( messageOut ) > 0 ) 
    {
      DEBUG_UPDATE( "sending message %s to client %d", messageOut, clientID );
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
  
    for( char* axisCommand = strtok( messageIn, ":" ); axisCommand != NULL; axisCommand = strtok( NULL, ":" ) )
    {
      unsigned int deviceID = (unsigned int) strtoul( axisCommand, &axisCommand, 0 );
    
      if( deviceID < 0 || deviceID >= 1/*AESControl_GetDevicesNumber()*/ ) continue;
      
      NetworkAxis* networkAxis = &(networkAxesList[ deviceID ]);
      
      if( networkAxis->dataClientID == -1 )
      {
        DEBUG_PRINT( "new client for axis %u: %d", deviceID, clientID );
        networkAxis->dataClientID = clientID;
      }
      else if( networkAxis->dataClientID != clientID ) continue;
      
      DEBUG_UPDATE( "parsing axis %u command \"%s\"", deviceID, axisCommand );
      
      setpoint = strtod( axisCommand, &axisCommand );
      setpointDerivative = strtod( axisCommand, &axisCommand );
      setpointsInterval = strtod( axisCommand, &axisCommand );
      
      //DEBUG_PRINT( "received setpoint: %f", setpoint );
      
      TrajectoryPlanner_SetCurve( networkAxis->trajectoryPlanner, setpoint, setpointDerivative, setpointsInterval );
    }
  }
  
  strcpy( messageOut, "" );
  for( size_t deviceID = 0; deviceID < 1/*AESControl_GetDevicesNumber()*/; deviceID++ )
  {
    double* controlMeasuresList = /*AESControl_GetMeasuresList*/AxisControl.GetMeasuresList( deviceID );
    if( controlMeasuresList != NULL )
    {
      if( strlen( messageOut ) > 0 ) strcat( messageOut, ":" );

      sprintf( &messageOut[ strlen( messageOut ) ], "%u", deviceID );

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

/*static void WriteAxisControlState( unsigned int deviceID, const char* command )
{
  bool motorEnabled = (bool) strtoul( command, &command, 0 );

  if( motorEnabled ) AESControl_EnableMotor( deviceID );
  else AESControl_DisableMotor( deviceID );

  bool reset = (bool) strtoul( command, NULL, 0 );

  if( reset ) AESControl_Reset( deviceID );
}

static char* ReadAxisControlState( unsigned int deviceID )
{
  const size_t STATE_MAX_LEN = 2;
  static char readout[ STATE_MAX_LEN * AXIS_STATES_NUMBER + 1 ];
  
  bool* motorStatesList = AESControl_GetMotorStatus( deviceID );
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

static void WriteAxisControlData( unsigned int deviceID, const char* command )
{
  static double setpointsList[ TRAJECTORY_VALUES_NUMBER ];
  static size_t setpointsCount;
  
  if( clientID != -1 ) return;
  
  networkAxesList[ deviceID ].dataClient = clientID;

  setpointsCount = 0;
  while( *command != '\0' && setpointsCount < TRAJECTORY_VALUES_NUMBER )
  {
    setpointsList[ setpointsCount ] = strtod( command, &command );
    setpointsCount++;
  }

  TrajectoryPlanner_SetCurve( networkAxesList[ deviceID ].trajectoryPlanner, setpointsList );
}

static char* ReadAxisControlData( unsigned int deviceID )
{
  const size_t VALUE_MAX_LEN = 10;
  
  static char readout[ VALUE_MAX_LEN * AXIS_FORCE + 1 ];
  
  double* controlMeasuresList = AESControl_GetMeasuresList( deviceID );
  if( controlMeasuresList != NULL )
  {
    //double* jointMeasuresList = EMGAESControl_ApplyGains( deviceID, maxStiffness );
      
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
  
  unsigned int deviceID = (unsigned int) ( messageCode & 0x000000ff );
  bool enabled = (bool) ( ( messageCode & 0x0000ff00 ) / 0x100 );
  unsigned int muscleGroup = (unsigned int) ( ( messageCode & 0x00ff0000 ) / 0x10000 ); 

	if( handle == gMaxToggleSubscriber )
	{
    EMGAESControl_ChangeState( deviceID, muscleGroup, enabled ? EMG_CONTRACTION_PHASE : EMG_ACTIVATION_PHASE ); 
    
    //DEBUG_PRINT( "axis %u %s %s EMG contraction phase", deviceID, enabled ? "starting" : "ending", ( muscleGroup == MUSCLE_AGONIST ) ? "agonist" : "antagonist" );
	}
	else if( handle == gMinToggleSubscriber )
	{
    EMGAESControl_ChangeState( deviceID, muscleGroup, enabled ? EMG_RELAXATION_PHASE : EMG_ACTIVATION_PHASE );
    
    //DEBUG_PRINT( "axis %u %s %s EMG relaxation phase", deviceID, enabled ? "starting" : "ending", ( muscleGroup == MUSCLE_AGONIST ) ? "agonist" : "antagonist" );
	}*/
}


#endif //ROBREHAB_NETWORK_H
