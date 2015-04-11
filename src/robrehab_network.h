#ifndef ROBREHAB_NETWORK_H
#define ROBREHAB_NETWORK_H

#include <cvinetv.h>
#include <utility.h>
#include "Common.h"

#ifdef _CVI_
  #include "cvirte_ip_connection.h"
#else
  #include "async_ip_connection.h"
#endif

#include "axis_control.h"
#include "emg_processing.h"

#include "async_debug.h"

static int infoServerConnectionID;
static int dataServerConnectionID;

static ListType infoClientsList;
static ListType dataClientsList;

typedef struct _NetworkAxis
{
  unsigned int axisID;
  int infoClient;
  int dataClient;
} 
NetworkAxis;

static NetworkAxis* networkAxesList;

static char axesInfoString[ IP_CONNECTION_MSG_LEN ] = ""; // String containing used axes names

static CNVData data = 0;

static double axisStiffness = 10.0, axisDamping = 0.0;
static CNVSubscriber gStiffnessSubscriber, gDampingSubscriber;
void CVICALLBACK GainDataCallback( void*, CNVData, void* );

static CNVSubscriber gEnableSubscriber;
void CVICALLBACK EnableDataCallback( void*, CNVData, void* );

static CNVBufferedWriter gWavePublisher;

const int FILE_SETPOINTS_NUMBER = 444;
static double fileSetpointsList[ FILE_SETPOINTS_NUMBER ];

int RobRehabNetwork_Init()
{
  DEBUG_EVENT( 0, "Initializing RobRehab Network on thread %x", CmtGetCurrentThreadID() );
  if( (infoServerConnectionID = AsyncIPConnection_Open( NULL, "50000", TCP )) == -1 )
    return -1;
  if( (dataServerConnectionID = AsyncIPConnection_Open( NULL, "50001", UDP )) == -1 )
  {
    AsyncIPConnection_Close( infoServerConnectionID );
    return -1;
  }
  
  DEBUG_EVENT( 1, "Received server connection IDs: %d (State), %d (Data)", infoServerConnectionID, dataServerConnectionID );
  
  infoClientsList = ListCreate( sizeof(int) );
  dataClientsList = ListCreate( sizeof(int) );
  
  
  FILE* setpointsFile = fopen( "../config/setpoints.dat", "r" );
  
  for( size_t i = 0; i < FILE_SETPOINTS_NUMBER; i++ )
    fscanf( setpointsFile, "%lf\n", &(fileSetpointsList[ i ]) );
            
  fclose( setpointsFile );
  
  
  //EMGProcessing_Init();
  AxisControl_Init();
  
  networkAxesList = (NetworkAxis*) calloc( AxisControl_GetActiveAxesNumber(), sizeof(NetworkAxis) );
  
  for( size_t axisID = 0; axisID < AxisControl_GetActiveAxesNumber(); axisID++ )
  {
    if( AxisControl_GetAxisName( axisID ) != NULL )
      snprintf( axesInfoString, IP_CONNECTION_MSG_LEN, "%s%u:%s:", axesInfoString, axisID, AxisControl_GetAxisName( axisID ) );
    
    networkAxesList[ axisID ].infoClient = networkAxesList[ axisID ].dataClient = 0;
  }
  
  if( strlen( axesInfoString ) > 0 )
    axesInfoString[ strlen( axesInfoString ) - 1 ] = '\0';
  
  int status = 0;
  
  // Create network variable connections.
	status = CNVCreateSubscriber( "\\\\localhost\\" PROCESS "\\" STIFFNESS_VARIABLE, GainDataCallback, 0, 0, 10000, 0, &gStiffnessSubscriber );
	if( status != 0 ) DEBUG_PRINT( "%s", CNVGetErrorDescription( status ) );
	status = CNVCreateSubscriber( "\\\\localhost\\" PROCESS "\\" DAMPING_VARIABLE, GainDataCallback, 0, 0, 10000, 0, &gDampingSubscriber );
	if( status != 0 ) DEBUG_PRINT( "%s", CNVGetErrorDescription( status ) );
  
  status = CNVCreateBufferedWriter( "\\\\localhost\\" PROCESS "\\" WAVE_VARIABLE, 0, 0, 0, 10, 10000, 0, &gWavePublisher );
	if( status != 0 ) DEBUG_PRINT( "%s", CNVGetErrorDescription( status ) );
  
  status = CNVCreateSubscriber( "\\\\localhost\\" PROCESS "\\" ENABLE_VARIABLE, EnableDataCallback, 0, 0, 10000, 0, &gEnableSubscriber );
	if( status != 0 ) printf( "%s\n\n", CNVGetErrorDescription( status ) );
  
  DEBUG_EVENT( 0, "RobRehab Network initialized on thread %x", CmtGetCurrentThreadID() );
  
  return 0;
}

void RobRehabNetwork_End()
{
  DEBUG_EVENT( 0, "Ending RobRehab Network on thread %x", CmtGetCurrentThreadID() );
  
  AsyncIPConnection_Close( infoServerConnectionID );
  AsyncIPConnection_Close( dataServerConnectionID );
  
  ListDispose( infoClientsList );
  ListDispose( dataClientsList );
  
  if( data ) CNVDisposeData( data );
  if( gStiffnessSubscriber ) CNVDispose( gStiffnessSubscriber );
	if( gDampingSubscriber ) CNVDispose( gDampingSubscriber );
  if( gWavePublisher ) CNVDispose( gWavePublisher );
  if( gEnableSubscriber ) CNVDispose( gEnableSubscriber );
	CNVFinish();
  
  AxisControl_End();
  //EMGProcessing_End();
  
  DEBUG_EVENT( 0, "RobRehab Network ended on thread %x", CmtGetCurrentThreadID() );
}

static int CVICALLBACK UpdateMotorState( int, void*, void* );
static int CVICALLBACK UpdateMotorData( int, void*, void* );

static int CVICALLBACK UpdateAxisControl( int, void*, void* );

typedef char* (*ProcessAxisCommandFunction)( int, unsigned int, const char* );
static char* ProcessAxisControlState( int, unsigned int, const char* );
static char* ProcessAxisControlData( int, unsigned int, const char* );


void RobRehabNetwork_Update()
{
  static int newInfoClient;
  static int newDataClient;
  
  if( (newInfoClient = AsyncIPConnection_GetClient( infoServerConnectionID )) != -1 )
  {
    AsyncIPConnection_WriteMessage( newInfoClient, axesInfoString );
    ListInsertItem( infoClientsList, &newInfoClient, END_OF_LIST );
  }
  
  if( (newDataClient = AsyncIPConnection_GetClient( dataServerConnectionID )) != -1 )
    ListInsertItem( dataClientsList, &newDataClient, END_OF_LIST );
  
  for( size_t axisID = 0; axisID < AxisControl_GetActiveAxesNumber(); axisID++ )
    networkAxesList[ axisID ].infoClient = networkAxesList[ axisID ].dataClient = 0;
  
  ProcessAxisCommandFunction ref_ProcessAxisCommand;
  //ref_ProcessAxisCommand = ProcessAxisControlState;
  //ListApplyToEach( infoClientsList, 1, UpdateAxisControl, &ref_ProcessAxisCommand );
  //ref_ProcessAxisCommand = ProcessAxisControlData;
  //ListApplyToEach( dataClientsList, 1, UpdateAxisControl, &ref_ProcessAxisCommand );
  
  // Test
  int dummyClientID = 0;
  ref_ProcessAxisCommand = ProcessAxisControlState;
  //UpdateAxisControl( 0, &dummyClientID, (void*) &ref_ProcessAxisCommand );
  ref_ProcessAxisCommand = ProcessAxisControlData;
  UpdateAxisControl( 0, &dummyClientID, (void*) &ref_ProcessAxisCommand );
}


static int CVICALLBACK UpdateAxisControl( int index, void* ref_clientID, void* ref_callback )
{
  int clientID = *((int*) ref_clientID);
  ProcessAxisCommandFunction ref_ProcessAxisCommand = *((ProcessAxisCommandFunction*) ref_callback);
  
  //static char* messageIn;
  const char* messageIn = "0 0.0";
  static char messageOut[ IP_CONNECTION_MSG_LEN ];
  
  //if( (messageIn = AsyncIPConnection_ReadMessage( clientID )) == NULL ) return 0;
  
  DEBUG_UPDATE( "received input message: %s", messageIn );
  
  strcpy( messageOut, "" );
  for( char* axisCommand = strtok( messageIn, ":" ); axisCommand != NULL; axisCommand = strtok( NULL, ":" ) )
  {
    unsigned int axisID = (unsigned int) strtoul( axisCommand, &axisCommand, 0 );
    
    if( axisID < 0 || axisID >= AxisControl_GetActiveAxesNumber() ) continue;
    
    DEBUG_UPDATE( "parsing axis %u command \"%s\"", axisID, axisCommand );
    char* readout = ref_ProcessAxisCommand( clientID, axisID, axisCommand );
    if( strcmp( readout, "" ) != 0 )
    {
      if( strlen( messageOut ) > 0 ) strcat( messageOut, ":" );
      
      snprintf( messageOut, IP_CONNECTION_MSG_LEN, "%s%u %s", messageOut, axisID, readout );
    }
  }
  
  DEBUG_UPDATE( "generated output message: %s", messageOut );
  //AsyncIPConnection_WriteMessage( clientID, messageOut );
  
  return 0;
}

static char* ProcessAxisControlState( int clientID, unsigned int axisID, const char* command )
{
  const size_t STATE_MAX_LEN = 2;
  static char readout[ STATE_MAX_LEN * AXIS_N_STATES + 1 ];
  
  if( networkAxesList[ axisID ].infoClient == 0 )
  {
    networkAxesList[ axisID ].infoClient = clientID;

    bool motorEnabled = (bool) strtoul( command, &command, 0 );

    if( motorEnabled ) AxisControl_EnableMotor( axisID );
    else AxisControl_DisableMotor( axisID );

    bool reset = (bool) strtoul( command, NULL, 0 );

    if( reset ) AxisControl_Reset( axisID );

    bool* motorStatesList = AxisControl_GetMotorStatus( axisID );
    if( motorStatesList != NULL )
    {
      strcpy( readout, "" );
      for( size_t stateIndex = 0; stateIndex < AXIS_N_STATES; stateIndex++ )
        strcat( readout, ( motorStatesList[ stateIndex ] == true ) ? "1 " : "0 " );
      readout[ strlen( readout ) - 1 ] = '\0';
    }
  }
  
  return readout;
}

static char* ProcessAxisControlData( int clientID, unsigned int axisID, const char* command )
{
  const size_t VALUE_MAX_LEN = 10;
  const size_t SETPOINTS_MAX_NUMBER = IP_CONNECTION_MSG_LEN / VALUE_MAX_LEN;
  const size_t AXIS_VALUES_NUMBER = AXIS_DIMS_NUMBER + AXIS_PARAMS_NUMBER;
  
  static char readout[ VALUE_MAX_LEN * AXIS_VALUES_NUMBER + 1 ];
  
  static double motorParametersList[ AXIS_PARAMS_NUMBER ];
  
  static double setpointsList[ SETPOINTS_MAX_NUMBER ];
  static double setpointValue;
  static size_t setpointsCount = 0;
  
  if( networkAxesList[ axisID ].dataClient == 0 )
  {
    networkAxesList[ axisID ].dataClient = clientID;
    
    static size_t setpointIndex; // Gamb
    while( *command != '\0' )
    {
      setpointValue = strtod( command, &command );
      //setpointsList[ setpointsCount++ ] = setpointValue;
      setpointsList[ setpointsCount++ ] = fileSetpointsList[ setpointIndex % FILE_SETPOINTS_NUMBER ] / ( 2 * PI ); // Gamb
      setpointIndex++; // Gamb
    }
    
    AxisControl_LoadSetpoints( axisID, setpointsList, setpointsCount );

    motorParametersList[ PROPORTIONAL_GAIN ] = axisStiffness;
    motorParametersList[ DERIVATIVE_GAIN ] = axisDamping;
    
    AxisControl_ConfigMotor( axisID, motorParametersList );
    
    double* motorMeasuresList = AxisControl_GetSensorMeasures( axisID );
    if( motorMeasuresList != NULL )
    {
      strcpy( readout, "" );
      for( size_t dimensionIndex = 0; dimensionIndex < AXIS_DIMS_NUMBER; dimensionIndex++ )
        sprintf( &readout[ strlen( readout ) ], "%.3f ", motorMeasuresList[ dimensionIndex ] );
      for( size_t parameterIndex = 0; parameterIndex < AXIS_PARAMS_NUMBER; parameterIndex++ )
        sprintf( &readout[ strlen( readout ) ], "%.3f ", motorParametersList[ parameterIndex ] );
      readout[ strlen( readout ) - 1 ] = '\0';
  
      //Gamb 
      static size_t valuesCount = 0;
      static double dataArray[ AXIS_VALUES_NUMBER * NUM_POINTS ];
      static size_t arrayDims = AXIS_VALUES_NUMBER * NUM_POINTS;

      for( size_t dimensionIndex = 0; dimensionIndex < AXIS_DIMS_NUMBER; dimensionIndex++ )
        dataArray[ valuesCount * AXIS_VALUES_NUMBER + dimensionIndex ] = motorMeasuresList[ dimensionIndex ];
      for( size_t parameterIndex = 0; parameterIndex < AXIS_PARAMS_NUMBER; parameterIndex++ )
        dataArray[ valuesCount * AXIS_VALUES_NUMBER + ( AXIS_DIMS_NUMBER + parameterIndex ) ] = motorParametersList[ parameterIndex ];
      
      size_t dataIndex = valuesCount * AXIS_VALUES_NUMBER + ( AXIS_DIMS_NUMBER + POSITION_SETPOINT );
      DEBUG_PRINT( "position setpoint index: %u * %u + %u + %u = %u", valuesCount, AXIS_VALUES_NUMBER, AXIS_DIMS_NUMBER, POSITION_SETPOINT, dataIndex );
      //DEBUG_PRINT( "got position setpoint %d value %g", valuesCount * AXIS_VALUES_NUMBER + ( AXIS_DIMS_NUMBER + POSITION_SETPOINT ), dataArray[ valuesCount * AXIS_VALUES_NUMBER + ( AXIS_DIMS_NUMBER + POSITION_SETPOINT ) ] );
      
      //double* emgValuesList = EMGProcessing_GetValues();
      //if( emgValuesList != NULL ) dataArray[ TENSION * NUM_POINTS + valuesCount ] = emgValuesList[ 0 ];
      //DEBUG_PRINT( "got EMG value %g", dataArray[ TENSION * NUM_POINTS + valuesCount ] );
      
      valuesCount++;
      
      if( valuesCount >= NUM_POINTS )
      {
        CNVCreateArrayDataValue( (CNVData*) &data, CNVDouble, dataArray, 1, &arrayDims );
    	  CNVPutDataInBuffer( gWavePublisher, data, 1000 );
        valuesCount = 0;
      }
    }
  }
  
  DEBUG_PRINT( "measure readout: %s", readout );
  
  return readout;
}


void CVICALLBACK GainDataCallback( void* handle, CNVData data, void* callbackData )
{
	double value;

	if( handle == gStiffnessSubscriber )
	{
		CNVGetScalarDataValue( data, CNVDouble, &value );
		axisStiffness = value;
    DEBUG_PRINT( "got stiffness value %g", axisStiffness );
	}
	else if( handle == gDampingSubscriber )
	{
		CNVGetScalarDataValue( data, CNVDouble, &value );
		axisDamping = value;
    DEBUG_PRINT( "got damping value %g", axisDamping );
	}
}

void CVICALLBACK EnableDataCallback( void * handle, CNVData data, void * callbackData )
{
	char enabled;

	CNVGetScalarDataValue( data, CNVBool, &enabled );
	if( enabled )
  {
    AxisControl_Reset( 0 );
		AxisControl_EnableMotor( 0 );
  }
  else
    AxisControl_DisableMotor( 0 );
  
  DEBUG_PRINT( "motor 0 %s", enabled ? "enabled" : "disabled" ); 
}

#endif //ROBREHAB_NETWORK_H
