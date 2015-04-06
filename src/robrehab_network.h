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

const int N_SETPOINTS = 444;
static double setpointsList[ N_SETPOINTS ];

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
  
  DEBUG_EVENT( 1, "Received server connection IDs: %d (Info), %d (Data)", infoServerConnectionID, dataServerConnectionID );
  
  infoClientsList = ListCreate( sizeof(int) );
  dataClientsList = ListCreate( sizeof(int) );
  
  EMGProcessing_Init();
  AxisControl_Init();
  
  FILE* setpointsFile = fopen( "../config/setpoints.dat", "r" );
  
  for( size_t i = 0; i < N_SETPOINTS; i++ )
    fscanf( setpointsFile, "%lf\n", &(setpointsList[ i ]) );
            
  fclose( setpointsFile );
  
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
  EMGProcessing_End();
  
  DEBUG_EVENT( 0, "RobRehab Network ended on thread %x", CmtGetCurrentThreadID() );
}

static int CVICALLBACK UpdateMotorInfo( int, void*, void* );
static int CVICALLBACK UpdateMotorData( int, void*, void* );

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
  
  //ListApplyToEach( infoClientsList, 1, UpdateMotorInfo, (void*) motorInfoClientsList );
  
  //ListApplyToEach( dataClientsList, 1, UpdateMotorData, (void*) motorDataClientsList );
  
  // Test
  int dummyClientID = 0;
  UpdateMotorInfo( 0, &dummyClientID, NULL );
  UpdateMotorData( 0, &dummyClientID, NULL );
}

static int CVICALLBACK UpdateMotorInfo( int index, void* ref_clientID, void* data )
{
  int clientID = *((int*) ref_clientID);
  
  //static char* messageIn;
  const char* messageIn = "";
  
  static bool* motorStatesList;
  static char messageOut[ IP_CONNECTION_MSG_LEN ] = "";
  
  //if( (messageIn = AsyncIPConnection_ReadMessage( clientID )) == NULL ) return 0;
  
  DEBUG_UPDATE( "received input message: %s", messageIn );
  
  for( char* command = strtok( messageIn, ":" ); command != NULL; command = strtok( NULL, ":" ) )
  {
    unsigned int axisID = (unsigned int) strtoul( command, NULL, 0 );
    
    if( axisID < 0 || axisID >= AxisControl_GetActiveAxesNumber() ) continue;
    else if( networkAxesList[ axisID ].infoClient != 0 ) continue;
    else networkAxesList[ axisID ].infoClient = clientID;
    
    bool motorEnabled = (bool) strtoul( command, &command, 0 );
    
    if( motorEnabled ) AxisControl_EnableMotor( axisID );
    else AxisControl_DisableMotor( axisID );
    
    bool reset = (bool) strtoul( command, NULL, 0 );
    
    if( reset ) AxisControl_Reset( axisID );
    
    if( (motorStatesList = AxisControl_GetMotorStatus( axisID )) != NULL )
    {
      snprintf( messageOut, IP_CONNECTION_MSG_LEN, "%s%u:", messageOut, axisID );
      
      for( size_t stateIndex = 0; stateIndex < AXIS_N_STATES; stateIndex++ )
        snprintf( messageOut, IP_CONNECTION_MSG_LEN, "%s%c ", messageOut, ( motorStatesList[ stateIndex ] == true ) ? '1' : '0' );
    }
  }
  
  if( strlen( messageOut ) > 0 )
  {
    messageOut[ strlen( messageOut ) - 1 ] = '\0';
    DEBUG_UPDATE( "generated output message: %s", messageOut );
    //AsyncIPConnection_WriteMessage( clientID, messageOut );
  }
  
  return 0;
}

static int CVICALLBACK UpdateMotorData( int index, void* ref_clientID, void* data )
{
  int clientID = *((int*) ref_clientID);
  
  //static char* messageIn;
  static char messageIn[ IP_CONNECTION_MSG_LEN ];
  sprintf( messageIn, "0:0.0 0.0 %.3f %.3f", axisStiffness, axisDamping );
  
  static char messageOut[ IP_CONNECTION_MSG_LEN ] = "";
  
  static double* motorMeasuresList;
  static double motorParametersList[ AXIS_N_PARAMS ];
  
  //if( (messageIn = AsyncIPConnection_ReadMessage( clientID )) == NULL ) return 0;
  
  DEBUG_UPDATE( "received input message: %s", messageIn );
  
  for( char* command = strtok( messageIn, ":" ); command != NULL; command = strtok( NULL, ":" ) )
  {
    unsigned int axisID = (unsigned int) strtoul( command, NULL, 0 );
    
    if( axisID < 0 || axisID >= AxisControl_GetActiveAxesNumber() ) continue;
    else if( networkAxesList[ axisID ].dataClient != 0 ) continue;
    else networkAxesList[ axisID ].dataClient = clientID;
    
    for( size_t parameterIndex = 0; parameterIndex < AXIS_N_PARAMS; parameterIndex++ )
      motorParametersList[ parameterIndex ] = strtod( command, &command );
    
    static size_t setpointIndex;
    motorParametersList[ POSITION_SETPOINT ] = setpointsList[ setpointIndex % N_SETPOINTS ] / ( 2 * PI );
    setpointIndex++;
    
    AxisControl_ConfigMotor( axisID, motorParametersList );
    
    if( (motorMeasuresList = AxisControl_GetSensorMeasures( axisID )) != NULL )
    {
      snprintf( messageOut, IP_CONNECTION_MSG_LEN, "%s%u:", messageOut, axisID );
      
      for( size_t dimensionIndex = 0; dimensionIndex < AXIS_N_DIMS; dimensionIndex++ )
        snprintf( messageOut, IP_CONNECTION_MSG_LEN, "%s%.3f ", messageOut, motorMeasuresList[ dimensionIndex ] );
      
      static size_t valuesCount;
      static double dimensionValuesArray[ AXIS_N_DIMS * NUM_POINTS ];
      static size_t arrayDims = AXIS_N_DIMS * NUM_POINTS;
  
      for( size_t dimensionIndex = 0; dimensionIndex < AXIS_N_DIMS; dimensionIndex++ )
        dimensionValuesArray[ valuesCount * AXIS_N_DIMS + dimensionIndex ] = motorMeasuresList[ dimensionIndex ];
      
      // Gamb
      dimensionValuesArray[ valuesCount * AXIS_N_DIMS + CURRENT ] = motorParametersList[ POSITION_SETPOINT ];
      double* emgValuesList = EMGProcessing_GetValues();
      if( emgValuesList != NULL ) dimensionValuesArray[ valuesCount * AXIS_N_DIMS + TENSION ] = emgValuesList[ 0 ];
      //DEBUG_PRINT( "got EMG value %g", dimensionValuesArray[ valuesCount * AXIS_N_DIMS + TENSION ] );
      
      valuesCount++;
      
      if( valuesCount >= NUM_POINTS )
      {
        CNVCreateArrayDataValue( (CNVData*) &data, CNVDouble, dimensionValuesArray, 1, &arrayDims );
    	  CNVPutDataInBuffer( gWavePublisher, data, 1000 );
        valuesCount = 0;
      }
    }
  }
  
  if( strlen( messageOut ) > 0 )
  {
    messageOut[ strlen( messageOut ) - 1 ] = '\0';
    DEBUG_UPDATE( "generated output message: %s", messageOut );
    //AsyncIPConnection_WriteMessage( clientID, messageOut );
  }
  
  return 0;
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
