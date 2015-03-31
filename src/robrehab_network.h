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

static CNVSubscriber gAmplitudeSubscriber, gFrequencySubscriber;
DefineThreadSafeVar( double, Amplitude );
DefineThreadSafeVar( double, Frequency );
void CVICALLBACK AmplitudeFrequencyDataCallback( void*, CNVData, void* );
void CVICALLBACK StopDataCallback( void*, CNVData, void* );

static CNVBufferedWriter gWavePublisher;
static double wave[ NUM_POINTS ];

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
	status = CNVCreateSubscriber( "\\\\localhost\\" PROCESS "\\" AMPLITUDE_VARIABLE, AmplitudeFrequencyDataCallback, 0, 0, 10000, 0, &gAmplitudeSubscriber );
	if( status != 0 ) DEBUG_PRINT( "%s", CNVGetErrorDescription( status ) );
	status = CNVCreateSubscriber( "\\\\localhost\\" PROCESS "\\" FREQUENCY_VARIABLE, AmplitudeFrequencyDataCallback, 0, 0, 10000, 0, &gFrequencySubscriber );
	if( status != 0 ) DEBUG_PRINT( "%s", CNVGetErrorDescription( status ) );
  
  status = CNVCreateBufferedWriter( "\\\\localhost\\" PROCESS "\\" WAVE_VARIABLE, 0, 0, 0, 10, 10000, 0, &gWavePublisher );
	if( status != 0 ) DEBUG_PRINT( "%s", CNVGetErrorDescription( status ) );
  
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
  if( gAmplitudeSubscriber ) CNVDispose( gAmplitudeSubscriber );
	if( gFrequencySubscriber ) CNVDispose( gFrequencySubscriber );
  if( gWavePublisher ) CNVDispose( gWavePublisher );
  
  UninitializeAmplitude();
	UninitializeFrequency();
  
  AxisControl_End();
  
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
  //int dummyClientID = 0;
  //UpdateMotorInfo( 0, &dummyClientID, NULL );
  //UpdateMotorData( 0, &dummyClientID, NULL );
  
  static double motorParametersList[ AXIS_N_PARAMS ];
  
  motorParametersList[ POSITION_SETPOINT ] = GetFrequency();
  motorParametersList[ PROPORTIONAL_GAIN ] = GetAmplitude();
  
  AxisControl_ConfigMotor( 0, motorParametersList );
  
  static size_t valuesCount;
  static size_t arrayDims = NUM_POINTS;
  
  static double motorMeasuresList[ AXIS_N_DIMS ];
  AxisControl_GetMotorMeasures( 0, motorMeasuresList );
  wave[ valuesCount ] = motorMeasuresList[ POSITION ];
  DEBUG_PRINT( "got axis value %u: %.3f", valuesCount, wave[ valuesCount ] );
  valuesCount++;
  
  if( valuesCount >= arrayDims )
  {
    CNVCreateArrayDataValue( &data, CNVDouble, wave, 1, &arrayDims );
	  CNVPutDataInBuffer( gWavePublisher, data, 1000 );
    valuesCount = 0;
  }
}

static int CVICALLBACK UpdateMotorInfo( int index, void* ref_clientID, void* data )
{
  int clientID = *((int*) ref_clientID);
  
  //static char* messageIn;
  const char* messageIn = "";
  
  static bool motorStatesList[ AXIS_N_STATES ];
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
    
    if( reset ) AxisControl_ResetMotor( axisID );
    
    if( AxisControl_GetMotorStatus( axisID, motorStatesList ) != -1 )
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
  const char* messageIn = "";
  
  static char messageOut[ IP_CONNECTION_MSG_LEN ] = "";
  
  static double motorParametersList[ AXIS_N_PARAMS ];
  static double motorMeasuresList[ AXIS_N_DIMS ];
  
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
    
    AxisControl_ConfigMotor( axisID, motorParametersList );
    
    if( AxisControl_GetMotorMeasures( axisID, motorMeasuresList ) != -1 )
    {
      snprintf( messageOut, IP_CONNECTION_MSG_LEN, "%s%u:", messageOut, axisID );
      
      for( size_t dimensionIndex = 0; dimensionIndex < AXIS_N_DIMS; dimensionIndex++ )
        snprintf( messageOut, IP_CONNECTION_MSG_LEN, "%s%.3f ", messageOut, motorMeasuresList[ dimensionIndex ] );
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

void CVICALLBACK AmplitudeFrequencyDataCallback( void* handle, CNVData data, void* callbackData )
{
	double value;

	if( handle == gAmplitudeSubscriber )
	{
		CNVGetScalarDataValue( data, CNVDouble, &value );
		SetAmplitude( value );
	}
	else if( handle == gFrequencySubscriber )
	{
		CNVGetScalarDataValue( data, CNVDouble, &value );
		SetFrequency( value );
	}
}

#endif //ROBREHAB_NETWORK_H
