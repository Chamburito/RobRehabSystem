#ifndef ROBREHAB_NETWORK_H
#define ROBREHAB_NETWORK_H

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

static char axesInfoString[ IP_CONNECTION_MSG_LEN ] = ""; // String containing used axes names

/*typedef struct _Network_Axis
{
  
}
Network_Axis;*/

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
  
  for( size_t motorID = 0; motorID < CONTROL_N_JOINTS; motorID++ )
  {
    if( AxisControl_GetMotorName( motorID ) != NULL )
      snprintf( axesInfoString, IP_CONNECTION_MSG_LEN, "%s%u:%s:", axesInfoString, motorID, AxisControl_GetMotorName( motorID ) );
  }
  
  if( strlen( axesInfoString ) > 0 )
    axesInfoString[ strlen( axesInfoString ) - 1 ] = '\0';
  
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
  
  size_t motorInfoClientsList[ CONTROL_N_JOINTS ] = { 0 };
  //ListApplyToEach( infoClientsList, 1, UpdateMotorInfo, (void*) motorInfoClientsList );
  
  size_t motorDataClientsList[ CONTROL_N_JOINTS ] = { 0 };
  //ListApplyToEach( dataClientsList, 1, UpdateMotorData, (void*) motorDataClientsList );
  
  // Test
  int dummyClientID = 0;
  UpdateMotorInfo( 0, &dummyClientID, (void*) motorInfoClientsList );
  UpdateMotorData( 0, &dummyClientID, (void*) motorDataClientsList ); 
}

static int CVICALLBACK UpdateMotorInfo( int index, void* ref_clientID, void* motorClientsData )
{
  int clientID = *((int*) ref_clientID);
  size_t* motorClientsList = (size_t*) motorClientsData;
  
  //static char* messageIn;
  const char* messageIn = "";
  
  static bool* motorStatesList;
  static char messageOut[ IP_CONNECTION_MSG_LEN ] = "";
  
  //if( (messageIn = AsyncIPConnection_ReadMessage( clientID )) == NULL ) return 0;
  
  DEBUG_UPDATE( "received input message: %s", messageIn );
  
  for( char* command = strtok( messageIn, ":" ); command != NULL; command = strtok( NULL, ":" ) )
  {
    unsigned int motorID = (unsigned int) strtoul( command, NULL, 0 );
    
    if( motorClientsList[ motorID ] != 0 ) continue;
    else motorClientsList[ motorID ] = clientID;
    
    bool motorEnabled = (bool) strtoul( command, &command, 0 );
    
    if( motorEnabled ) AxisControl_EnableMotor( motorID );
    else AxisControl_DisableMotor( motorID );
    
    bool reset = (bool) strtoul( command, NULL, 0 );
    
    if( reset ) AxisControl_ResetMotor( motorID );
    
    if( (motorStatesList = AxisControl_GetMotorStatus( motorID )) != NULL )
    {
      snprintf( messageOut, IP_CONNECTION_MSG_LEN, "%s%u:", messageOut, motorID );
      
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

static int CVICALLBACK UpdateMotorData( int index, void* ref_clientID, void* motorClientsData )
{
  int clientID = *((int*) ref_clientID);
  size_t* motorClientsList = (size_t*) motorClientsData;
  
  //static char* messageIn;
  const char* messageIn = "";
  
  static char messageOut[ IP_CONNECTION_MSG_LEN ] = "";
  
  static double motorParametersList[ AXIS_N_PARAMS ];
  
  static double* motorMeasuresList;
  
  //if( (messageIn = AsyncIPConnection_ReadMessage( clientID )) == NULL ) return 0;
  
  DEBUG_UPDATE( "received input message: %s", messageIn );
  
  for( char* command = strtok( messageIn, ":" ); command != NULL; command = strtok( NULL, ":" ) )
  {
    unsigned int motorID = (unsigned int) strtoul( command, NULL, 0 );
    
    if( motorClientsList[ motorID ] != 0 ) continue;
    else motorClientsList[ motorID ] = clientID;
    
    for( size_t parameterIndex = 0; parameterIndex < AXIS_N_PARAMS; parameterIndex++ )
      motorParametersList[ parameterIndex ] = strtod( command, &command );
    
    AxisControl_ConfigMotor( motorID, (double*) motorParametersList );
    
    if( (motorMeasuresList = AxisControl_GetMotorMeasures( motorID )) != NULL )
    {
      snprintf( messageOut, IP_CONNECTION_MSG_LEN, "%s%u:", messageOut, motorID );
      
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

#endif //ROBREHAB_NETWORK_H
