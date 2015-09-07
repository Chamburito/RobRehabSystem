#include "shm_axis_control.h"
#include "axis_control.h"

#include "klib/kvec.h"
#include "klib/khash.h"

#include "utils/file_parsing/json_parser.h"

#include "async_debug.h"

typedef struct _SHMAxisControl
{
  int shmAxisControlDataID;
  int axisControllerID;
}
SHMAxisControl;

KHASH_MAP_INIT_INT( SHMAxisControl, SHMAxisControl )
static khash_t( SHMAxisControl )* shmAxisControlsList = NULL;

int RobRehabControl_Init()
{
  int shmAxisControlDataID, axisControllerID;
  bool loadError = false;
  
  shmAxisControlsList = kh_init( SHMAxisControl );
  
  FileParser parser = JSONParser;
  int configFileID = parser.LoadFile( "../config/shared_axes" );
  if( configFileID != -1 )
  {
    if( parser.HasKey( configFileID, "axes" ) )
    {
      size_t shmAxesNumber = parser.GetListSize( configFileID, "axes" );
      
      DEBUG_PRINT( "List size: %u", shmAxesNumber );
      
      char searchPath[ FILE_PARSER_MAX_PATH_LENGTH ];
      for( size_t shmAxisDataIndex = 0; shmAxisDataIndex < shmAxesNumber; shmAxisDataIndex++ )
      {
        sprintf( searchPath, "axes.%u.name", shmAxisDataIndex );
        char* deviceName = parser.GetStringValue( configFileID, searchPath );
        
        if( (axisControllerID = AxisControl.Init( deviceName )) != -1 )
        {
          sprintf( searchPath, "axes.%u.key", shmAxisDataIndex );
          int shmKey = (int) parser.GetIntegerValue( configFileID, searchPath );
          
          if( (shmAxisControlDataID = ShmAxisControl_Init( shmKey )) != -1 )
          {
            int insertionStatus;
            khint_t shmAxisDataID = kh_put( SHMAxisControlControl, shmAxisControlsList, shmAxisControlDataID, &insertionStatus );
            if( insertionStatus != -1 )
            {
              kh_value( shmAxisControlsList, shmAxisDataID ).shmAxisControlDataID = shmAxisControlDataID;
              kh_value( shmAxisControlsList, shmAxisDataID ).axisControllerID = axisControllerID;
            }
            else loadError = true;
          }
          else loadError = true;
        }
        else loadError = true;
        
      }
    }
    else loadError = true;
    
    parser.UnloadFile( configFileID );
  }
  else loadError = true;
  
  if( loadError )
  {
    ShmAxisControl_End( shmAxisControlDataID );
    AxisControl.End( axisControllerID );
    return -1;
  }
  
  //DEBUG_PRINT( "Created axes info string: %s", axesInfoString );
  
  ///*DEBUG_EVENT( 0,*/DEBUG_PRINT( "RobRehab Network initialized on thread %x", THREAD_ID );
  
  return 0;
}

void RobRehabNetwork_End()
{
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "Ending RobRehab Network on thread %x", THREAD_ID );
  
  AsyncIPConnection_Close( infoServerConnectionID );
  AsyncIPConnection_Close( dataServerConnectionID );
  
  kv_destroy( infoClientsList );
  kv_destroy( dataClientsList );
  
  for( khint_t shmSHMAxisControlControlDataID = (khint_t) 0; khint_t shmSHMAxisControlControlDataID != kh_end( shmAxisControlsList ); shmSHMAxisControlControlDataID++ )
  {
    if( !kh_exist( shmAxisControlsList, shmSHMAxisControlControlDataID ) ) continue;
      
    ShmAxisControl_End( kh_key( shmAxisControlsList, shmSHMAxisControlControlDataID ) );
    TrajectoryPlanner_End( kh_value( shmAxisControlsList, shmSHMAxisControlControlDataID ).trajectoryPlanner );
  }
  kh_destroy( SHMAxisControlControl, shmAxisControlsList );
  
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "RobRehab Network ended on thread %x", THREAD_ID );
}

static int UpdateControlState( int );
static int UpdateControlData( int );

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
  
  for( size_t infoClientIndex = 0; infoClientIndex < kv_size( infoClientsList ); infoClientIndex++ )
    UpdateControlState( kv_A( infoClientsList, infoClientIndex ) );
  
  for( size_t dataClientIndex = 0; dataClientIndex < kv_size( dataClientsList ); dataClientIndex++ )
    UpdateControlData( kv_A( dataClientsList, dataClientIndex ) );
  
  for( khint_t shmSHMAxisControlControlDataID = (khint_t) 0; shmSHMAxisControlControlDataID != kh_end( shmAxisControlsList ); shmSHMAxisControlControlDataID++ )
  {
    if( !kh_exist( shmAxisControlsList, shmSHMAxisControlControlDataID ) ) continue;
    
    DEBUG_PRINT( "updating axis %u", shmSHMAxisControlControlDataID );
    
    double* controlParametersList = ShmAxisControl_GetParametersList( kh_key( shmAxisControlsList, shmSHMAxisControlControlDataID ) );
    double* targetsList = TrajectoryPlanner_GetTargetList( kh_value( shmAxisControlsList, shmSHMAxisControlControlDataID ).trajectoryPlanner );
    controlParametersList[ AXIS_FORCE ] = targetsList[ TRAJECTORY_POSITION ];
    ShmAxisControl_SetParameters( kh_key( shmAxisControlsList, shmSHMAxisControlControlDataID ) );
  }
}

static int UpdateControlState( int clientID )
{
  static char messageOut[ IP_CONNECTION_MSG_LEN ];
  
  char* messageIn = AsyncIPConnection_ReadMessage( clientID );
  
  if( messageIn != NULL ) 
  {
    DEBUG_UPDATE( "received input message: %s", messageIn );
  
    for( char* axisCommand = strtok( messageIn, ":" ); axisCommand != NULL; axisCommand = strtok( NULL, ":" ) )
    {
      khint_t shmSHMAxisControlControlDataID = (khint_t) strtoul( axisCommand, &axisCommand, 0 );
    
      if( !kh_exist( shmAxisControlsList, shmSHMAxisControlControlDataID ) ) continue;
    
      DEBUG_UPDATE( "parsing axis %u command \"%s\"", shmSHMAxisControlControlDataID, axisCommand );

      bool* statesList = ShmAxisControl_GetStatesList( kh_key( shmAxisControlsList, shmSHMAxisControlControlDataID ) );
      
      statesList[ CONTROL_ENABLED ] = (bool) strtoul( axisCommand, &axisCommand, 0 );
      statesList[ CONTROL_RESET ] = (bool) strtoul( axisCommand, &axisCommand, 0 );

      if( statesList[ CONTROL_RESET ] ) kh_value( shmAxisControlsList, shmSHMAxisControlControlDataID ).dataClientID = -1;
      
      ShmAxisControl_SetStates( kh_key( shmAxisControlsList, shmSHMAxisControlControlDataID ) );
      
      double* parametersList = ShmAxisControl_GetParametersList( kh_key( shmAxisControlsList, shmSHMAxisControlControlDataID ) );
      
      parametersList[ CONTROL_STIFFNESS ] = strtod( axisCommand, &axisCommand );
      parametersList[ CONTROL_DAMPING ] = strtod( axisCommand, &axisCommand );
      parametersList[ CONTROL_OFFSET ] = strtod( axisCommand, NULL );
      
      ShmAxisControl_SetParameters( kh_key( shmAxisControlsList, shmSHMAxisControlControlDataID ) );
    }
    
    strcpy( messageOut, "" );
    for( khint_t shmSHMAxisControlControlDataID = (khint_t) 0; shmSHMAxisControlControlDataID != kh_end( shmAxisControlsList ); shmSHMAxisControlControlDataID++ )
    {
      if( !kh_exist( shmAxisControlsList, shmSHMAxisControlControlDataID ) ) continue;
      
      bool* statesList = ShmAxisControl_GetStatesList( kh_key( shmAxisControlsList, shmSHMAxisControlControlDataID ) );
      
      if( strlen( messageOut ) > 0 ) strcat( messageOut, ":" );
      sprintf( &messageOut[ strlen( messageOut ) ], "%u", shmSHMAxisControlControlDataID );
      for( size_t stateIndex = 0; stateIndex < CONTROL_STATES_NUMBER; stateIndex++ )
        strcat( messageOut, ( statesList[ stateIndex ] == true ) ? " 1" : " 0" );
    }
  
    if( strlen( messageOut ) > 0 ) 
    {
      DEBUG_UPDATE( "sending message %s to client %d", messageOut, clientID );
      AsyncIPConnection_WriteMessage( clientID, messageOut );
    }
  }
  
  return 0;
}

static int UpdateControlData( int clientID )
{
  static char messageOut[ IP_CONNECTION_MSG_LEN ];
  
  static double setpoint, setpointDerivative, setpointsInterval;
  
  char* messageIn = AsyncIPConnection_ReadMessage( clientID );
  
  if( messageIn != NULL ) 
  {
    DEBUG_UPDATE( "received input message: %s", messageIn );
  
    for( char* axisCommand = strtok( messageIn, ":" ); axisCommand != NULL; axisCommand = strtok( NULL, ":" ) )
    {
      khint_t shmSHMAxisControlControlDataID = (khint_t) strtoul( axisCommand, &axisCommand, 0 );
    
      if( !kh_exist( shmAxisControlsList, shmSHMAxisControlID ) ) continue;
      
      SHMAxisControl* networkAxis = &(kh_value( shmAxisControlsList, shmSHMAxisControlID ));
      
      if( networkAxis->dataClientID == -1 )
      {
        DEBUG_PRINT( "new client for axis %u: %d", shmSHMAxisControlID, clientID );
        networkAxis->dataClientID = clientID;
      }
      else if( networkAxis->dataClientID != clientID ) continue;
      
      DEBUG_UPDATE( "parsing axis %u command \"%s\"", shmSHMAxisControlID, axisCommand );
      
      setpoint = strtod( axisCommand, &axisCommand );
      setpointDerivative = strtod( axisCommand, &axisCommand );
      setpointsInterval = strtod( axisCommand, &axisCommand );
      
      //DEBUG_PRINT( "received setpoint: %f", setpoint );
      
      TrajectoryPlanner_SetCurve( networkAxis->trajectoryPlanner, setpoint, setpointDerivative, setpointsInterval );
    }
  }
  
  strcpy( messageOut, "" );
  for( khint_t shmSHMAxisControlID = (khint_t) 0; shmSHMAxisControlID != kh_end( shmAxisControlsList ); shmSHMAxisControlID++ )
  {
    if( !kh_exist( shmAxisControlsList, shmSHMAxisControlID ) ) continue;
    
    double* measuresList = ShmAxisControl_GetMeasuresList( kh_key( shmAxisControlsList, shmSHMAxisControlID ) );
    
    if( strlen( messageOut ) > 0 ) strcat( messageOut, ":" );
    sprintf( &messageOut[ strlen( messageOut ) ], "%u", shmSHMAxisControlID );
    for( size_t dimensionIndex = 0; dimensionIndex < AXIS_FORCE; dimensionIndex++ )
      sprintf( &messageOut[ strlen( messageOut ) ], " %f", measuresList[ dimensionIndex ] );
  }
  
  if( strlen( messageOut ) > 0 ) 
  {
    DEBUG_UPDATE( "sending message %s to client %d", messageOut, clientID );
    AsyncIPConnection_WriteMessage( clientID, messageOut );
  }
  
  return 0;
}

int main( int argc, char* argv[] )
{
  DEBUG_PRINT( "running %s", argv[ 0 ] );
  
  if( argc >= 2 )
  {
    RobRehabNetwork_Init( argv[ 1 ] );
    
    while( true )
    {
      RobRehabNetwork_Update();
      Timing_Delay( 1000 );
    }
    
    RobRehabNetwork_End();
  }
  
  return 0;
}
