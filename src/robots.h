#ifndef ROBOTS_H
#define ROBOTS_H

#include "actuators.h"
#include "robot_control/interface.h"
#include "plugin_loader.h"

#include "config_parser.h"

#include "klib/khash.h"

#include "debug/async_debug.h"
#include "debug/data_logging.h"

/////////////////////////////////////////////////////////////////////////////////
/////                            CONTROL DEVICE                             /////
/////////////////////////////////////////////////////////////////////////////////

typedef struct _AxisData
{
  double measuresList[ CONTROL_VARS_NUMBER ];
  double setpointsList[ CONTROL_VARS_NUMBER ];
}
AxisData;

typedef AxisData* Axis;

typedef struct _RobotData
{
  Actuator* jointsList;
  double** jointMeasuresTable;
  double** jointSetpointsTable;
  Axis axesList;
  double** axisMeasuresTable;
  double** axisSetpointsTable;
  size_t dofsNumber;
  RobotControlInterface control;
  Controller controller;
  Thread controlThread;
  bool isControlRunning;
}
RobotData;

typedef RobotData* Robot;

KHASH_MAP_INIT_INT( RobotInt, Robot )
khash_t( RobotInt )* robotsList = NULL;


#define ROBOT_FUNCTIONS( namespace, function_init ) \
        function_init( int, namespace, Init, const char* ) \
        function_init( void, namespace, End, int ) \
        function_init( bool, namespace, Enable, int ) \
        function_init( void, namespace, Disable, int ) \
        function_init( Actuator, namespace, GetJoint, int, size_t ) \
        function_init( char*, namespace, GetJointName, int, size_t ) \
        function_init( Axis, namespace, GetAxis, int, size_t ) \
        function_init( char*, namespace, GetAxisName, int, size_t ) \
        function_init( double*, namespace, GetAxisMeasuresList, Axis ) \
        function_init( double, namespace, SetAxisSetpoint, Axis, enum ControlVariables, double ) \
        function_init( size_t, namespace, GetDoFsNumber, int )

INIT_NAMESPACE_INTERFACE( Robots, ROBOT_FUNCTIONS )


static inline Robot LoadRobotData( const char* );
static inline void UnloadRobotData( Robot );

const int ROBOT_INVALID_ID = -1;

static void* AsyncControl( void* );

int Robots_Init( const char* configFileName )
{
  DEBUG_EVENT( 0, "Initializing Axis Controller %s on thread %x", configFileName, THREAD_ID );
  
  if( robotsList == NULL ) robotsList = kh_init( RobotInt );
  
  int configKey = (int) kh_str_hash_func( configFileName );
  
  int insertionStatus;
  khint_t newRobotIndex = kh_put( RobotInt, robotsList, configKey, &insertionStatus );
  if( insertionStatus <= 0 ) return ROBOT_INVALID_ID;

  kh_value( robotsList, newRobotIndex ) = LoadRobotData( configFileName );
  if( kh_value( robotsList, newRobotIndex ) == NULL )
  {
    Robots_End( (int) newRobotIndex );
    return ROBOT_INVALID_ID;
  }
  
  // hack
  //Robots_Enable( (int) kh_key( robotsList, newRobotIndex ) ); 
  
  DEBUG_PRINT( "robot %s created (iterator %u)", configFileName, newRobotIndex );
  
  return (int) kh_key( robotsList, newRobotIndex );
}

void Robots_End( int robotID )
{
  DEBUG_EVENT( 0, "ending Axis Controller %d", robotID );
  
  Robots_Disable( robotID );

  khint_t robotIndex = kh_get( RobotInt, robotsList, (khint_t) robotID );
  if( robotIndex == kh_end( robotsList ) ) return;
  
  Robot robot = kh_value( robotsList, robotIndex );
  UnloadRobotData( robot );
  
  kh_del( RobotInt, robotsList, robotIndex );
  
  if( kh_size( robotsList ) == 0 )
  {
    kh_destroy( RobotInt, robotsList );
    robotsList = NULL;
  }
}

bool Robots_Enable( int robotID )
{
  khint_t robotIndex = kh_get( RobotInt, robotsList, (khint_t) robotID );
  if( robotIndex == kh_end( robotsList ) ) return false;
  
  Robot robot = kh_value( robotsList, robotIndex );
  
  for( size_t jointIndex = 0; jointIndex < robot->dofsNumber; jointIndex++ )
  {
    Actuators.Enable( robot->jointsList[ jointIndex ] );
    
    if( !Actuators.IsEnabled( robot->jointsList[ jointIndex ] ) ) return false;
  }
  
  if( !robot->isControlRunning )
  {
    robot->controlThread = Threading.StartThread( AsyncControl, robot, THREAD_JOINABLE );
  
    if( robot->controlThread == THREAD_INVALID_HANDLE ) return false;
  }
  
  return true;
}

void Robots_Disable( int robotID )
{
  khint_t robotIndex = kh_get( RobotInt, robotsList, (khint_t) robotID );
  if( robotIndex == kh_end( robotsList ) ) return;
  
  Robot robot = kh_value( robotsList, robotIndex );
  
  robot->isControlRunning = false;
  Threading.WaitExit( robot->controlThread, 5000 );
  
  for( size_t jointIndex = 0; jointIndex < robot->dofsNumber; jointIndex++ )
    Actuators.Disable( robot->jointsList[ jointIndex ] );
}

inline Axis Robots_GetAxis( int robotID, size_t axisIndex )
{
  khint_t robotIndex = kh_get( RobotInt, robotsList, (khint_t) robotID );
  if( robotIndex == kh_end( robotsList ) ) return NULL;
  
  Robot robot = kh_value( robotsList, robotIndex );
  
  if( axisIndex >= robot->dofsNumber ) return NULL;
  
  return &(robot->axesList[ axisIndex ]);
}

inline char* Robots_GetAxisName( int robotID, size_t axisIndex )
{
  khint_t robotIndex = kh_get( RobotInt, robotsList, (khint_t) robotID );
  if( robotIndex == kh_end( robotsList ) ) return NULL;
  
  Robot robot = kh_value( robotsList, robotIndex );
  
  if( axisIndex >= robot->dofsNumber ) return NULL;
  
  char** axisNamesList = robot->control.GetAxisNamesList( robot->controller );
  
  if( axisNamesList == NULL ) return NULL;
  
  return axisNamesList[ axisIndex ];
}

inline double* Robots_GetAxisMeasuresList( Axis axis )
{
  if( axis == NULL ) return NULL;
    
  return (double*) axis->measuresList;
}

inline double Robots_SetAxisSetpoint( Axis axis, enum ControlVariables variable, double value )
{
  if( axis == NULL ) return 0.0;
  
  if( variable < 0 || variable >= CONTROL_VARS_NUMBER ) return 0.0;
  
  axis->setpointsList[ variable ] = value;
  
  return value;
}

inline Actuator Robots_GetJoint( int robotID, size_t jointIndex )
{
  khint_t robotIndex = kh_get( RobotInt, robotsList, (khint_t) robotID );
  if( robotIndex == kh_end( robotsList ) ) return NULL;
  
  Robot robot = kh_value( robotsList, robotIndex );
  
  if( jointIndex >= robot->dofsNumber ) return NULL;
  
  return robot->jointsList[ jointIndex ];
}

inline char* Robots_GetJointName( int robotID, size_t jointIndex )
{
  khint_t robotIndex = kh_get( RobotInt, robotsList, (khint_t) robotID );
  if( robotIndex == kh_end( robotsList ) ) return NULL;
  
  Robot robot = kh_value( robotsList, robotIndex );
  
  if( jointIndex >= robot->dofsNumber ) return NULL;
  
  char** jointNamesList = robot->control.GetJointNamesList( robot->controller );
  
  if( jointNamesList == NULL ) return NULL;
  
  return jointNamesList[ jointIndex ];
}

inline size_t Robots_GetDoFsNumber( int robotID )
{
  khint_t robotIndex = kh_get( RobotInt, robotsList, (khint_t) robotID );
  if( robotIndex == kh_end( robotsList ) ) return 0;
  
  Robot robot = kh_value( robotsList, robotIndex );
  
  return robot->dofsNumber;
}

/////////////////////////////////////////////////////////////////////////////////
/////                         ASYNCRONOUS CONTROL                           /////
/////////////////////////////////////////////////////////////////////////////////

const unsigned long CONTROL_PASS_INTERVAL_MS = (unsigned long) ( 1000 * CONTROL_PASS_INTERVAL );

static void* AsyncControl( void* ref_robot )
{
  Robot robot = (Robot) ref_robot;
  
  unsigned long execTime, elapsedTime;
  
  robot->isControlRunning = true;
  
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "starting to run control for robot %p on thread %x", robot, THREAD_ID );
  
  while( robot->isControlRunning )
  {
    execTime = Timing.GetExecTimeMilliseconds();
    
    for( size_t jointIndex = 0; jointIndex < robot->dofsNumber; jointIndex++ )
      Actuators.UpdateMeasures( robot->jointsList[ jointIndex ] );
  
    robot->control.RunStep( robot->controller, robot->jointMeasuresTable, robot->axisMeasuresTable, robot->jointSetpointsTable, robot->axisSetpointsTable );
  
    for( size_t jointIndex = 0; jointIndex < robot->dofsNumber; jointIndex++ )
      Actuators.RunControl( robot->jointsList[ jointIndex ] );
    
    elapsedTime = Timing.GetExecTimeMilliseconds() - execTime;
    DEBUG_UPDATE( "control pass for robot %p (before delay): elapsed time: %lu ms", robot, elapsedTime );
    if( elapsedTime < CONTROL_PASS_INTERVAL_MS ) Timing.Delay( CONTROL_PASS_INTERVAL_MS - elapsedTime );
    DEBUG_UPDATE( "control pass for robot %p (after delay): elapsed time: %lu ms", robot, Timing.GetExecTimeMilliseconds() - execTime );
  }
  
  return NULL;
}

/////////////////////////////////////////////////////////////////////////////////
/////                   CONFIGURATION LOADING/UNLOADING                     /////
/////////////////////////////////////////////////////////////////////////////////

char filePath[ PARSER_MAX_FILE_PATH_LENGTH ];
static inline Robot LoadRobotData( const char* configFileName )
{
  DEBUG_PRINT( "Trying to create robot robot %s", configFileName );
  
  Robot newRobot = NULL;
  
  sprintf( filePath, "robots/%s", configFileName );
  int configFileID = ConfigParsing.LoadConfigFile( filePath );
  if( configFileID != PARSED_DATA_INVALID_ID )
  {
    ParserInterface parser = ConfigParsing.GetParser();
    
    newRobot = (Robot) malloc( sizeof(RobotData) );
    memset( newRobot, 0, sizeof(RobotData) );
  
    bool loadSuccess = false;
    sprintf( filePath, "robot_control/%s", parser.GetStringValue( configFileID, "", "controller.type" ) );
    GET_PLUGIN_INTERFACE( ROBOT_CONTROL_FUNCTIONS, filePath, newRobot->control, loadSuccess );
    if( loadSuccess )
    {
      newRobot->controller = newRobot->control.InitController( parser.GetStringValue( configFileID, "", "controller.id" ) );
      newRobot->dofsNumber = newRobot->control.GetDoFsNumber( newRobot->controller );

      newRobot->jointsList = (Actuator*) calloc( newRobot->dofsNumber, sizeof(Actuator) );
      newRobot->axesList = (Axis) calloc( newRobot->dofsNumber, sizeof(AxisData) );
      newRobot->jointMeasuresTable = (double**) calloc( newRobot->dofsNumber, sizeof(double*) );
      newRobot->jointSetpointsTable = (double**) calloc( newRobot->dofsNumber, sizeof(double*) );
      newRobot->axisMeasuresTable = (double**) calloc( newRobot->dofsNumber, sizeof(double*) );
      newRobot->axisSetpointsTable = (double**) calloc( newRobot->dofsNumber, sizeof(double*) );

      for( size_t jointIndex = 0; jointIndex < newRobot->dofsNumber; jointIndex++ )
      {
        newRobot->jointsList[ jointIndex ] = Actuators.Init( parser.GetStringValue( configFileID, "", "actuators.%lu", jointIndex ) );
        newRobot->jointMeasuresTable[ jointIndex ] = Actuators.GetMeasuresList( newRobot->jointsList[ jointIndex ] );
        newRobot->jointSetpointsTable[ jointIndex ] = Actuators.GetSetpointsList( newRobot->jointsList[ jointIndex ] );
        
        if( newRobot->jointsList[ jointIndex ] == NULL ) loadSuccess = false;
      }

      for( size_t axisIndex = 0; axisIndex < newRobot->dofsNumber; axisIndex++ )
      {
        memset( &(newRobot->axesList[ axisIndex ]), 0, sizeof(AxisData) );
        newRobot->axisMeasuresTable[ axisIndex ] = (double*) newRobot->axesList[ axisIndex ].measuresList;
        newRobot->axisSetpointsTable[ axisIndex ] = (double*) newRobot->axesList[ axisIndex ].setpointsList;
      }
    }

    newRobot->controlThread = Threading.StartThread( AsyncControl, newRobot, THREAD_JOINABLE );
    
    parser.UnloadData( configFileID );

    if( !loadSuccess )
    {
      UnloadRobotData( newRobot );
      return NULL;
    }

    DEBUG_PRINT( "robot %s created", configFileName );
  }
  else
    DEBUG_PRINT( "configuration for robot %s is not available", configFileName );
  
  return newRobot;
}

static inline void UnloadRobotData( Robot robot )
{
  if( robot == NULL ) return;
    
  DEBUG_PRINT( "ending robot robot %p", robot );
  
  robot->control.EndController( robot->controller );
  
  for( size_t jointIndex = 0; jointIndex < robot->dofsNumber; jointIndex++ )
    Actuators.End( robot->jointsList[ jointIndex ] );
  free( robot->jointsList );
  
  free( robot->axesList );
  free( robot->jointMeasuresTable );
  free( robot->jointSetpointsTable );
  free( robot->axisMeasuresTable );
  free( robot->axisSetpointsTable );
    
  free( robot );

  DEBUG_PRINT( "robot robot %p discarded", robot );
}

#endif /* ROBOTS_H */ 
