#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include "actuator_control.h"
#include "mechanics/mechanical_interface.h"

#include "config_parser.h"
#include "plugin_loader.h"

#include "klib/khash.h"

#include "debug/async_debug.h"
#include "debug/data_logging.h"

/////////////////////////////////////////////////////////////////////////////////
/////                            CONTROL DEVICE                             /////
/////////////////////////////////////////////////////////////////////////////////

typedef struct _AxisData
{
  char name[ PARSER_MAX_VALUE_LENGTH ];
  double measuresList[ CONTROL_VARS_NUMBER ];
  double setpointsList[ CONTROL_VARS_NUMBER ];
}
AxisData;

typedef AxisData* Axis;

typedef struct _RobotControllerData
{
  MechanicsInterface mechanics;
  MechanicalModel mechanism;
  Actuator* jointsList;
  double** jointMeasuresTable;
  double** jointSetpointsTable;
  Axis axesList;
  double** axisMeasuresTable;
  double** axisSetpointsTable;
  size_t dofsNumber;
  Thread controlThread;
  bool isControlRunning;
}
RobotControllerData;

typedef RobotControllerData* RobotController;

KHASH_MAP_INIT_INT( RobotControlInt, RobotController )
khash_t( RobotControlInt )* robotsList = NULL;


#define ROBOT_CONTROL_INTERFACE( namespace, function_init ) \
        function_init( int, namespace, InitController, const char* ) \
        function_init( void, namespace, EndController, int ) \
        function_init( bool, namespace, Enable, int ) \
        function_init( void, namespace, Disable, int ) \
        function_init( Actuator, namespace, GetJoint, int, size_t ) \
        function_init( Axis, namespace, GetAxis, int, size_t ) \
        function_init( char*, namespace, GetAxisName, Axis ) \
        function_init( double*, namespace, GetAxisMeasuresList, Axis ) \
        function_init( double, namespace, SetAxisSetpoint, Axis, enum ControlVariables, double ) \
        function_init( size_t, namespace, GetDoFsNumber, int )

INIT_NAMESPACE_INTERFACE( RobotControl, ROBOT_CONTROL_INTERFACE )


static inline RobotController LoadControllerData( const char* );
static inline void UnloadControllerData( RobotController );

const int ROBOT_INVALID_ID = -1;

static void* AsyncControl( void* );

int RobotControl_InitController( const char* configFileName )
{
  DEBUG_EVENT( 0, "Initializing Axis Controller %s on thread %x", configFileName, THREAD_ID );
  
  if( robotsList == NULL ) robotsList = kh_init( RobotControlInt );
  
  int configKey = (int) kh_str_hash_func( configFileName );
  
  int insertionStatus;
  khint_t newRobotIndex = kh_put( RobotControlInt, robotsList, configKey, &insertionStatus );
  if( insertionStatus <= 0 ) return ROBOT_INVALID_ID;

  kh_value( robotsList, newRobotIndex ) = LoadControllerData( configFileName );
  if( kh_value( robotsList, newRobotIndex ) == NULL )
  {
    RobotControl_EndController( (int) newRobotIndex );
    return ROBOT_INVALID_ID;
  }
  
  DEBUG_PRINT( "robot robot %s created (iterator %u)", configFileName, newRobotIndex );
  
  return (int) kh_key( robotsList, newRobotIndex );
}

void RobotControl_EndController( int robotID )
{
  DEBUG_EVENT( 0, "ending Axis Controller %d", robotID );
  
  RobotControl_Disable( robotID );

  khint_t robotIndex = kh_get( RobotControlInt, robotsList, (khint_t) robotID );
  if( robotIndex == kh_end( robotsList ) ) return;
  
  RobotController robot = kh_value( robotsList, robotIndex );
  UnloadControllerData( robot );
  
  kh_del( RobotControlInt, robotsList, robotIndex );
  
  if( kh_size( robotsList ) == 0 )
  {
    kh_destroy( RobotControlInt, robotsList );
    robotsList = NULL;
  }
}

bool RobotControl_Enable( int robotID )
{
  khint_t robotIndex = kh_get( RobotControlInt, robotsList, (khint_t) robotID );
  if( robotIndex == kh_end( robotsList ) ) return false;
  
  RobotController robot = kh_value( robotsList, robotIndex );
  
  for( size_t jointIndex = 0; jointIndex < robot->dofsNumber; jointIndex++ )
  {
    ActuatorControl.Enable( robot->jointsList[ jointIndex ] );
    
    if( !ActuatorControl.IsEnabled( robot->jointsList[ jointIndex ] ) ) return false;
  }
  
  if( !robot->isControlRunning )
  {
    robot->controlThread = Threading.StartThread( AsyncControl, robot, THREAD_JOINABLE );
  
    if( robot->controlThread == THREAD_INVALID_HANDLE ) return false;
  }
  
  return true;
}

void RobotControl_Disable( int robotID )
{
  khint_t robotIndex = kh_get( RobotControlInt, robotsList, (khint_t) robotID );
  if( robotIndex == kh_end( robotsList ) ) return;
  
  RobotController robot = kh_value( robotsList, robotIndex );
  
  robot->isControlRunning = false;
  Threading.WaitExit( robot->controlThread, 5000 );
  
  for( size_t jointIndex = 0; jointIndex < robot->dofsNumber; jointIndex++ )
    ActuatorControl.Disable( robot->jointsList[ jointIndex ] );
}

inline Axis RobotControl_GetAxis( int robotID, size_t axisIndex )
{
  khint_t robotIndex = kh_get( RobotControlInt, robotsList, (khint_t) robotID );
  if( robotIndex == kh_end( robotsList ) ) return NULL;
  
  RobotController robot = kh_value( robotsList, robotIndex );
  
  if( axisIndex >= robot->dofsNumber ) return NULL;
  
  return &(robot->axesList[ axisIndex ]);
}

inline double* RobotControl_GetAxisMeasuresList( Axis axis )
{
  if( axis == NULL ) return NULL;
    
  return (double*) axis->measuresList;
}

inline double RobotControl_SetAxisSetpoint( Axis axis, enum ControlVariables variable, double value )
{
  if( axis == NULL ) return 0.0;
  
  if( variable < 0 || variable >= CONTROL_VARS_NUMBER ) return 0.0;
  
  axis->setpointsList[ variable ] = value;
  
  return value;
}

inline Actuator RobotControl_GetJoint( int robotID, size_t jointIndex )
{
  khint_t robotIndex = kh_get( RobotControlInt, robotsList, (khint_t) robotID );
  if( robotIndex == kh_end( robotsList ) ) return NULL;
  
  RobotController robot = kh_value( robotsList, robotIndex );
  
  if( jointIndex >= robot->dofsNumber ) return NULL;
  
  return robot->jointsList[ jointIndex ];
}

inline size_t RobotControl_GetDoFsNumber( int robotID )
{
  khint_t robotIndex = kh_get( RobotControlInt, robotsList, (khint_t) robotID );
  if( robotIndex == kh_end( robotsList ) ) return 0;
  
  RobotController robot = kh_value( robotsList, robotIndex );
  
  return robot->dofsNumber;
}

/////////////////////////////////////////////////////////////////////////////////
/////                         ASYNCRONOUS CONTROL                           /////
/////////////////////////////////////////////////////////////////////////////////

const unsigned long CONTROL_PASS_INTERVAL_MS = (unsigned long) ( 1000 * CONTROL_PASS_INTERVAL );

static void* AsyncControl( void* ref_robot )
{
  RobotController robot = (RobotController) ref_robot;
  
  unsigned long execTime, elapsedTime;
  
  robot->isControlRunning = true;
  
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "starting to run control for robot %p on thread %x", robot, THREAD_ID );
  
  while( robot->isControlRunning )
  {
    execTime = Timing.GetExecTimeMilliseconds();
    
    for( size_t jointIndex = 0; jointIndex < robot->dofsNumber; jointIndex++ )
      ActuatorControl.UpdateMeasures( robot->jointsList[ jointIndex ] );
  
    robot->mechanics.SolveForwardMechanics( robot->mechanism, robot->jointMeasuresTable, robot->axisMeasuresTable );
  
    robot->mechanics.SolveInverseMechanics( robot->mechanism, robot->axisSetpointsTable, robot->jointSetpointsTable );
  
    robot->mechanics.GetJointControlActions( robot->mechanism, robot->jointMeasuresTable, robot->jointSetpointsTable );
  
    for( size_t jointIndex = 0; jointIndex < robot->dofsNumber; jointIndex++ )
      ActuatorControl.RunControl( robot->jointsList[ jointIndex ] );
    
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
static inline RobotController LoadControllerData( const char* configFileName )
{
  DEBUG_PRINT( "Trying to create robot robot %s", configFileName );
  
  RobotController newRobot = NULL;
  
  sprintf( filePath, "robots/%s", configFileName );
  int configFileID = ConfigParsing.LoadConfigFile( filePath );
  if( configFileID != PARSED_DATA_INVALID_ID )
  {
    ParserInterface parser = ConfigParsing.GetParser();
    
    newRobot = (RobotController) malloc( sizeof(RobotControllerData) );
    memset( newRobot, 0, sizeof(RobotControllerData) );
  
    bool loadSuccess = false;
    sprintf( filePath, "mechanics/%s", parser.GetStringValue( configFileID, "", "mechanism.type" ) );
    GET_PLUGIN_IMPLEMENTATION( MECHANICS_FUNCTIONS, filePath, newRobot->mechanics, loadSuccess );
    if( loadSuccess )
    {
      newRobot->mechanism = newRobot->mechanics.InitModel( parser.GetStringValue( configFileID, "", "mechanism.id" ) );
      newRobot->dofsNumber = newRobot->mechanics.GetDoFsNumber( newRobot->mechanism );

      newRobot->jointsList = (Actuator*) calloc( newRobot->dofsNumber, sizeof(Actuator) );
      newRobot->axesList = (Axis) calloc( newRobot->dofsNumber, sizeof(AxisData) );
      newRobot->jointMeasuresTable = (double**) calloc( newRobot->dofsNumber, sizeof(double*) );
      newRobot->jointSetpointsTable = (double**) calloc( newRobot->dofsNumber, sizeof(double*) );
      newRobot->axisMeasuresTable = (double**) calloc( newRobot->dofsNumber, sizeof(double*) );
      newRobot->axisSetpointsTable = (double**) calloc( newRobot->dofsNumber, sizeof(double*) );

      for( size_t jointIndex = 0; jointIndex < newRobot->dofsNumber; jointIndex++ )
      {
        newRobot->jointsList[ jointIndex ] = ActuatorControl.InitController( parser.GetStringValue( configFileID, "", "actuators.%lu", jointIndex ) );
        newRobot->jointMeasuresTable[ jointIndex ] = ActuatorControl.GetMeasuresList( newRobot->jointsList[ jointIndex ] );
        newRobot->jointSetpointsTable[ jointIndex ] = ActuatorControl.GetSetpointsList( newRobot->jointsList[ jointIndex ] );
        
        if( newRobot->jointsList[ jointIndex ] == NULL ) loadSuccess = false;
      }

      for( size_t axisIndex = 0; axisIndex < newRobot->dofsNumber; axisIndex++ )
      {
        memset( &(newRobot->axesList[ axisIndex ]), 0, sizeof(AxisData) );
        newRobot->axisMeasuresTable[ axisIndex ] = (double*) newRobot->axesList[ axisIndex ].measuresList;
        newRobot->axisSetpointsTable[ axisIndex ] = (double*) newRobot->axesList[ axisIndex ].setpointsList;
      }
    }

    parser.UnloadData( configFileID );

    if( !loadSuccess )
    {
      UnloadControllerData( newRobot );
      return NULL;
    }

    DEBUG_PRINT( "robot robot %s created", configFileName );
  }
  else
    DEBUG_PRINT( "configuration for robot %s is not available", configFileName );
  
  return newRobot;
}

static inline void UnloadControllerData( RobotController robot )
{
  if( robot == NULL ) return;
    
  DEBUG_PRINT( "ending robot robot %p", robot );
  
  robot->mechanics.EndModel( robot->mechanism );
  
  for( size_t jointIndex = 0; jointIndex < robot->dofsNumber; jointIndex++ )
    ActuatorControl.EndController( robot->jointsList[ jointIndex ] );
  free( robot->jointsList );
  
  free( robot->axesList );
  free( robot->jointMeasuresTable );
  free( robot->jointSetpointsTable );
  free( robot->axisMeasuresTable );
  free( robot->axisSetpointsTable );
    
  free( robot );

  DEBUG_PRINT( "robot robot %p discarded", robot );
}

#endif /* ROBOT_CONTROL_H */ 
