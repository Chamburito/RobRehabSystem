////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  Copyright (c) 2016 Leonardo José Consoni                                  //
//                                                                            //
//  This file is part of RobRehabSystem.                                      //
//                                                                            //
//  RobRehabSystem is free software: you can redistribute it and/or modify    //
//  it under the terms of the GNU Lesser General Public License as published  //
//  by the Free Software Foundation, either version 3 of the License, or      //
//  (at your option) any later version.                                       //
//                                                                            //
//  RobRehabSystem is distributed in the hope that it will be useful,         //
//  but WITHOUT ANY WARRANTY; without even the implied warranty of            //
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the              //
//  GNU Lesser General Public License for more details.                       //
//                                                                            //
//  You should have received a copy of the GNU Lesser General Public License  //
//  along with RobRehabSystem. If not, see <http://www.gnu.org/licenses/>.    //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////


#include "config_parser.h"

#include "klib/khash.h"

#include "threads/threading.h"
#include "time/timing.h"

#include "debug/async_debug.h"
#include "debug/data_logging.h"

#include "robot_control/interface.h"
#include "robots.h"


/////////////////////////////////////////////////////////////////////////////////
/////                            CONTROL DEVICE                             /////
/////////////////////////////////////////////////////////////////////////////////

struct _JointData
{
  Actuator actuator;
  ControlVariables measures;
  ControlVariables setpoints;
};

struct _AxisData
{
  ControlVariables measures;
  ControlVariables setpoints;
};

struct _RobotData
{
  DECLARE_MODULE_INTERFACE_REF( ROBOT_CONTROL_INTERFACE );
  Controller controller;
  Thread controlThread;
  bool isControlRunning;
  Joint* jointsList;
  ControlVariables** jointMeasuresList;
  ControlVariables** jointSetpointsList;
  size_t jointsNumber;
  Axis* axesList;
  ControlVariables** axisMeasuresList;
  ControlVariables** axisSetpointsList;
  size_t axesNumber;
};

KHASH_MAP_INIT_INT( RobotInt, Robot )
khash_t( RobotInt )* robotsList = NULL;

DEFINE_NAMESPACE_INTERFACE( Robots, ROBOT_INTERFACE )


static inline Robot LoadRobotData( const char* );
static inline void UnloadRobotData( Robot );

static void* AsyncControl( void* );

int Robots_Init( const char* configFileName )
{
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "Initializing Axis Controller %s on thread %lx", configFileName, THREAD_ID );
  
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
  
  // temp
  Robots_Enable( (int) kh_key( robotsList, newRobotIndex ) ); 
  
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
  DEBUG_PRINT( "Trying to enable robot %d", robotID );
  
  khint_t robotIndex = kh_get( RobotInt, robotsList, (khint_t) robotID );
  if( robotIndex == kh_end( robotsList ) ) return false;
  
  Robot robot = kh_value( robotsList, robotIndex );
  
  DEBUG_PRINT( "Enabling robot %p", robot );
  
  for( size_t jointIndex = 0; jointIndex < robot->jointsNumber; jointIndex++ )
  {
    Actuators.Enable( robot->jointsList[ jointIndex ]->actuator );
    
    //if( !Actuators.IsEnabled( robot->jointsList[ jointIndex ]->actuator ) ) return false;
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
  
  for( size_t jointIndex = 0; jointIndex < robot->jointsNumber; jointIndex++ )
    Actuators.Disable( robot->jointsList[ jointIndex ]->actuator );
}

Joint Robots_GetJoint( int robotID, size_t jointIndex )
{
  khint_t robotIndex = kh_get( RobotInt, robotsList, (khint_t) robotID );
  if( robotIndex == kh_end( robotsList ) ) return NULL;
  
  Robot robot = kh_value( robotsList, robotIndex );
  
  if( jointIndex >= robot->jointsNumber ) return NULL;
  
  return robot->jointsList[ jointIndex ];
}

Axis Robots_GetAxis( int robotID, size_t axisIndex )
{
  khint_t robotIndex = kh_get( RobotInt, robotsList, (khint_t) robotID );
  if( robotIndex == kh_end( robotsList ) ) return NULL;
  
  Robot robot = kh_value( robotsList, robotIndex );
  
  if( axisIndex >= robot->axesNumber ) return NULL;
  
  return robot->axesList[ axisIndex ];
}

char* Robots_GetJointName( int robotID, size_t jointIndex )
{
  khint_t robotIndex = kh_get( RobotInt, robotsList, (khint_t) robotID );
  if( robotIndex == kh_end( robotsList ) ) return NULL;
  
  Robot robot = kh_value( robotsList, robotIndex );
  
  if( jointIndex >= robot->jointsNumber ) return NULL;
  
  char** jointNamesList = robot->GetJointNamesList( robot->controller );
  
  if( jointNamesList == NULL ) return NULL;
  
  return jointNamesList[ jointIndex ];
}

char* Robots_GetAxisName( int robotID, size_t axisIndex )
{
  khint_t robotIndex = kh_get( RobotInt, robotsList, (khint_t) robotID );
  if( robotIndex == kh_end( robotsList ) ) return NULL;
  
  Robot robot = kh_value( robotsList, robotIndex );
  
  if( axisIndex >= robot->axesNumber ) return NULL;
  
  char** axisNamesList = robot->GetAxisNamesList( robot->controller );
  
  if( axisNamesList == NULL ) return NULL;
  
  return axisNamesList[ axisIndex ];
}

ControlVariables* Robots_GetJointMeasuresList( Joint joint )
{
  if( joint == NULL ) return NULL;
    
  return &(joint->measures);
}

ControlVariables* Robots_GetAxisMeasuresList( Axis axis )
{
  if( axis == NULL ) return NULL;
    
  return &(axis->measures);
}

double Robots_SetJointSetpoint( Joint joint, enum ControlVariable variable, double value )
{
  if( joint == NULL ) return 0.0;
  
  if( variable >= CONTROL_VARS_NUMBER ) return 0.0;
  
  double* setpointsList = (double*) &(joint->setpoints);
  setpointsList[ variable ] = value;
  
  return value;
}

double Robots_SetAxisSetpoint( Axis axis, enum ControlVariable variable, double value )
{
  if( axis == NULL ) return 0.0;
  
  if( variable >= CONTROL_VARS_NUMBER ) return 0.0;
  
  double* setpointsList = (double*) &(axis->setpoints);
  setpointsList[ variable ] = value;
  
  return value;
}

Actuator Robots_GetJointActuator( Joint joint )
{
  if( joint == NULL ) return NULL;
    
  return joint->actuator;
}

size_t Robots_GetJointsNumber( int robotID )
{
  khint_t robotIndex = kh_get( RobotInt, robotsList, (khint_t) robotID );
  if( robotIndex == kh_end( robotsList ) ) return 0;
  
  Robot robot = kh_value( robotsList, robotIndex );
  
  return robot->jointsNumber;
}

size_t Robots_GetAxesNumber( int robotID )
{
  khint_t robotIndex = kh_get( RobotInt, robotsList, (khint_t) robotID );
  if( robotIndex == kh_end( robotsList ) ) return 0;
  
  Robot robot = kh_value( robotsList, robotIndex );
  
  return robot->axesNumber;
}

/////////////////////////////////////////////////////////////////////////////////
/////                         ASYNCRONOUS CONTROL                           /////
/////////////////////////////////////////////////////////////////////////////////

static void* AsyncControl( void* ref_robot )
{
  const unsigned long CONTROL_PASS_INTERVAL_MS = (unsigned long) ( 1000 * CONTROL_PASS_INTERVAL );
  
  Robot robot = (Robot) ref_robot;
  
  unsigned long execTime, elapsedTime;
  
  robot->isControlRunning = true;
  
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "starting to run control for robot %p on thread %lx", robot, THREAD_ID );
  
  while( robot->isControlRunning )
  {
    execTime = Timing.GetExecTimeMilliseconds();
    
    for( size_t jointIndex = 0; jointIndex < robot->jointsNumber; jointIndex++ )
      (void) Actuators.UpdateMeasures( robot->jointsList[ jointIndex ]->actuator, robot->jointMeasuresList[ jointIndex ] );
  
    robot->RunControlStep( robot->controller, robot->jointMeasuresList, robot->axisMeasuresList, robot->jointSetpointsList, robot->axisSetpointsList );
  
    for( size_t jointIndex = 0; jointIndex < robot->jointsNumber; jointIndex++ )
      (void) Actuators.RunControl( robot->jointsList[ jointIndex ]->actuator, robot->jointMeasuresList[ jointIndex ], robot->jointSetpointsList[ jointIndex ] );
    
    elapsedTime = Timing.GetExecTimeMilliseconds() - execTime;
    ///*DEBUG_UPDATE*/DEBUG_PRINT( "step time for robot %p (before delay): %lu ms", robot, elapsedTime );
    if( elapsedTime < CONTROL_PASS_INTERVAL_MS ) Timing.Delay( CONTROL_PASS_INTERVAL_MS - elapsedTime );
    ///*DEBUG_UPDATE*/DEBUG_PRINT( "step time for robot %p (after delay): %lu ms", robot, Timing.GetExecTimeMilliseconds() - execTime );
  }
  
  return NULL;
}

/////////////////////////////////////////////////////////////////////////////////
/////                   CONFIGURATION LOADING/UNLOADING                     /////
/////////////////////////////////////////////////////////////////////////////////

Robot LoadRobotData( const char* configFileName )
{
  static char filePath[ DATA_IO_MAX_FILE_PATH_LENGTH ];

  DEBUG_PRINT( "Trying to create robot %s", configFileName );
  
  Robot newRobot = NULL;
  
  sprintf( filePath, "robots/%s", configFileName );
  int configFileID = ConfigParsing.LoadConfigFile( filePath );
  if( configFileID != DATA_INVALID_ID )
  {
    newRobot = (Robot) malloc( sizeof(RobotData) );
    memset( newRobot, 0, sizeof(RobotData) );
  
    bool loadSuccess = false;
    sprintf( filePath, "robot_control/%s", ConfigParsing.GetParser()->GetStringValue( configFileID, "", "controller.type" ) );
    LOAD_MODULE_IMPLEMENTATION( ROBOT_CONTROL_INTERFACE, filePath, newRobot, &loadSuccess );
    if( loadSuccess )
    {
      newRobot->controller = newRobot->InitController( ConfigParsing.GetParser()->GetStringValue( configFileID, "", "controller.id" ) );
      
      newRobot->jointsNumber = newRobot->GetJointsNumber( newRobot->controller );
      newRobot->jointsList = (Joint*) calloc( newRobot->jointsNumber, sizeof(Joint) );
      newRobot->jointMeasuresList = (ControlVariables**) calloc( newRobot->jointsNumber, sizeof(ControlVariables*) );
      newRobot->jointSetpointsList = (ControlVariables**) calloc( newRobot->jointsNumber, sizeof(ControlVariables*) );
      for( size_t jointIndex = 0; jointIndex < newRobot->jointsNumber; jointIndex++ )
      {
        newRobot->jointsList[ jointIndex ] = (Joint) malloc( sizeof(JointData) );
        newRobot->jointsList[ jointIndex ]->actuator = Actuators.Init( ConfigParsing.GetParser()->GetStringValue( configFileID, "", "actuators.%lu", jointIndex ) );
        newRobot->jointMeasuresList[ jointIndex ] = (ControlVariables*) &(newRobot->jointsList[ jointIndex ]->measures);
        newRobot->jointSetpointsList[ jointIndex ] = (ControlVariables*) &(newRobot->jointsList[ jointIndex ]->setpoints);
        
        if( newRobot->jointsList[ jointIndex ] == NULL ) loadSuccess = false;
      }

      newRobot->axesNumber = newRobot->GetAxesNumber( newRobot->controller );
      newRobot->axesList = (Axis*) calloc( newRobot->axesNumber, sizeof(Axis) );
      newRobot->axisMeasuresList = (ControlVariables**) calloc( newRobot->axesNumber, sizeof(ControlVariables*) );
      newRobot->axisSetpointsList = (ControlVariables**) calloc( newRobot->axesNumber, sizeof(ControlVariables*) );
      for( size_t axisIndex = 0; axisIndex < newRobot->axesNumber; axisIndex++ )
      {
        newRobot->axesList[ axisIndex ] = (Axis) malloc( sizeof(AxisData) );
        newRobot->axisMeasuresList[ axisIndex ] = (ControlVariables*) &(newRobot->axesList[ axisIndex ]->measures);
        newRobot->axisSetpointsList[ axisIndex ] = (ControlVariables*) &(newRobot->axesList[ axisIndex ]->setpoints);
      }
    }
    
    ConfigParsing.GetParser()->UnloadData( configFileID );

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

void UnloadRobotData( Robot robot )
{
  if( robot == NULL ) return;
    
  DEBUG_PRINT( "ending robot robot %p", robot );
  
  robot->EndController( robot->controller );
  
  for( size_t jointIndex = 0; jointIndex < robot->jointsNumber; jointIndex++ )
  {
    Actuators.End( robot->jointsList[ jointIndex ]->actuator );
    free( robot->jointsList[ jointIndex ] );
  }
  free( robot->jointsList );
  
  for( size_t axisIndex = 0; axisIndex < robot->axesNumber; axisIndex++ )
    free( robot->axesList[ axisIndex ] );
  free( robot->axesList );
  
  free( robot->jointMeasuresList );
  free( robot->jointSetpointsList );
  free( robot->axisMeasuresList );
  free( robot->axisSetpointsList );
    
  free( robot );

  DEBUG_PRINT( "robot robot %p discarded", robot );
}
