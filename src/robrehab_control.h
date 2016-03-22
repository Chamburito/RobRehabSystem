#ifndef ROBREHAB_CONTROL_H
#define ROBREHAB_CONTROL_H

#include "shm_control.h"
#include "shm_axis_control.h"
#include "shm_joint_control.h"
#include "robots.h"

#include "klib/kvec.h"
#include "klib/khash.h"

#include "config_parser.h"

#include "debug/async_debug.h"

const unsigned long UPDATE_INTERVAL_MS = (unsigned long) CONTROL_PASS_INTERVAL * 1000;

kvec_t( int ) robotIDsList;

SHMController sharedRobotAxesInfo;
SHMController sharedRobotJointsInfo;
SHMController sharedRobotAxesData;
SHMController sharedRobotJointsData;

kvec_t( Axis ) axesList;
kvec_t( Actuator ) jointsList;


#define SUBSYSTEM RobRehabControl

#define ROBREHAB_CONTROL_FUNCTIONS( namespace, function_init ) \
        function_init( int, namespace, Init, const char* ) \
        function_init( void, namespace, End, void ) \
        function_init( void, namespace, Update, void )

INIT_NAMESPACE_INTERFACE( SUBSYSTEM, ROBREHAB_CONTROL_FUNCTIONS )


int RobRehabControl_Init( const char* configType )
{
  kv_init( robotIDsList );
  
  kv_init( axesList );
  kv_init( jointsList );
  
  sharedRobotAxesInfo = SHMControl.InitData( "robot_axes_info", SHM_CONTROL_IN );
  sharedRobotJointsInfo = SHMControl.InitData( "robot_joints_info", SHM_CONTROL_IN );
  sharedRobotAxesData = SHMControl.InitData( "robot_axes_data", SHM_CONTROL_IN );
  sharedRobotJointsData = SHMControl.InitData( "robot_joints_data", SHM_CONTROL_IN );

  char robotAxesInfo[ SHM_CONTROL_MAX_DATA_SIZE ] = "";
  char robotJointsInfo[ SHM_CONTROL_MAX_DATA_SIZE ] = "";
  
  if( ConfigParsing.Init( configType ) )
  {
    int configFileID = ConfigParsing.LoadConfigFile( "shared_robots" );
    if( configFileID != PARSED_DATA_INVALID_ID )
    {
      ParserInterface parser = ConfigParsing.GetParser(); 
      
      if( parser.HasKey( configFileID, "robots" ) )
      {
        size_t sharedRobotsNumber = parser.GetListSize( configFileID, "robots" );

        DEBUG_PRINT( "List size: %lu", sharedRobotsNumber );

        for( size_t sharedRobotIndex = 0; sharedRobotIndex < sharedRobotsNumber; sharedRobotIndex++ )
        {
          char* robotName = parser.GetStringValue( configFileID, NULL, "robots.%lu", sharedRobotIndex );
          if( robotName != NULL )
          {
            int robotID = Robots.Init( robotName );
            if( robotID != ROBOT_INVALID_ID )
            {
              if( strlen(robotJointsInfo) > 0 ) strcat( robotJointsInfo, "|" );
              sprintf( robotJointsInfo + strlen(robotJointsInfo), "%lu:%s", kv_size(robotIDsList), robotName );
              kv_push( int, robotIDsList, robotID );

              size_t dofsNumber = Robots.GetDoFsNumber( robotID );
              for( size_t dofIndex = 0; dofIndex < dofsNumber; dofIndex++ )
              {
                char* axisName = Robots.GetAxisName( robotID, dofIndex );
                if( axisName != NULL ) 
                {
                  if( strlen(robotAxesInfo) > 0 ) strcat( robotAxesInfo, "|" );
                  sprintf( robotAxesInfo + strlen(robotAxesInfo), "%lu:%s-%s", kv_size(axesList), robotName, axisName );
                  kv_push( Axis, axesList, Robots.GetAxis( robotID, dofIndex ) );
                }

                char* jointName = Robots.GetJointName( robotID, dofIndex );
                if( jointName != NULL ) 
                {
                  sprintf( robotJointsInfo + strlen(robotJointsInfo), ":%lu:%s", kv_size(jointsList), jointName );
                  kv_push( Actuator, jointsList, Robots.GetJoint( robotID, dofIndex ) );
                }
              }
            }
          }
        }
      }

      parser.UnloadData( configFileID );
    }
  }
  else
    DEBUG_PRINT( "couldn't load %s configuration loading plugin", configType );

  return 0;
}

void RobRehabControl_End()
{
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "Ending RobRehab Control on thread %x", THREAD_ID );

  SHMControl.EndData( sharedRobotAxesInfo );
  SHMControl.EndData( sharedRobotJointsInfo );
  SHMControl.EndData( sharedRobotAxesData );
  SHMControl.EndData( sharedRobotJointsData );
  
  for( size_t robotIndex = 0; robotIndex < kv_size( robotIDsList ); robotIndex++ )
    Robots.End( kv_A( robotIDsList, robotIndex ) );
  
  kv_destroy( axesList );
  kv_destroy( jointsList );
  
  kv_destroy( robotIDsList );

  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "RobRehab Control ended on thread %x", THREAD_ID );
}

const size_t AXIS_DATA_BLOCK_SIZE = SHM_AXIS_FLOATS_NUMBER * sizeof(float);
const size_t JOINT_DATA_BLOCK_SIZE = SHM_JOINT_FLOATS_NUMBER * sizeof(float);
void RobRehabControl_Update()
{
  static uint8_t controlData[ SHM_CONTROL_MAX_DATA_SIZE ];

  uint32_t axesMask = SHMControl.GetData( sharedRobotAxesData, (void*) controlData, 0, SHM_CONTROL_MAX_DATA_SIZE, SHM_CONTROL_REMOVE );
  for( size_t axisIndex = 0; axisIndex < kv_size(axesList); axisIndex++ )
  {
    Axis axis = kv_A( axesList, axisIndex );
    
    if( SHM_CONTROL_IS_BIT_SET( axesMask, axisIndex ) )
    {
      size_t axisDataOffset = axisIndex * (1 + AXIS_DATA_BLOCK_SIZE);
      uint8_t setpointsMask = controlData[ axisDataOffset++ ];

      DEBUG_UPDATE( "updating axis controller %lu", axisIndex );

      float* controlSetpointsList = (float*) (controlData + axisDataOffset);
      //DEBUG_PRINT( "stiffness: %g - setpoint: %g", controlValuesList[ SHM_AXIS_STIFFNESS ], controlValuesList[ SHM_AXIS_POSITION ] );

      if( SHM_CONTROL_IS_BIT_SET( setpointsMask, SHM_AXIS_POSITION ) ) Robots.SetAxisSetpoint( axis, CONTROL_POSITION, controlSetpointsList[ SHM_AXIS_POSITION ] );
      if( SHM_CONTROL_IS_BIT_SET( setpointsMask, SHM_AXIS_VELOCITY ) ) Robots.SetAxisSetpoint( axis, CONTROL_VELOCITY, controlSetpointsList[ SHM_AXIS_VELOCITY ] );
      if( SHM_CONTROL_IS_BIT_SET( setpointsMask, SHM_AXIS_ACCELERATION ) ) Robots.SetAxisSetpoint( axis, CONTROL_ACCELERATION, controlSetpointsList[ SHM_AXIS_ACCELERATION ] );
      if( SHM_CONTROL_IS_BIT_SET( setpointsMask, SHM_AXIS_FORCE ) ) Robots.SetAxisSetpoint( axis, CONTROL_FORCE, controlSetpointsList[ SHM_AXIS_FORCE ] );
      if( SHM_CONTROL_IS_BIT_SET( setpointsMask, SHM_AXIS_STIFFNESS ) ) Robots.SetAxisSetpoint( axis, CONTROL_STIFFNESS, controlSetpointsList[ SHM_AXIS_STIFFNESS ] );
      if( SHM_CONTROL_IS_BIT_SET( setpointsMask, SHM_AXIS_DAMPING ) ) Robots.SetAxisSetpoint( axis, CONTROL_DAMPING, controlSetpointsList[ SHM_AXIS_DAMPING ] );
    }
    
    double* axisMeasuresList = Robots.GetAxisMeasuresList( axis );
    if( axisMeasuresList != NULL )
    {
      float* controlMeasuresList = (float*) controlData;
      size_t axisDataOffset = axisIndex * AXIS_DATA_BLOCK_SIZE;
      
      controlMeasuresList[ SHM_AXIS_POSITION ] = (float) axisMeasuresList[ CONTROL_POSITION ];
      controlMeasuresList[ SHM_AXIS_VELOCITY ] = (float) axisMeasuresList[ CONTROL_VELOCITY ];
      controlMeasuresList[ SHM_AXIS_ACCELERATION ] = (float) axisMeasuresList[ CONTROL_ACCELERATION ];
      controlMeasuresList[ SHM_AXIS_FORCE ] = (float) axisMeasuresList[ CONTROL_FORCE ];
      controlMeasuresList[ SHM_AXIS_STIFFNESS ] = (float) axisMeasuresList[ CONTROL_STIFFNESS ];
      controlMeasuresList[ SHM_AXIS_DAMPING ] = (float) axisMeasuresList[ CONTROL_DAMPING ];
      
      //DEBUG_PRINT( "measures: p: %.3f - v: %.3f - f: %.3f", controlValuesList[ SHM_CONTROL_POSITION ], controlValuesList[ SHM_CONTROL_VELOCITY ], controlValuesList[ SHM_CONTROL_FORCE ] );
      
      SHMControl.SetData( sharedRobotAxesData, (void*) controlData, axisDataOffset, AXIS_DATA_BLOCK_SIZE, 0xFFFFFFFF );
    }
  }

  uint32_t eventMask = SHMControl.GetData( sharedRobotJointsInfo, (void*) controlData, 0, SHM_CONTROL_MAX_DATA_SIZE, SHM_CONTROL_REMOVE );
  if( eventMask )
  {
    size_t eventIndex = 0;
    for( size_t robotIndex = 0; robotIndex < kv_size(robotIDsList); robotIndex++ )
    {
      int robotID = kv_A( robotIDsList, robotIndex );
      
      if( controlData[ eventIndex ] == SHM_ROBOT_DISABLE ) Robots.Disable( robotID );
      else if( controlData[ eventIndex ] == SHM_ROBOT_ENABLE ) Robots.Enable( robotID );
      
      eventIndex++;
      
      size_t robotJointsNumber = Robots.GetDoFsNumber( kv_A( robotIDsList, robotIndex ) );
      for( size_t robotJointIndex = 0; robotJointIndex < robotJointsNumber; robotJointIndex++ )
      {
        Actuator joint = Robots.GetJoint( robotID, robotJointIndex );
        
        if( controlData[ eventIndex ] == SHM_JOINT_RESET ) Actuators.Reset( joint );
        else if( controlData[ eventIndex ] == SHM_JOINT_OFFSET ) Actuators.Reset( joint );
        else if( controlData[ eventIndex ] == SHM_JOINT_CALIBRATE ) Actuators.Calibrate( joint );
        
        eventIndex++;
      }
    }
  }
  
  uint32_t jointsMask = SHMControl.GetData( sharedRobotJointsData, (void*) controlData, 0, SHM_CONTROL_MAX_DATA_SIZE, SHM_CONTROL_REMOVE );
  for( size_t jointIndex = 0; jointIndex < kv_size( jointsList ); jointIndex++ )
  {
    Actuator joint = kv_A( jointsList, jointIndex );
    
    if( SHM_CONTROL_IS_BIT_SET( jointsMask, jointIndex ) )
    {
      size_t jointDataOffset = jointIndex * (1 + JOINT_DATA_BLOCK_SIZE);
      uint8_t setpointsMask = controlData[ jointDataOffset++ ];

      float* controlSetpointsList = (float*) (controlData + jointDataOffset);

      if( SHM_CONTROL_IS_BIT_SET( setpointsMask, SHM_JOINT_POSITION ) ) Actuators.SetSetpoint( joint, CONTROL_POSITION, controlSetpointsList[ SHM_JOINT_POSITION ] );
      if( SHM_CONTROL_IS_BIT_SET( setpointsMask, SHM_JOINT_FORCE ) ) Actuators.SetSetpoint( joint, CONTROL_FORCE, controlSetpointsList[ SHM_JOINT_FORCE ] );
      if( SHM_CONTROL_IS_BIT_SET( setpointsMask, SHM_JOINT_STIFFNESS ) ) Actuators.SetSetpoint( joint, CONTROL_STIFFNESS, controlSetpointsList[ SHM_AXIS_STIFFNESS ] );
    }
    
    double* jointMeasuresList = Actuators.GetMeasuresList( joint );
    if( jointMeasuresList != NULL )
    {
      float* controlMeasuresList = (float*) controlData;
      size_t jointDataOffset = jointIndex * JOINT_DATA_BLOCK_SIZE;
      
      controlMeasuresList[ SHM_JOINT_POSITION ] = (float) jointMeasuresList[ CONTROL_POSITION ];
      controlMeasuresList[ SHM_JOINT_FORCE ] = (float) jointMeasuresList[ CONTROL_FORCE ];
      controlMeasuresList[ SHM_JOINT_STIFFNESS ] = (float) jointMeasuresList[ CONTROL_STIFFNESS ];
      
      SHMControl.SetData( sharedRobotJointsData, (void*) controlData, jointDataOffset, JOINT_DATA_BLOCK_SIZE, 0xFFFFFFFF );
    }
  }
}

#endif // ROBREHAB_CONTROL_H
