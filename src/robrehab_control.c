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

#include <math.h>

#include "shm_control.h"
#include "shm_robot_control.h"
#include "shm_axis_control.h"
#include "shm_joint_control.h"
#include "control_definitions.h"
#include "robots.h"

#include "klib/kvec.h"
#include "klib/khash.h"

#include "config_parser.h"

#include "debug/async_debug.h"
#include "debug/data_logging.h"

#include "robrehab_subsystem.h"


const unsigned long UPDATE_INTERVAL_MS = (unsigned long) ( CONTROL_PASS_INTERVAL * 1000 );


kvec_t( int ) robotIDsList;

SHMController sharedRobotAxesInfo;
SHMController sharedRobotJointsInfo;
SHMController sharedRobotAxesData;
SHMController sharedRobotJointsData;

kvec_t( Axis ) axesList;
kvec_t( Joint ) jointsList;

char robotAxesInfo[ SHM_CONTROL_MAX_DATA_SIZE ] = "";
char robotJointsInfo[ SHM_CONTROL_MAX_DATA_SIZE ] = "";


DEFINE_NAMESPACE_INTERFACE( SubSystem, ROBREHAB_SUBSYSTEM_INTERFACE )

void LoadSharedRobotsInfo( void );


int SubSystem_Init( const char* configType, const char* configDirectory,  const char* logDirectory )
{
  kv_init( robotIDsList );
  
  kv_init( axesList );
  kv_init( jointsList );
  
  sharedRobotAxesInfo = SHMControl.InitData( "robot_axes_info", SHM_CONTROL_IN );
  sharedRobotJointsInfo = SHMControl.InitData( "robot_joints_info", SHM_CONTROL_IN );
  sharedRobotAxesData = SHMControl.InitData( "robot_axes_data", SHM_CONTROL_IN );
  sharedRobotJointsData = SHMControl.InitData( "robot_joints_data", SHM_CONTROL_IN );
  
  DEBUG_PRINT( "looking for %s configuration", configType );
  if( ! ConfigParsing.Init( configType ) )
  {
    DEBUG_PRINT( "couldn't load %s configuration loading plugin", configType );
    return -1;
  }
  
  ConfigParsing.SetBaseDirectory( configDirectory );
  DataLogging.SetBaseDirectory( logDirectory );
  
  LoadSharedRobotsInfo();

  return 0;
}

void SubSystem_End()
{
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "Ending RobRehab Control on thread %lx", THREAD_ID );

  SHMControl.EndData( sharedRobotAxesInfo );
  SHMControl.EndData( sharedRobotJointsInfo );
  SHMControl.EndData( sharedRobotAxesData );
  SHMControl.EndData( sharedRobotJointsData );
  
  for( size_t robotIndex = 0; robotIndex < kv_size( robotIDsList ); robotIndex++ )
    Robots.End( kv_A( robotIDsList, robotIndex ) );
  
  if( kv_size( axesList ) > 0 ) kv_destroy( axesList );
  if( kv_size( jointsList ) > 0 ) kv_destroy( jointsList );
  
  if( kv_size( robotIDsList ) > 0 ) kv_destroy( robotIDsList );

  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "RobRehab Control ended on thread %lx", THREAD_ID );
}

void UpdateEvents()
{
  if( SHMControl.GetControlByte( sharedRobotAxesInfo, 0, SHM_CONTROL_REMOVE ) ) LoadSharedRobotsInfo();
  
  for( size_t robotIndex = 0; robotIndex < kv_size( robotIDsList ); robotIndex++ )
  {
    uint8_t robotCommand = SHMControl.GetControlByte( sharedRobotJointsInfo, robotIndex, SHM_CONTROL_REMOVE );
    if( robotCommand != 0x00 )
    {
      DEBUG_PRINT( "Robot %lu received command %u", robotIndex, robotCommand );
      int robotID = kv_A( robotIDsList, robotIndex );

      uint8_t robotState = 0x00;
      
      if( robotCommand == SHM_ROBOT_DISABLE ) robotState = Robots.Disable( robotID ) ? SHM_ROBOT_DISABLED : 0x00;
      else if( robotCommand == SHM_ROBOT_ENABLE ) robotState = Robots.Enable( robotID ) ? SHM_ROBOT_ENABLED : 0x00;
      else if( robotCommand == SHM_ROBOT_OFFSET ) robotState = Robots.SetControlState( robotID, SHM_ROBOT_OFFSET ) ? SHM_ROBOT_OFFSETTING : 0x00;
      else if( robotCommand == SHM_ROBOT_CALIBRATE ) robotState = Robots.SetControlState( robotID, SHM_ROBOT_CALIBRATE ) ? SHM_ROBOT_CALIBRATING : 0x00;
      else if( robotCommand == SHM_ROBOT_OPTIMIZE ) robotState = Robots.SetControlState( robotID, SHM_ROBOT_OPTIMIZE ) ? SHM_ROBOT_OPTIMIZING : 0x00;
      else if( robotCommand == SHM_ROBOT_OPERATE ) robotState = Robots.SetControlState( robotID, SHM_ROBOT_OPERATE ) ? SHM_ROBOT_OPERATING : 0x00;

      SHMControl.SetControlByte( sharedRobotJointsInfo, robotIndex, robotState );
    }
  }
}

void UpdateAxes()
{
  static uint8_t measureData[ SHM_CONTROL_MAX_DATA_SIZE ], setpointData[ SHM_CONTROL_MAX_DATA_SIZE ];
  static uint8_t updateCount;
  
  SHMControl.GetData( sharedRobotAxesData, (void*) setpointData, 0, SHM_CONTROL_MAX_DATA_SIZE );
  for( size_t axisIndex = 0; axisIndex < kv_size(axesList); axisIndex++ )
  {
    Axis axis = kv_A( axesList, axisIndex );
    
    uint8_t axisMask = SHMControl.GetControlByte( sharedRobotAxesData, axisIndex, SHM_CONTROL_REMOVE );
    //DEBUG_PRINT( "axis %lu setpoint mask: %u%u%u%u%u%u", axisIndex, SHM_CONTROL_IS_BIT_SET( axisMask, SHM_AXIS_POSITION ), SHM_CONTROL_IS_BIT_SET( axisMask, SHM_AXIS_VELOCITY ),
    //                                                                SHM_CONTROL_IS_BIT_SET( axisMask, SHM_AXIS_ACCELERATION ), SHM_CONTROL_IS_BIT_SET( axisMask, SHM_AXIS_FORCE ),
    //                                                                SHM_CONTROL_IS_BIT_SET( axisMask, SHM_AXIS_STIFFNESS ), SHM_CONTROL_IS_BIT_SET( axisMask, SHM_AXIS_DAMPING ) );
    
    DEBUG_UPDATE( "updating axis controller %lu", axisIndex );

    float* controlSetpointsList = (float*) (setpointData + axisIndex * AXIS_DATA_BLOCK_SIZE);
    //DEBUG_PRINT( "stiffness: %g - setpoint: %g", controlSetpointsList[ SHM_AXIS_STIFFNESS ], controlSetpointsList[ SHM_AXIS_POSITION ] );

    if( SHM_CONTROL_IS_BIT_SET( axisMask, SHM_AXIS_POSITION ) ) Robots.SetAxisSetpoint( axis, CONTROL_POSITION, controlSetpointsList[ SHM_AXIS_POSITION ] );
    if( SHM_CONTROL_IS_BIT_SET( axisMask, SHM_AXIS_VELOCITY ) ) Robots.SetAxisSetpoint( axis, CONTROL_VELOCITY, controlSetpointsList[ SHM_AXIS_VELOCITY ] );
    if( SHM_CONTROL_IS_BIT_SET( axisMask, SHM_AXIS_ACCELERATION ) ) Robots.SetAxisSetpoint( axis, CONTROL_ACCELERATION, controlSetpointsList[ SHM_AXIS_ACCELERATION ] );
    if( SHM_CONTROL_IS_BIT_SET( axisMask, SHM_AXIS_FORCE ) ) Robots.SetAxisSetpoint( axis, CONTROL_FORCE, controlSetpointsList[ SHM_AXIS_FORCE ] );
    if( SHM_CONTROL_IS_BIT_SET( axisMask, SHM_AXIS_STIFFNESS ) ) Robots.SetAxisSetpoint( axis, CONTROL_STIFFNESS, controlSetpointsList[ SHM_AXIS_STIFFNESS ] );
    if( SHM_CONTROL_IS_BIT_SET( axisMask, SHM_AXIS_DAMPING ) ) Robots.SetAxisSetpoint( axis, CONTROL_DAMPING, controlSetpointsList[ SHM_AXIS_DAMPING ] );
    
    float* controlMeasuresList = (float*) (measureData + axisIndex * AXIS_DATA_BLOCK_SIZE);
      
    //if( fabs( controlMeasuresList[ SHM_AXIS_POSITION ] - (float) axisMeasures->position ) > 0.05 )
    //{
      controlMeasuresList[ SHM_AXIS_POSITION ] = (float) Robots.GetAxisMeasure( axis, CONTROL_POSITION );
      controlMeasuresList[ SHM_AXIS_VELOCITY ] = (float) Robots.GetAxisMeasure( axis, CONTROL_VELOCITY );
      controlMeasuresList[ SHM_AXIS_ACCELERATION ] = (float) Robots.GetAxisMeasure( axis, CONTROL_ACCELERATION );
      controlMeasuresList[ SHM_AXIS_FORCE ] = (float) Robots.GetAxisMeasure( axis, CONTROL_FORCE );
      controlMeasuresList[ SHM_AXIS_STIFFNESS ] = (float) Robots.GetAxisMeasure( axis, CONTROL_STIFFNESS );
      controlMeasuresList[ SHM_AXIS_DAMPING ] = (float) Robots.GetAxisMeasure( axis, CONTROL_DAMPING );
      
      //DEBUG_PRINT( "measures: p: %.3f - v: %.3f - f: %.3f", controlMeasuresList[ SHM_AXIS_POSITION ], controlMeasuresList[ SHM_AXIS_VELOCITY ], controlMeasuresList[ SHM_AXIS_FORCE ] );
      
      SHMControl.SetControlByte( sharedRobotAxesData, axisIndex, ++updateCount );
    //}
  }
  
  SHMControl.SetData( sharedRobotAxesData, (void*) measureData, 0, SHM_CONTROL_MAX_DATA_SIZE );
}

void UpdateJoints()
{
  static uint8_t measureData[ SHM_CONTROL_MAX_DATA_SIZE ], setpointData[ SHM_CONTROL_MAX_DATA_SIZE ];
  static uint8_t updateCount;
  
  SHMControl.GetData( sharedRobotJointsData, (void*) setpointData, 0, SHM_CONTROL_MAX_DATA_SIZE );
  for( size_t jointIndex = 0; jointIndex < kv_size( jointsList ); jointIndex++ )
  {
    Joint joint = kv_A( jointsList, jointIndex );
    
    uint8_t jointMask = SHMControl.GetControlByte( sharedRobotJointsData, jointIndex, SHM_CONTROL_REMOVE );

    float* controlSetpointsList = (float*) (setpointData + jointIndex * JOINT_DATA_BLOCK_SIZE);
    
    if( SHM_CONTROL_IS_BIT_SET( jointMask, SHM_JOINT_POSITION ) ) Robots.SetJointSetpoint( joint, CONTROL_POSITION, controlSetpointsList[ SHM_JOINT_POSITION ] );
    if( SHM_CONTROL_IS_BIT_SET( jointMask, SHM_JOINT_FORCE ) ) Robots.SetJointSetpoint( joint, CONTROL_FORCE, controlSetpointsList[ SHM_JOINT_FORCE ] );
    if( SHM_CONTROL_IS_BIT_SET( jointMask, SHM_JOINT_STIFFNESS ) ) Robots.SetJointSetpoint( joint, CONTROL_STIFFNESS, controlSetpointsList[ SHM_AXIS_STIFFNESS ] );
    
    float* controlMeasuresList = (float*) (measureData + jointIndex * JOINT_DATA_BLOCK_SIZE);
      
    //if( fabs( controlMeasuresList[ SHM_JOINT_POSITION ] - (float) jointMeasures->position ) > 0.05 )
    //{
      controlMeasuresList[ SHM_JOINT_POSITION ] = (float) Robots.GetJointMeasure( joint, CONTROL_POSITION );
      controlMeasuresList[ SHM_JOINT_FORCE ] = (float) Robots.GetJointMeasure( joint, CONTROL_FORCE );
      controlMeasuresList[ SHM_JOINT_STIFFNESS ] = (float) Robots.GetJointMeasure( joint, CONTROL_STIFFNESS );
     
      SHMControl.SetControlByte( sharedRobotJointsData, jointIndex, ++updateCount );
    //}
  }
  
  SHMControl.SetData( sharedRobotJointsData, (void*) measureData, 0, SHM_CONTROL_MAX_DATA_SIZE );
}

void SubSystem_Update()
{
  UpdateAxes();
  UpdateEvents();
  UpdateJoints();
}


void LoadSharedRobotsInfo()
{
  static uint8_t infoWriteCount;
  
  int configFileID = ConfigParsing.LoadConfigFile( "shared_robots" );
  if( configFileID != DATA_INVALID_ID )
  {
    if( ConfigParsing.GetParser()->HasKey( configFileID, "robots" ) )
    {
      size_t sharedRobotsNumber = ConfigParsing.GetParser()->GetListSize( configFileID, "robots" );
      kv_resize( int, robotIDsList, sharedRobotsNumber );

      DEBUG_PRINT( "List size: %lu", sharedRobotsNumber );

      char jointsInfo[ SHM_CONTROL_MAX_DATA_SIZE ] = "";
      for( size_t sharedRobotIndex = 0; sharedRobotIndex < sharedRobotsNumber; sharedRobotIndex++ )
      {
        char* robotName = ConfigParsing.GetParser()->GetStringValue( configFileID, NULL, "robots.%lu", sharedRobotIndex );
        if( robotName != NULL )
        {
          int robotID = Robots.Init( robotName );
          if( robotID != ROBOT_INVALID_ID )
          {
            if( strlen(robotJointsInfo) > 0 ) strcat( robotJointsInfo, ":" );
            sprintf( robotJointsInfo + strlen(robotJointsInfo), "%lu:%s", kv_size( robotIDsList ), robotName );
            kv_push( int, robotIDsList, robotID );

            size_t axesNumber = Robots.GetAxesNumber( robotID );
            for( size_t axisIndex = 0; axisIndex < axesNumber; axisIndex++ )
            {
              char* axisName = Robots.GetAxisName( robotID, axisIndex );
              if( axisName != NULL )
              {
                if( strlen(robotAxesInfo) > 0 ) strcat( robotAxesInfo, "|" );
                sprintf( robotAxesInfo + strlen(robotAxesInfo), "%lu:%s-%s", kv_size( axesList ), robotName, axisName );
                kv_push( Axis, axesList, Robots.GetAxis( robotID, axisIndex ) );
              }
            }

            size_t jointsNumber = Robots.GetJointsNumber( robotID );
            for( size_t jointIndex = 0; jointIndex < jointsNumber; jointIndex++ )
            {
              char* jointName = Robots.GetJointName( robotID, jointIndex );
              if( jointName != NULL )
              {
                if( strlen(jointsInfo) > 0 ) strcat( jointsInfo, ":" );
                sprintf( jointsInfo + strlen(jointsInfo), "%lu:%s", kv_max( robotIDsList ) + kv_size( jointsList ), jointName );
                kv_push( Joint, jointsList, Robots.GetJoint( robotID, jointIndex ) );
              }
            }
          }
        }
      }
      
      if( strlen( jointsInfo ) > 0 ) sprintf( robotJointsInfo + strlen( robotJointsInfo ), "|%s", jointsInfo );
    }

    ConfigParsing.GetParser()->UnloadData( configFileID );
  }
  
  SHMControl.SetData( sharedRobotAxesInfo, (void*) robotAxesInfo, 0, SHM_CONTROL_MAX_DATA_SIZE );
  SHMControl.SetControlByte( sharedRobotAxesInfo, 0, ++infoWriteCount );
  SHMControl.SetControlByte( sharedRobotAxesInfo, 1, (uint8_t) kv_size( axesList ) );
  
  SHMControl.SetData( sharedRobotJointsInfo, (void*) robotJointsInfo, 0, SHM_CONTROL_MAX_DATA_SIZE );
  SHMControl.SetControlByte( sharedRobotJointsInfo, 0, ++infoWriteCount );
  SHMControl.SetControlByte( sharedRobotJointsInfo, 1, (uint8_t) kv_size( robotIDsList ) );
  SHMControl.SetControlByte( sharedRobotJointsInfo, 2, (uint8_t) kv_size( jointsList ) );
}
