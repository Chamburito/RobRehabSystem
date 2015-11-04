#ifndef ROBREHAB_CONTROL_H
#define ROBREHAB_CONTROL_H

#include "shm_control.h"
#include "shm_axis_control.h"
#include "shm_emg_control.h"
#include "robot_control.h"

#include "klib/kvec.h"
#include "klib/khash.h"

#include "config_parser.h"

#include "debug/async_debug.h"

const unsigned long UPDATE_INTERVAL_MS = (unsigned long) CONTROL_SAMPLING_INTERVAL * 1000;

typedef struct _SharedAxisController
{
  int robotID;
  size_t controllerIndex;
  SHMController sharedData;
}
SharedAxisController;

kvec_t( SharedAxisController ) sharedDoFControllersList;
kvec_t( SharedAxisController ) sharedJointControllersList;
kvec_t( int ) robotControllersList;


#define SUBSYSTEM RobRehabControl

#define ROBREHAB_CONTROL_FUNCTIONS( namespace, function_init ) \
        function_init( int, namespace, Init, void ) \
        function_init( void, namespace, End, void ) \
        function_init( void, namespace, Update, void )

INIT_NAMESPACE_INTERFACE( SUBSYSTEM, ROBREHAB_CONTROL_FUNCTIONS )


int RobRehabControl_Init()
{
  kv_init( sharedDoFControllersList );
  kv_init( sharedJointControllersList );
  kv_init( robotControllersList );

  if( ConfigParsing.Init( "JSON" ) )
  {
    ParserInterface parser = ConfigParsing.GetParser();
    int configFileID = parser.LoadFileData( "shared_robots" );
    if( configFileID != -1 )
    {
      if( parser.HasKey( configFileID, "robots" ) )
      {
        size_t sharedRobotsNumber = parser.GetListSize( configFileID, "robots" );

        DEBUG_PRINT( "List size: %u", sharedRobotsNumber );

        for( size_t sharedRobotIndex = 0; sharedRobotIndex < sharedRobotsNumber; sharedRobotIndex++ )
        {
          char* robotName = parser.GetStringValue( configFileID, NULL, "robots.%lu", sharedRobotIndex );
          if( robotName != NULL )
          {
            int robotControllerID = RobotControl.InitController( robotName );
            if( robotControllerID != ROBOT_CONTROLLER_INVALID_ID )
            {
              kv_push( int, robotControllersList, robotControllerID );

              size_t dofsNumber = parser.GetListSize( configFileID, "robots.%lu.dofs", sharedRobotIndex );
              for( size_t dofIndex = 0; dofIndex < dofsNumber; dofIndex++ )
              {
                char* dofName = parser.GetStringValue( configFileID, "", "robots.%lu.dofs.%lu", sharedRobotIndex, dofIndex );
                SharedAxisController newSharedDoF = { .robotID = robotControllerID, .controllerIndex = dofIndex };
                char robotDoFName[ PARSER_MAX_KEY_PATH_LENGTH ];
                sprintf( robotDoFName, "%s-%s", robotName, dofName );
                newSharedDoF.sharedData = SHMControl.InitData( robotDoFName, SHM_CONTROL_IN );
                if( newSharedDoF.sharedData != NULL ) kv_push( SharedAxisController, sharedDoFControllersList, newSharedDoF );
              }

              size_t jointsNumber = parser.GetListSize( configFileID, "robots.%lu.joints", sharedRobotIndex );
              for( size_t jointIndex = 0; jointIndex < jointsNumber; jointIndex++ )
              {
                SharedAxisController newSharedJoint = { .robotID = robotControllerID, .controllerIndex = jointIndex };
                char* jointName = parser.GetStringValue( configFileID, "", "robots.%lu.joints.%lu", sharedRobotIndex, jointIndex );
                newSharedJoint.sharedData = SHMControl.InitData( jointName, SHM_CONTROL_IN );
                if( newSharedJoint.sharedData != NULL ) kv_push( SharedAxisController, sharedJointControllersList, newSharedJoint );
              }
            }
          }
        }
      }

      parser.UnloadData( configFileID );
    }
  }
  else
    DEBUG_PRINT( "couldn't load %s configuration loading plugin", "JSON" );

  return 0;
}

void RobRehabControl_End()
{
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "Ending RobRehab Control on thread %x", THREAD_ID );

  for( size_t controllerIndex = 0; controllerIndex < kv_size( sharedDoFControllersList ); controllerIndex++ )
  {
    RobotControl.EndController( kv_A( sharedDoFControllersList, controllerIndex ).robotID );
    SHMControl.EndData( kv_A( sharedDoFControllersList, controllerIndex ).sharedData );
  }

  kv_destroy( sharedDoFControllersList );
  kv_destroy( sharedJointControllersList );
  kv_destroy( robotControllersList );

  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "RobRehab Control ended on thread %x", THREAD_ID );
}

void RobRehabControl_Update()
{
  static float controlValuesList[ SHM_CONTROL_MAX_FLOATS_NUMBER ];

  for( size_t dofControllerIndex = 0; dofControllerIndex < kv_size( sharedDoFControllersList ); dofControllerIndex++ )
  {
    DEBUG_UPDATE( "updating axis controller %u", controllerIndex );

    SharedAxisController* sharedController = &(kv_A( sharedDoFControllersList, dofControllerIndex ));
    SHMController sharedDoF = sharedController->sharedData;
    int robotControllerID = sharedController->robotID;
    size_t dofControllerIndex = sharedController->controllerIndex;

    uint8_t command = SHMControl.GetByteValue( sharedDoF, SHM_CONTROL_REMOVE );
    if( command != SHM_CONTROL_BYTE_NULL )
    {
      //DEBUG_PRINT( "received command: %x", command );

      if( command == SHM_COMMAND_ENABLE ) RobotControl.Enable( robotControllerID );
      else if( command == SHM_COMMAND_DISABLE ) RobotControl.Disable( robotControllerID );
      else if( command == SHM_COMMAND_RESET ) RobotControl.Reset( robotControllerID );
      else if( command == SHM_COMMAND_CALIBRATE ) RobotControl.Calibrate( robotControllerID );

      if( RobotControl.IsEnabled( robotControllerID ) ) SHMControl.SetByteValue( sharedDoF, SHM_STATE_ENABLED );
    }

    if( RobotControl.HasError( robotControllerID ) ) SHMControl.SetByteValue( sharedDoF, SHM_STATE_ERROR );

    uint8_t dataMask = SHMControl.GetNumericValuesList( sharedDoF, controlValuesList, SHM_CONTROL_REMOVE );
    if( dataMask )
    {
      double* controlSetpointsList = RobotControl.GetDoFSetpointsList( robotControllerID, dofControllerIndex );
      //DEBUG_PRINT( "setpoints: p: %.3f - v: %.3f - s: %.3f", controlValuesList[ SHM_CONTROL_POSITION ], controlValuesList[ SHM_CONTROL_VELOCITY ], controlValuesList[ SHM_CONTROL_STIFFNESS ] );

      if( (dataMask & SHM_CONTROL_BIT_INDEX( SHM_AXIS_POSITION )) ) controlSetpointsList[ CONTROL_POSITION ] = controlValuesList[ SHM_AXIS_POSITION ];
      if( (dataMask & SHM_CONTROL_BIT_INDEX( SHM_AXIS_VELOCITY )) ) controlSetpointsList[ CONTROL_VELOCITY ] = controlValuesList[ SHM_AXIS_VELOCITY ];

      if( (dataMask & SHM_CONTROL_BIT_INDEX( SHM_AXIS_STIFFNESS )) | (dataMask & SHM_CONTROL_BIT_INDEX( SHM_AXIS_DAMPING )) )
        RobotControl.SetDoFImpedance( robotControllerID, dofControllerIndex, controlValuesList[ SHM_AXIS_STIFFNESS ], controlValuesList[ SHM_AXIS_DAMPING ] );
    }

    double* controlMeasuresList = RobotControl.GetDoFMeasuresList( robotControllerID, dofControllerIndex );
    if( controlMeasuresList != NULL )
    {
      controlValuesList[ SHM_AXIS_POSITION ] = (float) controlMeasuresList[ CONTROL_POSITION ];
      controlValuesList[ SHM_AXIS_VELOCITY ] = (float) controlMeasuresList[ CONTROL_VELOCITY ];
      controlValuesList[ SHM_AXIS_ACCELERATION ] = (float) controlMeasuresList[ CONTROL_ACCELERATION ];
      controlValuesList[ SHM_AXIS_FORCE ] = (float) controlMeasuresList[ CONTROL_FORCE ];
      SHMControl.SetNumericValuesList( sharedDoF, controlValuesList, 0xFF );

      //DEBUG_PRINT( "measures: p: %.3f - v: %.3f - f: %.3f", controlValuesList[ SHM_CONTROL_POSITION ], controlValuesList[ SHM_CONTROL_VELOCITY ], controlValuesList[ SHM_CONTROL_FORCE ] );
    }
  }

  for( size_t robotIndex = 0; robotIndex < kv_size( robotControllersList ); robotIndex++ )
    RobotControl.Update( kv_A( robotControllersList, robotIndex ) );

  for( size_t jointControllerIndex = 0; jointControllerIndex < kv_size( sharedJointControllersList ); jointControllerIndex++ )
  {
    SharedAxisController* sharedController = &(kv_A( sharedJointControllersList, jointControllerIndex ));
    SHMController sharedJoint = sharedController->sharedData;
    int robotControllerID = sharedController->robotID;
    size_t jointControllerIndex = sharedController->controllerIndex;

    double* jointMeasuresList = RobotControl.GetJointMeasuresList( robotControllerID, jointControllerIndex );
    if( jointMeasuresList != NULL )
    {
      controlValuesList[ SHM_EMG_JOINT_ANGLE ] = (float) jointMeasuresList[ CONTROL_POSITION ] ;
      controlValuesList[ SHM_EMG_TORQUE ] = (float) jointMeasuresList[ CONTROL_FORCE ] ;
      SHMControl.SetNumericValuesList( sharedJoint, controlValuesList, 0xFF );
    }
  }
}

#endif // ROBREHAB_CONTROL_H
