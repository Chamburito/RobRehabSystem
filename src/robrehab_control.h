#ifndef ROBREHAB_CONTROL_H
#define ROBREHAB_CONTROL_H

#include "shm_dof_control.h"
#include "shm_axis_control.h"
#include "shm_emg_control.h"
#include "robot_control.h"

#include "klib/kvec.h"
#include "klib/khash.h"

#include "config_parser.h"

#include "debug/async_debug.h"

const unsigned long UPDATE_INTERVAL_MS = (unsigned long) CONTROL_PASS_INTERVAL * 1000;

kvec_t( SHMDoFController ) sharedAxisControllersList;
kvec_t( SHMDoFController ) sharedJointControllersList;
kvec_t( int ) robotControllersList;


#define SUBSYSTEM RobRehabControl

#define ROBREHAB_CONTROL_FUNCTIONS( namespace, function_init ) \
        function_init( int, namespace, Init, void ) \
        function_init( void, namespace, End, void ) \
        function_init( void, namespace, Update, void )

INIT_NAMESPACE_INTERFACE( SUBSYSTEM, ROBREHAB_CONTROL_FUNCTIONS )


int RobRehabControl_Init()
{
  kv_init( sharedAxisControllersList );
  kv_init( sharedJointControllersList );
  kv_init( robotControllersList );

  if( ConfigParsing.Init( "JSON" ) )
  {
    int configFileID = ConfigParsing.LoadConfigFile( "shared_robots" );
    if( configFileID != PARSED_DATA_INVALID_ID )
    {
      ParserInterface parser = ConfigParsing.GetParser(); 
      
      if( parser.HasKey( configFileID, "robots" ) )
      {
        size_t sharedRobotsNumber = parser.GetListSize( configFileID, "robots" );

        DEBUG_PRINT( "List size: %u", sharedRobotsNumber );

        char robotVarName[ PARSER_MAX_KEY_PATH_LENGTH ];
        for( size_t sharedRobotIndex = 0; sharedRobotIndex < sharedRobotsNumber; sharedRobotIndex++ )
        {
          char* robotName = parser.GetStringValue( configFileID, NULL, "robots.%lu.name", sharedRobotIndex );
          if( robotName != NULL )
          {
            int robotControllerID = RobotControl.InitController( robotName );
            if( robotControllerID != ROBOT_CONTROLLER_INVALID_ID )
            {
              kv_push( int, robotControllersList, robotControllerID );

              size_t axesNumber = parser.GetListSize( configFileID, "robots.%lu.axes", sharedRobotIndex );
              for( size_t axisIndex = 0; axisIndex < axesNumber; axisIndex++ )
              {
                SHMDoFController newSharedAxis = (SHMDoFController) malloc( sizeof(SHMDoFControllerData) );
                newSharedAxis->ref_localDoF = (void*) RobotControl.GetAxis( robotControllerID, axisIndex );
                sprintf( robotVarName, "%s-%s", robotName, parser.GetStringValue( configFileID, "", "robots.%lu.axes.%lu", sharedRobotIndex, axisIndex ) );
                newSharedAxis->sharedData = SHMControl.InitData( robotVarName, SHM_CONTROL_IN );
                kv_push( SHMDoFController, sharedAxisControllersList, newSharedAxis );
              }

              size_t jointsNumber = parser.GetListSize( configFileID, "robots.%lu.joints", sharedRobotIndex );
              for( size_t jointIndex = 0; jointIndex < jointsNumber; jointIndex++ )
              {
                SHMDoFController newSharedJoint = (SHMDoFController) malloc( sizeof(SHMDoFControllerData) );
                newSharedJoint->ref_localDoF = (void*) RobotControl.GetJoint( robotControllerID, jointIndex );
                sprintf( robotVarName, "%s-%s", robotName, parser.GetStringValue( configFileID, "", "robots.%lu.joints.%lu", sharedRobotIndex, jointIndex ) );
                newSharedJoint->sharedData = SHMControl.InitData( robotVarName, SHM_CONTROL_IN );
                kv_push( SHMDoFController, sharedJointControllersList, newSharedJoint );
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

  for( size_t controllerIndex = 0; controllerIndex < kv_size( robotControllersList ); controllerIndex++ )
    RobotControl.EndController( kv_A( robotControllersList, controllerIndex ) );
  
  for( size_t controllerIndex = 0; controllerIndex < kv_size( sharedAxisControllersList ); controllerIndex++ )
    SHMControl.EndData( kv_A( sharedAxisControllersList, controllerIndex )->sharedData );
  
  for( size_t controllerIndex = 0; controllerIndex < kv_size( sharedJointControllersList ); controllerIndex++ )
    SHMControl.EndData( kv_A( sharedJointControllersList, controllerIndex )->sharedData );

  kv_destroy( sharedAxisControllersList );
  kv_destroy( sharedJointControllersList );
  kv_destroy( robotControllersList );

  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "RobRehab Control ended on thread %x", THREAD_ID );
}

void RobRehabControl_Update()
{
  static float controlValuesList[ SHM_CONTROL_MAX_FLOATS_NUMBER ];

  for( size_t axisControllerIndex = 0; axisControllerIndex < kv_size( sharedAxisControllersList ); axisControllerIndex++ )
  {
    DEBUG_UPDATE( "updating axis controller %u", controllerIndex );

    SHMDoFController sharedController = kv_A( sharedAxisControllersList, axisControllerIndex );
    Axis localAxis = (Axis) sharedController->ref_localDoF;

    /*uint8_t command = SHMControl.GetByteValue( sharedDoF, SHM_CONTROL_REMOVE );
    if( command != SHM_CONTROL_BYTE_NULL )
    {
      //DEBUG_PRINT( "received command: %x", command );

      else if( command == SHM_COMMAND_OFFSET ) RobotControl.SetAxisOffset( localAxis );
      else if( command == SHM_COMMAND_CALIBRATE ) RobotControl.CalibrateAxis( localAxis );
    }*/

    uint8_t dataMask = SHMControl.GetNumericValuesList( sharedController->sharedData, controlValuesList, SHM_CONTROL_REMOVE );
    if( dataMask )
    {
      //DEBUG_PRINT( "stiffness: %g - setpoint: %g", controlValuesList[ SHM_AXIS_STIFFNESS ], controlValuesList[ SHM_AXIS_POSITION ] );
      
      if( SHM_CONTROL_IS_BIT_SET( dataMask, SHM_AXIS_POSITION ) ) RobotControl.SetAxisSetpoint( localAxis, SHM_AXIS_POSITION, controlValuesList[ SHM_AXIS_POSITION ] );
      if( SHM_CONTROL_IS_BIT_SET( dataMask, SHM_AXIS_VELOCITY ) ) RobotControl.SetAxisSetpoint( localAxis, SHM_AXIS_VELOCITY, controlValuesList[ SHM_AXIS_VELOCITY ] );

      if( SHM_CONTROL_IS_BIT_SET( dataMask, SHM_AXIS_STIFFNESS ) | SHM_CONTROL_IS_BIT_SET( dataMask, SHM_AXIS_DAMPING ) )
        RobotControl.SetAxisImpedance( localAxis, controlValuesList[ SHM_AXIS_STIFFNESS ], controlValuesList[ SHM_AXIS_DAMPING ] );
    }

    double* controlMeasuresList = RobotControl.GetAxisMeasuresList( localAxis );
    if( controlMeasuresList != NULL )
    {
      controlValuesList[ SHM_AXIS_POSITION ] = (float) controlMeasuresList[ CONTROL_POSITION ];
      controlValuesList[ SHM_AXIS_VELOCITY ] = (float) controlMeasuresList[ CONTROL_VELOCITY ];
      //controlValuesList[ SHM_AXIS_ACCELERATION ] = (float) controlMeasuresList[ CONTROL_ACCELERATION ];
      controlValuesList[ SHM_AXIS_FORCE ] = (float) controlMeasuresList[ CONTROL_FORCE ];
      SHMControl.SetNumericValuesList( sharedController->sharedData, controlValuesList, 0xFF );

      //DEBUG_PRINT( "measures: p: %.3f - v: %.3f - f: %.3f", controlValuesList[ SHM_CONTROL_POSITION ], controlValuesList[ SHM_CONTROL_VELOCITY ], controlValuesList[ SHM_CONTROL_FORCE ] );
    }
  }

  for( size_t robotIndex = 0; robotIndex < kv_size( robotControllersList ); robotIndex++ )
    RobotControl.Update( kv_A( robotControllersList, robotIndex ) );

  for( size_t jointControllerIndex = 0; jointControllerIndex < kv_size( sharedJointControllersList ); jointControllerIndex++ )
  {
    SHMDoFController sharedController = kv_A( sharedJointControllersList, jointControllerIndex );
    Actuator localJoint = (Actuator) sharedController->ref_localDoF;

    uint8_t command = SHMControl.GetByteValue( sharedController->sharedData, SHM_CONTROL_REMOVE );
    if( command != SHM_CONTROL_BYTE_NULL )
    {
      DEBUG_PRINT( "received joint command: %x", command );

      if( command == SHM_COMMAND_ENABLE ) ActuatorControl.Enable( localJoint );
      else if( command == SHM_COMMAND_DISABLE ) ActuatorControl.Disable( localJoint );
      else if( command == SHM_COMMAND_RESET ) ActuatorControl.Reset( localJoint );
      else if( command == SHM_COMMAND_OFFSET ) ActuatorControl.SetOffset( localJoint );
      else if( command == SHM_COMMAND_CALIBRATE ) ActuatorControl.Calibrate( localJoint );

      if( ActuatorControl.IsEnabled( localJoint ) ) SHMControl.SetByteValue( sharedController->sharedData, SHM_STATE_ENABLED );
    }

    if( ActuatorControl.HasError( localJoint ) ) SHMControl.SetByteValue( sharedController->sharedData, SHM_STATE_ERROR );
    
    double* jointMeasuresList = ActuatorControl.GetMeasuresList( localJoint );
    if( jointMeasuresList != NULL )
    {
      SHMControl.GetNumericValuesList( sharedController->sharedData, controlValuesList, SHM_CONTROL_PEEK );
      controlValuesList[ SHM_JOINT_ANGLE ] = (float) jointMeasuresList[ CONTROL_POSITION ] * 360.0;
      controlValuesList[ SHM_JOINT_ID_TORQUE ] = (float) jointMeasuresList[ CONTROL_FORCE ];
      //DEBUG_PRINT( "angle: %g - torque: %g", controlValuesList[ SHM_JOINT_ANGLE ], controlValuesList[ SHM_JOINT_ID_TORQUE ] );
      SHMControl.SetNumericValuesList( sharedController->sharedData, controlValuesList, 0xFF );
    }
  }
}

#endif // ROBREHAB_CONTROL_H
