#ifndef ROBREHAB_CONTROL_H
#define ROBREHAB_CONTROL_H

#include "shm_control.h"
#include "shm_axis_control.h"
#include "robot_control.h"

#include "klib/kvec.h"
#include "klib/khash.h"

#include "config_parser.h"

#include "debug/async_debug.h"

const unsigned long UPDATE_INTERVAL_MS = (unsigned long) CONTROL_SAMPLING_INTERVAL * 1000;

typedef struct _SharedAxis
{
  int robotID;
  size_t controllerIndex;
  SHMController sharedData;
}
SharedAxisController;

kvec_t( SharedAxisController ) sharedControllersList;


#define SUBSYSTEM RobRehabControl

#define ROBREHAB_CONTROL_FUNCTIONS( namespace, function_init ) \
        function_init( int, namespace, Init, void ) \
        function_init( void, namespace, End, void ) \
        function_init( void, namespace, Update, void ) \

INIT_NAMESPACE_INTERFACE( SUBSYSTEM, ROBREHAB_CONTROL_FUNCTIONS )


int RobRehabControl_Init()
{
  kv_init( sharedControllersList );
  
  if( ConfigParser.Init( "JSON" ) )
  {
    int configFileID = ConfigParser.LoadFile( "shared_robots" );
    if( configFileID != -1 )
    {
      if( ConfigParser.HasKey( configFileID, "robots" ) )
      {
        size_t sharedRobotsNumber = ConfigParser.GetListSize( configFileID, "robots" );
        
        DEBUG_PRINT( "List size: %u", sharedRobotsNumber );
        
        char searchPath[ PARSER_MAX_KEY_PATH_LENGTH ];
        for( size_t sharedRobotIndex = 0; sharedRobotIndex < sharedRobotsNumber; sharedRobotIndex++ )
        {
          sprintf( searchPath, "robots.%u", sharedRobotIndex );
          char* deviceName = ConfigParser.GetStringValue( configFileID, searchPath );
          if( deviceName != NULL )
          {
            int robotControllerID = RobotControl.InitController( deviceName );
            if( robotControllerID != ROBOT_CONTROLLER_INVALID_ID )
            {
              size_t controllersNumber = RobotControl.GetDoFsNumber( robotControllerID );
              
              for( size_t controllerIndex = 0; controllerIndex < controllersNumber; controllerIndex++ )
              {
                const char* axisName = RobotControl.GetDoFName( robotControllerID, controllerIndex );
                SHMController sharedData = SHMControl.InitData( axisName, SHM_CONTROL_IN );
                if( sharedData != NULL )
                {
                  SharedAxisController newAxis = { .sharedData = sharedData, .robotID = robotControllerID, .controllerIndex = controllerIndex };
                  kv_push( SharedAxisController, sharedControllersList, newAxis );
                }
              }
            }
          }
        }
      }
    
      ConfigParser.UnloadFile( configFileID );
    }
  }
  
  return 0;
}

void RobRehabControl_End()
{
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "Ending RobRehab Control on thread %x", THREAD_ID );
  
  for( size_t controllerIndex = 0; controllerIndex < kv_size( sharedControllersList ); controllerIndex++ )
  {
    RobotControl.EndController( kv_A( sharedControllersList, controllerIndex ).robotID );
    SHMControl.EndData( kv_A( sharedControllersList, controllerIndex ).sharedData );
  }
  
  kv_destroy( sharedControllersList );
  
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "RobRehab Control ended on thread %x", THREAD_ID );
}

void RobRehabControl_Update()
{
  static float controlValuesList[ SHM_CONTROL_MAX_FLOATS_NUMBER ];
  
  for( size_t controllerIndex = 0; controllerIndex < kv_size( sharedControllersList ); controllerIndex++ )
  {
    DEBUG_UPDATE( "updating axis controller %u", controllerIndex );
    
    SharedAxisController* sharedController = &(kv_A( sharedControllersList, controllerIndex ));
    SHMController sharedData = sharedController->sharedData;
    int robotControllerID = sharedController->robotID;
    size_t axisControllerIndex = sharedController->controllerIndex;
   
    /*uint8_t command = SHMControl.GetByteValue( sharedData, SHM_CONTROL_REMOVE );
    if( command != SHM_CONTROL_NULL_BYTE )
    {
      //DEBUG_PRINT( "received command: %x", command );
      
      if( command == SHM_COMMAND_ENABLE ) AxisControl.Enable( axisController );
      else if( command == SHM_COMMAND_DISABLE ) AxisControl.Disable( axisController );
      else if( command == SHM_COMMAND_RESET ) AxisControl.Reset( axisController );
      else if( command == SHM_COMMAND_CALIBRATE ) AxisControl.Calibrate( axisController );
      
      sharedData->channelIn->byteValueUpdated = false;
      
      if( AxisControl.IsEnabled( axisController ) ) SHMControl.SetByteValue( sharedData, SHM_STATE_ENABLED );
    }
    
    if( AxisControl.HasError( axisController ) ) SHMControl.SetByteValue( sharedData, SHM_STATE_ERROR );*/

    uint8_t dataMask = SHMControl.GetNumericValuesList( sharedData, controlValuesList, SHM_CONTROL_REMOVE );
    if( dataMask )
    {
      double* controlSetpointsList = RobotControl.GetDoFSetpointsList( robotControllerID, axisControllerIndex );
      //DEBUG_PRINT( "setpoints: p: %.3f - v: %.3f - s: %.3f", controlValuesList[ SHM_CONTROL_POSITION ], controlValuesList[ SHM_CONTROL_VELOCITY ], controlValuesList[ SHM_CONTROL_STIFFNESS ] );
      
      if( (dataMask & SHM_CONTROL_BIT_INDEX( SHM_AXIS_POSITION )) ) controlSetpointsList[ CONTROL_POSITION ] = controlValuesList[ SHM_AXIS_POSITION ];
      if( (dataMask & SHM_CONTROL_BIT_INDEX( SHM_AXIS_VELOCITY )) ) controlSetpointsList[ CONTROL_VELOCITY ] = controlValuesList[ SHM_AXIS_VELOCITY ];
      
      if( (dataMask & SHM_CONTROL_BIT_INDEX( SHM_AXIS_STIFFNESS )) | (dataMask & SHM_CONTROL_BIT_INDEX( SHM_AXIS_DAMPING )) )
        RobotControl.SetDoFImpedance( robotControllerID, axisControllerIndex, controlValuesList[ SHM_AXIS_STIFFNESS ], controlValuesList[ SHM_AXIS_DAMPING ] );
    }
    
    double* controlMeasuresList = RobotControl.GetDoFMeasuresList( robotControllerID, axisControllerIndex );
    if( controlMeasuresList != NULL )
    {
      controlValuesList[ SHM_AXIS_POSITION ] = (float) controlMeasuresList[ CONTROL_POSITION ];
      controlValuesList[ SHM_AXIS_VELOCITY ] = (float) controlMeasuresList[ CONTROL_VELOCITY ];
      controlValuesList[ SHM_AXIS_ACCELERATION ] = (float) controlMeasuresList[ CONTROL_ACCELERATION ];
      controlValuesList[ SHM_AXIS_FORCE ] = (float) controlMeasuresList[ CONTROL_FORCE ];
      SHMControl.SetNumericValuesList( sharedData, controlValuesList, 0xFF );
      
      //DEBUG_PRINT( "measures: p: %.3f - v: %.3f - f: %.3f", controlValuesList[ SHM_CONTROL_POSITION ], controlValuesList[ SHM_CONTROL_VELOCITY ], controlValuesList[ SHM_CONTROL_FORCE ] );
    }
  }
}

#endif // ROBREHAB_CONTROL_H
