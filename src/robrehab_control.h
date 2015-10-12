#ifndef ROBREHAB_CONTROL_H
#define ROBREHAB_CONTROL_H

#include "shm_axis_control.h"
#include "axis_control.h"

#include "klib/kvec.h"
#include "klib/khash.h"

#include "file_parsing/json_parser.h"

#include "debug/async_debug.h"

const unsigned long UPDATE_INTERVAL_MS = (unsigned long) CONTROL_SAMPLING_INTERVAL * 1000;

typedef struct _SharedAxis
{
  AxisController controller;
  SHMAxis sharedData;
  double setpointPosition, setpointVelocity;
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
  
  SET_PATH( "config/" );
  
  FileParserOperations parser = JSONParser;
  int configFileID = parser.LoadFile( "shared_axes" );
  if( configFileID != -1 )
  {
    if( parser.HasKey( configFileID, "axes" ) )
    {
      size_t sharedAxesNumber = parser.GetListSize( configFileID, "axes" );
      
      DEBUG_PRINT( "List size: %u", sharedAxesNumber );
      
      char searchPath[ FILE_PARSER_MAX_PATH_LENGTH ];
      for( size_t sharedAxisIndex = 0; sharedAxisIndex < sharedAxesNumber; sharedAxisIndex++ )
      {
        sprintf( searchPath, "axes.%u", sharedAxisIndex );
        char* deviceName = parser.GetStringValue( configFileID, searchPath );
        if( deviceName != NULL )
        {
          AxisController axisController = AxisControl.InitController( deviceName );
          if( axisController != NULL )
          {
            SHMAxis sharedData = SHMAxisControl.InitData( deviceName, SHM_CONTROL_IN );
            if( sharedData != NULL )
            {
              SharedAxisController newAxis = { .sharedData = sharedData, .controller = axisController };
              
              kv_push( SharedAxisController, sharedControllersList, newAxis );
            }
            else
            {
              AxisControl.EndController( axisController );
            }
          }
        }
      }
    }
    
    SET_PATH( ".." );
    
    parser.UnloadFile( configFileID );
  }
  
  //DEBUG_PRINT( "Created axes info string: %s", axesInfoString );
  
  ///*DEBUG_EVENT( 0,*/DEBUG_PRINT( "RobRehab Network initialized on thread %x", THREAD_ID );
  
  return 0;
}

void RobRehabControl_End()
{
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "Ending RobRehab Control on thread %x", THREAD_ID );
  
  for( size_t controllerIndex = 0; controllerIndex < kv_size( sharedControllersList ); controllerIndex++ )
  {
    AxisControl.EndController( kv_A( sharedControllersList, controllerIndex ).controller );
    SHMAxisControl.EndData( kv_A( sharedControllersList, controllerIndex ).sharedData );
    //TrajectoryPlanner_End( kh_value( shmAxisControlsList, shmSHMAxisControlDataID ).trajectoryPlanner );
  }
  
  kv_destroy( sharedControllersList );
  
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "RobRehab Control ended on thread %x", THREAD_ID );
}

void RobRehabControl_Update()
{
  static float controlValuesList[ SHM_CONTROL_FLOATS_NUMBER ];
  
  for( size_t controllerIndex = 0; controllerIndex < kv_size( sharedControllersList ); controllerIndex++ )
  {
    DEBUG_UPDATE( "updating axis controller %u", controllerIndex );
    
    SharedAxisController* sharedController = &(kv_A( sharedControllersList, controllerIndex ));
    SHMAxis sharedData = sharedController->sharedData;
    AxisController axisController = sharedController->controller;
   
    uint8_t command = SHMAxisControl.GetByteValue( sharedData, SHM_REMOVE );
    if( command != SHM_CONTROL_NULL_BYTE )
    {
      //DEBUG_PRINT( "received command: %x", command );
      
      if( command == SHM_COMMAND_ENABLE ) AxisControl.Enable( axisController );
      else if( command == SHM_COMMAND_DISABLE ) AxisControl.Disable( axisController );
      else if( command == SHM_COMMAND_RESET ) AxisControl.Reset( axisController );
      else if( command == SHM_COMMAND_CALIBRATE ) AxisControl.Calibrate( axisController );
      
      sharedData->channelIn->byteValueUpdated = false;
      
      if( AxisControl.IsEnabled( axisController ) ) SHMAxisControl.SetByteValue( sharedData, SHM_STATE_ENABLED );
    }
    
    if( AxisControl.HasError( axisController ) ) SHMAxisControl.SetByteValue( sharedData, SHM_STATE_ERROR );

    uint8_t dataMask = SHMAxisControl.GetNumericValuesList( sharedData, controlValuesList, SHM_REMOVE );
    if( dataMask )
    {
      //DEBUG_PRINT( "setpoints: p: %.3f - v: %.3f - s: %.3f", controlValuesList[ SHM_CONTROL_POSITION ], controlValuesList[ SHM_CONTROL_VELOCITY ], controlValuesList[ SHM_CONTROL_STIFFNESS ] );
      
      if( (dataMask & BIT_INDEX( SHM_CONTROL_POSITION )) ) sharedController->setpointPosition = controlValuesList[ SHM_CONTROL_POSITION ];
      if( (dataMask & BIT_INDEX( SHM_CONTROL_VELOCITY )) ) sharedController->setpointVelocity = controlValuesList[ SHM_CONTROL_VELOCITY ];
      
      if( (dataMask & BIT_INDEX( SHM_CONTROL_STIFFNESS )) | (dataMask & BIT_INDEX( SHM_CONTROL_DAMPING )) )
        AxisControl.SetImpedance( axisController, controlValuesList[ SHM_CONTROL_STIFFNESS ], controlValuesList[ SHM_CONTROL_DAMPING ] );
    }
    
    sharedController->setpointPosition += sharedController->setpointVelocity * CONTROL_SAMPLING_INTERVAL;
    AxisControl.SetSetpoint( axisController, sharedController->setpointPosition );
    
    double* controlMeasuresList = AxisControl.GetMeasuresList( axisController );
    if( controlMeasuresList != NULL )
    {
      controlValuesList[ SHM_CONTROL_POSITION ] = (float) controlMeasuresList[ CONTROL_POSITION ];
      controlValuesList[ SHM_CONTROL_VELOCITY ] = (float) controlMeasuresList[ CONTROL_VELOCITY ];
      controlValuesList[ SHM_CONTROL_ACCELERATION ] = (float) controlMeasuresList[ CONTROL_ACCELERATION ];
      controlValuesList[ SHM_CONTROL_FORCE ] = (float) controlMeasuresList[ CONTROL_FORCE ];
      SHMAxisControl.SetNumericValuesList( sharedData, controlValuesList, 0xFF );
      
      //DEBUG_PRINT( "measures: p: %.3f - v: %.3f - f: %.3f", controlValuesList[ SHM_CONTROL_POSITION ], controlValuesList[ SHM_CONTROL_VELOCITY ], controlValuesList[ SHM_CONTROL_FORCE ] );
    }
  }
}

#endif // ROBREHAB_CONTROL_H
