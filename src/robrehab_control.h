#ifndef ROBREHAB_CONTROL_H
#define ROBREHAB_CONTROL_H

#include "shm_axis_control.h"
#include "axis_control.h"

#include "klib/kvec.h"
#include "klib/khash.h"

#include "file_parsing/json_parser.h"

#include "debug/async_debug.h"

typedef struct _SharedAxis
{
  AxisController controller;
  SHMAxis sharedData;
}
SharedAxisController;

kvec_t( SharedAxisController ) sharedControllersList;


#define ROBREHAB_CONTROL_FUNCTIONS( namespace, function_init ) \
        function_init( int, namespace, Init, void ) \
        function_init( void, namespace, End, void ) \
        function_init( void, namespace, Update, void ) \

INIT_NAMESPACE_INTERFACE( RobRehabControl, ROBREHAB_CONTROL_FUNCTIONS )


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
          AxisController sharedController = AxisControl.InitController( deviceName );
          if( sharedController != NULL )
          {
            SHMAxis sharedData = SHMAxisControl.InitData( deviceName );
            if( sharedData != NULL )
            {
              SharedAxisController newAxis = { .sharedData = sharedData, .controller = sharedController };
              
              kv_push( SharedAxisController, sharedControllersList, newAxis );
            }
            else
            {
              AxisControl.EndController( sharedController );
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
  static uint8_t state, command, dataMask;
  static float measuresList[ SHM_CONTROL_FLOATS_NUMBER ];
  
  for( size_t controllerIndex = 0; controllerIndex < kv_size( sharedControllersList ); controllerIndex++ )
  {
    //DEBUG_PRINT( "updating axis controller %u", controllerIndex );
    
    SHMAxis sharedData = kv_A( sharedControllersList, controllerIndex ).sharedData;
    AxisController sharedController = kv_A( sharedControllersList, controllerIndex ).controller;
    
    if( SHMAxisControl.GetByteValue( sharedData, SHM_CONTROL_IN, &command, SHM_REMOVE ) )
    {
      if( command == SHM_COMMAND_ENABLE ) AxisControl.Enable( sharedController );
      else if( command == SHM_COMMAND_DISABLE ) AxisControl.Disable( sharedController );
      else if( command == SHM_COMMAND_RESET ) AxisControl.Reset( sharedController );
      else if( command == SHM_COMMAND_CALIBRATE ) AxisControl.Calibrate( sharedController );
      
      DEBUG_PRINT( "received command: %x", command );
      
      if( AxisControl.HasError( sharedController ) ) state = SHM_STATE_ERROR;
      else if( AxisControl.IsEnabled( sharedController ) ) state = SHM_STATE_ENABLED;
      SHMAxisControl.SetByteValue( sharedData, SHM_CONTROL_OUT, state );
    }
    
    float* parametersList = SHMAxisControl.GetNumericValuesList( sharedData, SHM_CONTROL_IN, &dataMask, SHM_REMOVE );
    if( parametersList != NULL )
    {
      /*if( dataMask )
      {
        DEBUG_PRINT( "setpoints: p: %.3f - v: %.3f - s: %.3f", parametersList[ SHM_CONTROL_POSITION ], 
                                                               parametersList[ SHM_CONTROL_VELOCITY ], parametersList[ SHM_CONTROL_STIFFNESS ] );
      }*/
      
      if( (dataMask & BIT_INDEX( SHM_CONTROL_POSITION )) | (dataMask & BIT_INDEX( SHM_CONTROL_VELOCITY )) ) 
        AxisControl.SetSetpoint( sharedController, parametersList[ SHM_CONTROL_POSITION ] /*+ parametersList[ SHM_CONTROL_VELOCITY ] * 0.005*/ ); // hack
      
      if( (dataMask & BIT_INDEX( SHM_CONTROL_STIFFNESS )) | (dataMask & BIT_INDEX( SHM_CONTROL_DAMPING )) )
        AxisControl.SetImpedance( sharedController, parametersList[ SHM_CONTROL_STIFFNESS ], parametersList[ SHM_CONTROL_DAMPING ] );
    }
    
    double* controlMeasuresList = AxisControl.GetMeasuresList( sharedController );
    if( controlMeasuresList != NULL )
    {
      measuresList[ SHM_CONTROL_POSITION ] = (float) controlMeasuresList[ CONTROL_POSITION ];
      measuresList[ SHM_CONTROL_VELOCITY ] = (float) controlMeasuresList[ CONTROL_VELOCITY ];
      measuresList[ SHM_CONTROL_ACCELERATION ] = (float) controlMeasuresList[ CONTROL_ACCELERATION ];
      measuresList[ SHM_CONTROL_FORCE ] = (float) controlMeasuresList[ CONTROL_FORCE ];
      SHMAxisControl.SetNumericValuesList( sharedData, SHM_CONTROL_OUT, measuresList, 0xFF );
      
      //DEBUG_PRINT( "measures: p: %.3f - v: %.3f - f: %.3f", measuresList[ SHM_CONTROL_POSITION ], measuresList[ SHM_CONTROL_VELOCITY ], measuresList[ SHM_CONTROL_FORCE ] );
    }
  }
}

#endif // ROBREHAB_CONTROL_H
