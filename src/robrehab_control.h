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
  int axisID;
  SHMAxisController controller;
}
SharedAxis;

kvec_t( SharedAxis ) sharedAxesList;

/*static int RobRehabControl_Init();
static void RobRehabControl_End();
static void RobRehabControl_Update();

const struct
{
  int (*Init)( void );
  void (*End)( void );
  void (*Update)( void );
}
RobRehabControl = { .Init = RobRehabControl_Init, .End = RobRehabControl_End, .Update = RobRehabControl_Update };*/

#define NAMESPACE RobRehabControl

#define NAMESPACE_FUNCTIONS( namespace ) \
        NAMESPACE_FUNCTION( int, namespace, Init, void ) \
        NAMESPACE_FUNCTION( void, namespace, End, void ) \
        NAMESPACE_FUNCTION( void, namespace, Update, void ) \

#define NAMESPACE_FUNCTION( rvalue, namespace, name, ... ) static rvalue namespace##_##name( __VA_ARGS__ );
NAMESPACE_FUNCTIONS( NAMESPACE )
#undef NAMESPACE_FUNCTION

#define NAMESPACE_FUNCTION( rvalue, namespace, name, ... ) rvalue (*name)( __VA_ARGS__ );
const struct { NAMESPACE_FUNCTIONS( NAMESPACE ) }
#undef NAMESPACE_FUNCTION
#define NAMESPACE_FUNCTION( rvalue, namespace, name, ... ) .name = namespace##_##name,
NAMESPACE = { NAMESPACE_FUNCTIONS( NAMESPACE ) };
#undef NAMESPACE_FUNCTION

#undef NAMESPACE_FUNCTIONS
#undef NAMESPACE

static int RobRehabControl_Init()
{
  kv_init( sharedAxesList );
  
  SET_PATH( "config/" );
  
  FileParser parser = JSONParser;
  int configFileID = parser.LoadFile( "shared_axes" );
  if( configFileID != -1 )
  {
    if( parser.HasKey( configFileID, "axes" ) )
    {
      size_t sharedAxesNumber = parser.GetListSize( configFileID, "axes" );
      
      DEBUG_PRINT( "List size: %u", sharedAxesNumber );
      
      char searchPath[ FILE_PARSER_MAX_PATH_LENGTH ];
      for( size_t sharedAxisDataIndex = 0; sharedAxisDataIndex < sharedAxesNumber; sharedAxisDataIndex++ )
      {
        sprintf( searchPath, "axes.%u", sharedAxisDataIndex );
        char* deviceName = parser.GetStringValue( configFileID, searchPath );
        if( deviceName != NULL )
        {
          int sharedAxisID = AxisControl.InitController( deviceName );
          if( sharedAxisID != -1 )
          {
            SHMAxisController sharedController = SHMAxisControl.InitController( deviceName );
            if( sharedController != NULL )
            {
              SharedAxis newAxis = { .controller = sharedController, .axisID = sharedAxisID };
              
              kv_push( SharedAxis, sharedAxesList, newAxis );
            }
            else
            {
              AxisControl.EndController( sharedAxisID );
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

static void RobRehabControl_End()
{
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "Ending RobRehab Control on thread %x", THREAD_ID );
  
  for( size_t controllerIndex = 0; controllerIndex < kv_size( sharedAxesList ); controllerIndex++ )
  {
    AxisControl.EndController( kv_A( sharedAxesList, controllerIndex ).axisID );
    SHMAxisControl.EndController( kv_A( sharedAxesList, controllerIndex ).controller );
    //TrajectoryPlanner_End( kh_value( shmAxisControlsList, shmSHMAxisControlDataID ).trajectoryPlanner );
  }
  
  kv_destroy( sharedAxesList );
  
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "RobRehab Control ended on thread %x", THREAD_ID );
}

static void RobRehabControl_Update()
{
  static uint8_t state, command, dataMask;
  static float measuresList[ SHM_CONTROL_FLOATS_NUMBER ];
  
  for( size_t controllerIndex = 0; controllerIndex < kv_size( sharedAxesList ); controllerIndex++ )
  {
    //DEBUG_PRINT( "updating axis controller %u", controllerIndex );
    
    SHMAxisController sharedController = kv_A( sharedAxesList, controllerIndex ).controller;
    int sharedAxisID = kv_A( sharedAxesList, controllerIndex ).axisID;
    
    if( SHMAxisControl.GetByteValue( sharedController, SHM_CONTROL_IN, &command, SHM_REMOVE ) )
    {
      if( command == SHM_COMMAND_ENABLE ) AxisControl.Enable( sharedAxisID );
      else if( command == SHM_COMMAND_DISABLE ) AxisControl.Disable( sharedAxisID );
      else if( command == SHM_COMMAND_RESET ) AxisControl.Reset( sharedAxisID );
      else if( command == SHM_COMMAND_CALIBRATE ) AxisControl.Calibrate( sharedAxisID );
      
      if( AxisControl.HasError( sharedAxisID ) ) state = SHM_STATE_ERROR;
      else if( AxisControl.IsEnabled( sharedAxisID ) ) state = SHM_STATE_ENABLED;
      SHMAxisControl.SetByteValue( sharedController, SHM_CONTROL_OUT, state );
    }
    
    float* parametersList = SHMAxisControl.GetNumericValuesList( sharedController, SHM_CONTROL_IN, &dataMask, SHM_REMOVE );
    if( parametersList != NULL )
    {
      if( (dataMask & BIT_INDEX( SHM_CONTROL_POSITION )) | (dataMask & BIT_INDEX( SHM_CONTROL_VELOCITY )) ) 
        AxisControl.SetSetpoint( sharedAxisID, 0.0 /*parametersList[ SHM_CONTROL_POSITION ]*/ /*+ parametersList[ SHM_CONTROL_VELOCITY ] * 0.005*/ ); // hack
      
      if( (dataMask & BIT_INDEX( SHM_CONTROL_STIFFNESS )) | (dataMask & BIT_INDEX( SHM_CONTROL_DAMPING )) )
        AxisControl.SetImpedance( sharedAxisID, parametersList[ SHM_CONTROL_STIFFNESS ], parametersList[ SHM_CONTROL_DAMPING ] );
    }
    
    double* controlMeasuresList = AxisControl.GetMeasuresList( sharedAxisID );
    if( controlMeasuresList != NULL )
    {
      measuresList[ SHM_CONTROL_POSITION ] = (float) controlMeasuresList[ CONTROL_POSITION ];
      measuresList[ SHM_CONTROL_VELOCITY ] = (float) controlMeasuresList[ CONTROL_VELOCITY ];
      measuresList[ SHM_CONTROL_ACCELERATION ] = (float) controlMeasuresList[ CONTROL_ACCELERATION ];
      measuresList[ SHM_CONTROL_FORCE ] = (float) controlMeasuresList[ CONTROL_FORCE ];
      SHMAxisControl.SetNumericValuesList( sharedController, SHM_CONTROL_OUT, measuresList, 0xFFFF );
    }
  }
}

#endif // ROBREHAB_CONTROL_H
