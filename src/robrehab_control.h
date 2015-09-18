#ifndef SHM_AXIS_CONTROLLER
#define SHM_AXIS_CONTROLLER

#include "shm_axis_control.h"
#include "axis_control.h"

#include "klib/kvec.h"
#include "klib/khash.h"

#include "file_parsing/json_parser.h"

#include "debug/async_debug.h"

typedef struct _SHMAxisController
{
  int sharedDataID;
  int axisID;
}
SHMAxisController;

kvec_t( SHMAxisController ) controllersList;


static int RobRehabControl_Init();
static void RobRehabControl_End();
static void RobRehabControl_Update();

const struct
{
  int (*Init)( void );
  void (*End)( void );
  void (*Update)( void );
}
RobRehabControl = { .Init = RobRehabControl_Init, .End = RobRehabControl_End, .Update = RobRehabControl_Update };

static int RobRehabControl_Init()
{
  int sharedAxisControlDataID, axisControllerID;
  
  kv_init( controllersList );
  
  SET_PATH( "../config/" );
  
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
          if( (axisControllerID = AxisControl.InitController( deviceName )) != -1 )
          {          
            if( (sharedAxisControlDataID = SHMAxisControl.InitControllerData( deviceName )) != -1 )
            {
              SHMAxisController newController = { .sharedDataID = sharedAxisControlDataID, .axisID = axisControllerID };
              
              kv_push( SHMAxisController, controllersList, newController );
            }
            else
            {
              AxisControl.EndController( axisControllerID );
            }
          }
        }
      }
    }
    
    parser.UnloadFile( configFileID );
  }
  
  //DEBUG_PRINT( "Created axes info string: %s", axesInfoString );
  
  ///*DEBUG_EVENT( 0,*/DEBUG_PRINT( "RobRehab Network initialized on thread %x", THREAD_ID );
  
  return 0;
}

static void RobRehabControl_End()
{
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "Ending RobRehab Control on thread %x", THREAD_ID );
  
  for( size_t controllerIndex = 0; controllerIndex < kv_size( controllersList ); controllerIndex++ )
  {
    AxisControl.EndController( kv_A( controllersList, controllerIndex ).axisID );
    SHMAxisControl.EndControllerData( kv_A( controllersList, controllerIndex ).sharedDataID );
    //TrajectoryPlanner_End( kh_value( shmAxisControlsList, shmSHMAxisControllerControlDataID ).trajectoryPlanner );
  }
  
  kv_destroy( controllersList );
  
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "RobRehab Control ended on thread %x", THREAD_ID );
}

static void RobRehabControl_Update()
{
  static bool statesList[ SHM_CONTROL_STATES_NUMBER ];
  static float measuresList[ SHM_CONTROL_MEASURES_NUMBER ];
  
  for( size_t controllerIndex = 0; controllerIndex < kv_size( controllersList ); controllerIndex++ )
  {
    //DEBUG_PRINT( "updating axis controller %u", controllerIndex );
    
    int sharedAxisControlDataID = kv_A( controllersList, controllerIndex ).sharedDataID;
    int axisControllerID = kv_A( controllersList, controllerIndex ).axisID;
    
    bool* commandsList = SHMAxisControl.GetBooleanValuesList( sharedAxisControlDataID, SHM_CONTROL_COMMANDS, SHM_REMOVE, NULL );
    if( commandsList != NULL )
    {
      if( commandsList[ SHM_CONTROL_ENABLE ] ) AxisControl.Enable( axisControllerID );
      if( commandsList[ SHM_CONTROL_DISABLE ] ) AxisControl.Disable( axisControllerID );
      if( commandsList[ SHM_CONTROL_RESET ] ) AxisControl.Reset( axisControllerID );
      if( commandsList[ SHM_CONTROL_CALIBRATE ] ) AxisControl.Calibrate( axisControllerID );
      
      statesList[ SHM_CONTROL_ENABLED ] = AxisControl.IsEnabled( axisControllerID );
      statesList[ SHM_CONTROL_HAS_ERROR ] = false;//AxisControl.HasError( axisControllerID );
      SHMAxisControl.SetBooleanValues( sharedAxisControlDataID, SHM_CONTROL_STATES, statesList );
    }
    
    float* parametersList = SHMAxisControl.GetNumericValuesList( sharedAxisControlDataID, SHM_CONTROL_PARAMETERS, SHM_REMOVE, NULL );
    if( parametersList != NULL )
    {
      AxisControl.SetSetpoint( axisControllerID, parametersList[ SHM_CONTROL_POSITION ] /*+ parametersList[ SHM_CONTROL_VELOCITY ] * 0.005*/ ); // hack
      AxisControl.SetImpedance( axisControllerID, parametersList[ SHM_CONTROL_STIFFNESS ], parametersList[ SHM_CONTROL_DAMPING ] );
    }
    
    double* controlMeasuresList = AxisControl.GetMeasuresList( axisControllerID );
    if( controlMeasuresList != NULL )
    {
      measuresList[ SHM_CONTROL_POSITION ] = (float) controlMeasuresList[ CONTROL_POSITION ];
      measuresList[ SHM_CONTROL_VELOCITY ] = (float) controlMeasuresList[ CONTROL_VELOCITY ];
      measuresList[ SHM_CONTROL_ACCELERATION ] = (float) controlMeasuresList[ CONTROL_ACCELERATION ];
      measuresList[ SHM_CONTROL_FORCE ] = (float) controlMeasuresList[ CONTROL_FORCE ];
      SHMAxisControl.SetNumericValues( sharedAxisControlDataID, SHM_CONTROL_MEASURES, measuresList );
    }
  }
}

#endif // SHM_AXIS_CONTROLLER
