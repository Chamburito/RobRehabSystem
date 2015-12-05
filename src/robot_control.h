#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include "actuator_control.h"
#include "robot_mechanics_interface.h"

#include "config_parser.h"
#include "plugin_loader.h"

#include "klib/khash.h"

#include "debug/async_debug.h"
#include "debug/data_logging.h"

/////////////////////////////////////////////////////////////////////////////////
/////                            CONTROL DEVICE                             /////
/////////////////////////////////////////////////////////////////////////////////

typedef struct _AxisData
{
  double measuresList[ CONTROL_VARS_NUMBER ];
  double setpointsList[ CONTROL_VARS_NUMBER ];
  double stiffness, damping;
}
AxisData;

typedef AxisData* Axis;

typedef struct _RobotControllerData
{
  RobotMechanicsInterface mechanism;
  Actuator* jointsList;
  double** jointMeasuresTable;
  double** jointSetpointsTable;
  Axis axesList;
  double** axisMeasuresTable;
  double** axisSetpointsTable;
  size_t dofsNumber;
}
RobotControllerData;

typedef RobotControllerData* RobotController;

KHASH_MAP_INIT_INT( RobotControlInt, RobotController )
khash_t( RobotControlInt )* controllersList = NULL;


#define ROBOT_CONTROL_FUNCTIONS( namespace, function_init ) \
        function_init( int, namespace, InitController, const char* ) \
        function_init( void, namespace, EndController, int ) \
        function_init( bool, namespace, Update, int ) \
        function_init( Actuator, namespace, GetJoint, int, size_t ) \
        function_init( Axis, namespace, GetAxis, int, size_t ) \
        function_init( double*, namespace, GetAxisMeasuresList, Axis ) \
        function_init( double, namespace, SetAxisSetpoint, Axis, enum ControlVariables, double ) \
        function_init( bool, namespace, SetAxisImpedance, Axis, double, double ) \
        function_init( size_t, namespace, GetDoFsNumber, int )

INIT_NAMESPACE_INTERFACE( RobotControl, ROBOT_CONTROL_FUNCTIONS )


static inline RobotController LoadControllerData( const char* );
static inline void UnloadControllerData( RobotController );

const int ROBOT_CONTROLLER_INVALID_ID = -1;

int RobotControl_InitController( const char* configFileName )
{
  DEBUG_EVENT( 0, "Initializing Axis Controller %s on thread %x", configFileName, THREAD_ID );
  
  if( controllersList == NULL ) controllersList = kh_init( RobotControlInt );
  
  int configKey = (int) kh_str_hash_func( configFileName );
  
  int insertionStatus;
  khint_t newControllerIndex = kh_put( RobotControlInt, controllersList, configKey, &insertionStatus );
  if( insertionStatus <= 0 ) return ROBOT_CONTROLLER_INVALID_ID;

  kh_value( controllersList, newControllerIndex ) = LoadControllerData( configFileName );
  if( kh_value( controllersList, newControllerIndex ) == NULL )
  {
    RobotControl_EndController( (int) newControllerIndex );
    return ROBOT_CONTROLLER_INVALID_ID;
  }
  
  DEBUG_PRINT( "robot controller %s created (iterator %u)", configFileName, newControllerIndex );
  
  return (int) kh_key( controllersList, newControllerIndex );
}

void RobotControl_EndController( int controllerID )
{
  DEBUG_EVENT( 0, "ending Axis Controller %d", controllerID );

  khint_t controllerIndex = kh_get( RobotControlInt, controllersList, (khint_t) controllerID );
  if( controllerIndex == kh_end( controllersList ) ) return;
  
  RobotController controller = kh_value( controllersList, controllerIndex );
  UnloadControllerData( controller );
  
  kh_del( RobotControlInt, controllersList, controllerIndex );
  
  if( kh_size( controllersList ) == 0 )
  {
    kh_destroy( RobotControlInt, controllersList );
    controllersList = NULL;
  }
}

/*inline void RobotControl_SetOffset( int controllerID )
{
  khint_t controllerIndex = kh_get( RobotControlInt, controllersList, (khint_t) controllerID );
  if( controllerIndex == kh_end( controllersList ) ) return;
  
  RobotController controller = kh_value( controllersList, controllerIndex );
  
  for( size_t jointIndex = 0; jointIndex < controller->dofsNumber; jointIndex++ )
  {
    Actuator joint = controller->jointsList[ jointIndex ];
    
    DEBUG_PRINT( "setting offset for controller joint %p", joint );
    ActuatorControl.SetOffset( joint );
  }
}

inline void RobotControl_Calibrate( int controllerID )
{
  khint_t controllerIndex = kh_get( RobotControlInt, controllersList, (khint_t) controllerID );
  if( controllerIndex == kh_end( controllersList ) ) return;
  
  RobotController controller = kh_value( controllersList, controllerIndex );
  
  for( size_t jointIndex = 0; jointIndex < controller->dofsNumber; jointIndex++ )
  {
    Actuator joint = controller->jointsList[ jointIndex ];
    
    DEBUG_PRINT( "calibrating controller joint %p", joint );
    ActuatorControl.Calibrate( joint );
  }
}*/

inline bool RobotControl_Update( int controllerID )
{
  khint_t controllerIndex = kh_get( RobotControlInt, controllersList, (khint_t) controllerID );
  if( controllerIndex == kh_end( controllersList ) ) return false;
  
  RobotController controller = kh_value( controllersList, controllerIndex );
  
  for( size_t axisIndex = 0; axisIndex < controller->dofsNumber; axisIndex++ )
  {
    controller->mechanism.GetForwardDynamics( controller->jointMeasuresTable, controller->axisMeasuresTable[ axisIndex ], axisIndex );
    double positionError = controller->axisSetpointsTable[ axisIndex ][ CONTROL_POSITION ] - controller->axisMeasuresTable[ axisIndex ][ CONTROL_POSITION ];
    controller->axisSetpointsTable[ axisIndex ][ CONTROL_FORCE ] = controller->axesList[ axisIndex ].stiffness * positionError;
  }
  
  //DEBUG_PRINT( "setpoint: %g * (%g - %g)", controller->axesList[ 0 ].stiffness, controller->axisSetpointsTable[ 0 ][ CONTROL_POSITION ], controller->axisMeasuresTable[ 0 ][ CONTROL_POSITION ] ); 
  
  for( size_t jointIndex = 0; jointIndex < controller->dofsNumber; jointIndex++ )
    controller->mechanism.GetInverseDynamics( controller->axisSetpointsTable, controller->jointSetpointsTable[ jointIndex ], jointIndex );
  
  return true;
}

inline double* RobotControl_GetAxisMeasuresList( Axis axis )
{
  if( axis == NULL ) return NULL;
    
  return (double*) axis->measuresList;
}

inline double RobotControl_SetAxisSetpoint( Axis axis, enum ControlVariables variable, double value )
{
  if( axis == NULL ) return 0.0;
  
  if( variable < 0 || variable >= CONTROL_VARS_NUMBER ) return 0.0;
  
  axis->setpointsList[ variable ] = value;
  
  return value;
}

inline bool RobotControl_SetAxisImpedance( Axis axis, double stiffness, double damping )
{
  if( axis == NULL ) return false;
  
  axis->stiffness = ( stiffness > 0.0 ) ? stiffness : 0.0;
  axis->damping = ( damping > 0.0 ) ? damping : 0.0;
  
  if( stiffness < 0.0 || damping < 0.0 ) return false;
  
  return true;
}

inline Actuator RobotControl_GetJoint( int controllerID, size_t jointIndex )
{
  khint_t controllerIndex = kh_get( RobotControlInt, controllersList, (khint_t) controllerID );
  if( controllerIndex == kh_end( controllersList ) ) return NULL;
  
  RobotController controller = kh_value( controllersList, controllerIndex );
  
  if( jointIndex >= controller->dofsNumber ) return NULL;
  
  return controller->jointsList[ jointIndex ];
}

inline Axis RobotControl_GetAxis( int controllerID, size_t axisIndex )
{
  khint_t controllerIndex = kh_get( RobotControlInt, controllersList, (khint_t) controllerID );
  if( controllerIndex == kh_end( controllersList ) ) return NULL;
  
  RobotController controller = kh_value( controllersList, controllerIndex );
  
  if( axisIndex >= controller->dofsNumber ) return NULL;
  
  return &(controller->axesList[ axisIndex ]);
}

inline size_t RobotControl_GetDoFsNumber( int controllerID )
{
  khint_t controllerIndex = kh_get( RobotControlInt, controllersList, (khint_t) controllerID );
  if( controllerIndex == kh_end( controllersList ) ) return 0;
  
  RobotController controller = kh_value( controllersList, controllerIndex );
  
  return controller->dofsNumber;
}


static inline RobotController LoadControllerData( const char* configFileName )
{
  DEBUG_PRINT( "Trying to create robot controller %s", configFileName );
  
  RobotController newController = NULL;
  
  int configFileID = ConfigParsing.LoadConfigFile( configFileName );
  if( configFileID != PARSED_DATA_INVALID_ID )
  {
    ParserInterface parser = ConfigParsing.GetParser();
    
    newController = (RobotController) malloc( sizeof(RobotControllerData) );
    memset( newController, 0, sizeof(RobotControllerData) );
  
    bool loadSuccess = false;
    GET_PLUGIN_INTERFACE( ROBOT_MECHANICS_FUNCTIONS, parser.GetStringValue( configFileID, "", "mechanics" ), newController->mechanism, loadSuccess );
    if( loadSuccess )
    {
      newController->dofsNumber = newController->mechanism.GetDoFsNumber();

      newController->jointsList = (Actuator*) calloc( newController->dofsNumber, sizeof(Actuator) );
      newController->axesList = (Axis) calloc( newController->dofsNumber, sizeof(AxisData) );
      newController->jointMeasuresTable = (double**) calloc( newController->dofsNumber, sizeof(double*) );
      newController->jointSetpointsTable = (double**) calloc( newController->dofsNumber, sizeof(double*) );
      newController->axisMeasuresTable = (double**) calloc( newController->dofsNumber, sizeof(double*) );
      newController->axisSetpointsTable = (double**) calloc( newController->dofsNumber, sizeof(double*) );

      for( size_t jointIndex = 0; jointIndex < newController->dofsNumber; jointIndex++ )
      {
        newController->jointsList[ jointIndex ] = ActuatorControl.InitController( parser.GetStringValue( configFileID, "", "joints.%lu", jointIndex ) );
        if( newController->jointsList[ jointIndex ] != NULL )
        {
          newController->jointMeasuresTable[ jointIndex ] = ActuatorControl.GetMeasuresList( newController->jointsList[ jointIndex ] );
          newController->jointSetpointsTable[ jointIndex ] = ActuatorControl.GetSetpointsList( newController->jointsList[ jointIndex ] );
        }
        else loadSuccess = false;
      }

      for( size_t axisIndex = 0; axisIndex < newController->dofsNumber; axisIndex++ )
      {
        memset( &(newController->axesList[ axisIndex ]), 0, sizeof(AxisData) );
        newController->axisMeasuresTable[ axisIndex ] = (double*) newController->axesList[ axisIndex ].measuresList;
        newController->axisSetpointsTable[ axisIndex ] = (double*) newController->axesList[ axisIndex ].setpointsList;
      }
    }

    parser.UnloadData( configFileID );

    if( !loadSuccess )
    {
      UnloadControllerData( newController );
      return NULL;
    }

    DEBUG_PRINT( "robot controller %s created", configFileName );
  }
  else
    DEBUG_PRINT( "configuration for controller %s is not available", configFileName );
  
  return newController;
}

static inline void UnloadControllerData( RobotController controller )
{
  if( controller == NULL ) return;
    
  DEBUG_PRINT( "ending robot controller %p", controller );
  
  free( controller->jointsList );
  free( controller->axesList );
  free( controller->jointMeasuresTable );
  free( controller->jointSetpointsTable );
  free( controller->axisMeasuresTable );
  free( controller->axisSetpointsTable );
    
  free( controller );

  DEBUG_PRINT( "robot controller %p discarded", controller );
}

#endif /* ROBOT_CONTROL_H */ 
