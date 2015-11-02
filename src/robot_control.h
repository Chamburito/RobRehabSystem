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

typedef struct _DoFData
{
  double measuresList[ CONTROL_VARS_NUMBER ];
  double setpointsList[ CONTROL_VARS_NUMBER ];
  double stiffness, damping;
}
DoFData;

typedef DoFData* DoF;

typedef struct _RobotControllerData
{
  RobotMechanicsInterface mechanism;
  Actuator* jointsList;
  double** jointMeasuresTable;
  double** jointSetpointsTable;
  DoF dofsList;
  double** dofMeasuresTable;
  double** dofSetpointsTable;
  size_t dofsNumber;
}
RobotControllerData;

typedef RobotControllerData* RobotController;

KHASH_MAP_INIT_INT( RobotControlInt, RobotController )
khash_t( RobotControlInt )* controllersList = NULL;


#define ROBOT_CONTROL_FUNCTIONS( namespace, function_init ) \
        function_init( int, namespace, InitController, const char* ) \
        function_init( void, namespace, EndController, int ) \
        function_init( void, namespace, Enable, int ) \
        function_init( void, namespace, Disable, int ) \
        function_init( void, namespace, Reset, int ) \
        function_init( void, namespace, Calibrate, int ) \
        function_init( bool, namespace, IsEnabled, int ) \
        function_init( bool, namespace, HasError, int ) \
        function_init( double*, namespace, GetJointMeasuresList, int, size_t ) \
        function_init( double*, namespace, GetDoFMeasuresList, int, size_t ) \
        function_init( double*, namespace, GetDoFSetpointsList, int, size_t ) \
        function_init( bool, namespace, SetDoFSetpoints, int, double* ) \
        function_init( bool, namespace, SetDoFImpedance, int, size_t, double, double ) \
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

inline void RobotControl_Enable( int controllerID )
{
  khint_t controllerIndex = kh_get( RobotControlInt, controllersList, (khint_t) controllerID );
  if( controllerIndex == kh_end( controllersList ) ) return;
  
  RobotController controller = kh_value( controllersList, controllerIndex );
  
  for( size_t jointIndex = 0; jointIndex < controller->dofsNumber; jointIndex++ )
  {
    Actuator joint = controller->jointsList[ jointIndex ];
    
    ActuatorControl.Reset( joint );
  
    if( !( ActuatorControl.IsEnabled( joint ) ) )
    {
      DEBUG_PRINT( "enabling controller joint %p", joint );
      ActuatorControl.Enable( joint );
    }
  }
}

inline void RobotControl_Disable( int controllerID )
{
  khint_t controllerIndex = kh_get( RobotControlInt, controllersList, (khint_t) controllerID );
  if( controllerIndex == kh_end( controllersList ) ) return;
  
  RobotController controller = kh_value( controllersList, controllerIndex );
  
  for( size_t jointIndex = 0; jointIndex < controller->dofsNumber; jointIndex++ )
  {
    Actuator joint = controller->jointsList[ jointIndex ];
  
    if( ActuatorControl.IsEnabled( joint ) )
    {
      DEBUG_PRINT( "disabling controller joint %p", joint );
      ActuatorControl.Disable( joint );
    }
  }
}

inline void RobotControl_Reset( int controllerID )
{
  khint_t controllerIndex = kh_get( RobotControlInt, controllersList, (khint_t) controllerID );
  if( controllerIndex == kh_end( controllersList ) ) return;
  
  RobotController controller = kh_value( controllersList, controllerIndex );
  
  for( size_t jointIndex = 0; jointIndex < controller->dofsNumber; jointIndex++ )
  {
    Actuator joint = controller->jointsList[ jointIndex ];
    
    DEBUG_PRINT( "reseting controller joint %p", joint );
    ActuatorControl.Reset( joint );
  }
}

inline bool RobotControl_IsEnabled( int controllerID )
{
  khint_t controllerIndex = kh_get( RobotControlInt, controllersList, (khint_t) controllerID );
  if( controllerIndex == kh_end( controllersList ) ) return false;
  
  RobotController controller = kh_value( controllersList, controllerIndex );
  
  bool isEnabled = true;
  for( size_t jointIndex = 0; jointIndex < controller->dofsNumber; jointIndex++ )
  {
    Actuator joint = controller->jointsList[ jointIndex ];
    if( !( ActuatorControl.IsEnabled( joint ) ) ) isEnabled = false;
  }
  
  return isEnabled;
}

inline bool RobotControl_HasError( int controllerID )
{
  khint_t controllerIndex = kh_get( RobotControlInt, controllersList, (khint_t) controllerID );
  if( controllerIndex == kh_end( controllersList ) ) return false;
  
  RobotController controller = kh_value( controllersList, controllerIndex );
  
  bool hasError = false;
  for( size_t jointIndex = 0; jointIndex < controller->dofsNumber; jointIndex++ )
  {
    Actuator joint = controller->jointsList[ jointIndex ];
    if( !( ActuatorControl.HasError( joint ) ) ) hasError = true;
  }
  
  return hasError;
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
}

inline double* RobotControl_GetJointMeasuresList( int controllerID, size_t jointIndex )
{
  khint_t controllerIndex = kh_get( RobotControlInt, controllersList, (khint_t) controllerID );
  if( controllerIndex == kh_end( controllersList ) ) return NULL;
  
  RobotController controller = kh_value( controllersList, controllerIndex );
  
  if( jointIndex >= controller->dofsNumber ) return NULL;
    
  return controller->jointMeasuresTable[ jointIndex ];
}

inline double* RobotControl_GetDoFMeasuresList( int controllerID, size_t dofIndex )
{
  khint_t controllerIndex = kh_get( RobotControlInt, controllersList, (khint_t) controllerID );
  if( controllerIndex == kh_end( controllersList ) ) return NULL;
  
  RobotController controller = kh_value( controllersList, controllerIndex );
  
  if( dofIndex >= controller->dofsNumber ) return NULL;
  
  controller->mechanism.GetForwardDynamics( controller->jointMeasuresTable, controller->dofMeasuresTable[ dofIndex ], dofIndex );
    
  return controller->dofMeasuresTable[ dofIndex ];
}

inline double* RobotControl_GetDoFSetpointsList( int controllerID, size_t dofIndex )
{
  khint_t controllerIndex = kh_get( RobotControlInt, controllersList, (khint_t) controllerID );
  if( controllerIndex == kh_end( controllersList ) ) return NULL;
  
  RobotController controller = kh_value( controllersList, controllerIndex );
  
  if( dofIndex >= controller->dofsNumber ) return NULL;
    
  return controller->dofSetpointsTable[ dofIndex ];
}

inline bool RobotControl_SetDoFSetpoints( int controllerID, size_t dofIndex, double* setpointsList )
{
  khint_t controllerIndex = kh_get( RobotControlInt, controllersList, (khint_t) controllerID );
  if( controllerIndex == kh_end( controllersList ) ) return false;
  
  RobotController controller = kh_value( controllersList, controllerIndex );
  
  if( dofIndex >= controller->dofsNumber ) return false;
  
  controller->mechanism.GetForwardDynamics( controller->jointMeasuresTable, controller->dofMeasuresTable[ dofIndex ], dofIndex );
  memcpy( controller->dofSetpointsTable[ dofIndex ], setpointsList, CONTROL_VARS_NUMBER * sizeof(double) );
  double positionError = controller->dofSetpointsTable[ dofIndex ][ CONTROL_POSITION ] - controller->dofMeasuresTable[ dofIndex ][ CONTROL_POSITION ];
  controller->dofSetpointsTable[ dofIndex ][ CONTROL_FORCE ] = controller->dofsList[ dofIndex ].stiffness * positionError;
  for( size_t jointIndex = 0; jointIndex < controller->dofsNumber; jointIndex++ )
    controller->mechanism.GetInverseDynamics( controller->dofSetpointsTable, controller->jointSetpointsTable[ jointIndex ], jointIndex );
  
  return true;
}

inline bool RobotControl_SetDoFImpedance( int controllerID, size_t dofIndex, double stiffness, double damping )
{
  khint_t controllerIndex = kh_get( RobotControlInt, controllersList, (khint_t) controllerID );
  if( controllerIndex == kh_end( controllersList ) ) return false;
  
  RobotController controller = kh_value( controllersList, controllerIndex );
  
  if( dofIndex >= controller->dofsNumber ) return false;
  
  controller->dofsList[ dofIndex ].stiffness = ( stiffness > 0.0 ) ? stiffness : 0.0;
  controller->dofsList[ dofIndex ].damping = ( damping > 0.0 ) ? damping : 0.0;
  
  if( stiffness < 0.0 || damping < 0.0 ) return false;
  
  return true
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
  static char searchPath[ PARSER_MAX_KEY_PATH_LENGTH ];
  
  bool loadError = false;
  
  DEBUG_PRINT( "Trying to create robot controller %s", configFileName );
  
  RobotController newController = (RobotController) malloc( sizeof(RobotControllerData) );
  memset( newController, 0, sizeof(RobotControllerData) );
  
  int configFileID = ConfigParser.LoadFileData( configFileName );
  if( configFileID != PARSED_DATA_INVALID_ID )
  {
    bool pluginLoaded = false;
    GET_PLUGIN_INTERFACE( ROBOT_MECHANICS_FUNCTIONS, ConfigParser.GetStringValue( configFileID, "mechanics", "" ), newController->mechanism, pluginLoaded );
    if( pluginLoaded )
    {
      const char** DOF_NAMES_LIST = newController->mechanism.GetDoFsList( &(newController->dofsNumber) );
      
      newController->jointsList = (Actuator*) calloc( newController->dofsNumber, sizeof(Actuator) );
      newController->dofsList = (DoF) calloc( newController->dofsNumber, sizeof(DoFData) );
      newController->jointMeasuresTable = (double**) calloc( newController->dofsNumber, sizeof(double*) );
      newController->jointSetpointsTable = (double**) calloc( newController->dofsNumber, sizeof(double*) );
      newController->dofMeasuresTable = (double**) calloc( newController->dofsNumber, sizeof(double*) );
      newController->dofSetpointsTable = (double**) calloc( newController->dofsNumber, sizeof(double*) );
      
      for( size_t jointIndex = 0; jointIndex < newController->dofsNumber; jointIndex++ )
      {
        sprintf( searchPath, "joints.%lu", jointIndex );
        newController->jointsList[ jointIndex ] = ActuatorControl.InitController( ConfigParser.GetStringValue( configFileID, searchPath, "" ) );
        if( newController->jointsList[ jointIndex ] != NULL )
        {
          newController->jointMeasuresTable[ jointIndex ] = ActuatorControl.GetMeasuresList( newController->jointsList[ jointIndex ] );
          newController->jointSetpointsTable[ jointIndex ] = ActuatorControl.GetSetpointsList( newController->jointsList[ jointIndex ] );
        }
        else loadError = true;
      }
      
      for( size_t dofIndex = 0; dofIndex < newController->dofsNumber; dofIndex++ )
      {
        memset( &(newController->dofsList[ dofIndex ]), 0, sizeof(DoFData) );
        sprintf( newController->dofsList[ dofIndex ].name, DOF_NAMES_LIST[ dofIndex ] );
        newController->dofMeasuresTable[ dofIndex ] = (double*) newController->dofsList[ dofIndex ].measuresList;
        newController->dofSetpointsTable[ dofIndex ] = (double*) newController->dofsList[ dofIndex ].setpointsList;
      }
    }
    else loadError = true;
    
    ConfigParser.UnloadData( configFileID );
  }
  else
  {
    loadError = true;
    DEBUG_PRINT( "configuration file for controller %s not found", configFileName );
  }
  
  if( loadError )
  {
    UnloadControllerData( newController );
    return NULL;
  }
  
  DEBUG_PRINT( "robot controller %s created", configFileName );
  
  return newController;
}

static inline void UnloadControllerData( RobotController controller )
{
  if( controller == NULL ) return;
    
  DEBUG_PRINT( "ending robot controller %p", controller );

  free( controller->jointsList );
  free( controller->dofsList );
  free( controller->jointMeasuresTable );
  free( controller->jointSetpointsTable );
  free( controller->dofMeasuresTable );
  free( controller->dofSetpointsTable );
    
  free( controller );

  DEBUG_PRINT( "robot controller %p discarded", controller );
}

#endif /* ROBOT_CONTROL_H */ 
