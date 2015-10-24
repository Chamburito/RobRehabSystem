#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include "actuator_control.h"
#include "robot_mechanics_interface.h"

#include "spline3_interpolation.h"

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
  char name[ 32 ];
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
        function_init( bool, namespace, Update, int ) \
        function_init( double*, namespace, GetDoFMeasuresList, int, size_t ) \
        function_init( double*, namespace, GetDoFSetpointsList, int, size_t ) \
        function_init( void, namespace, SetDoFImpedance, int, size_t, double, double ) \
        function_init( size_t, namespace, GetDoFsNumber, int ) \
        function_init( const char*, namespace, GetDoFName, int, size_t )

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

inline bool RobotControl_Update( int controllerID )
{
  khint_t controllerIndex = kh_get( RobotControlInt, controllersList, (khint_t) controllerID );
  if( controllerIndex == kh_end( controllersList ) ) return false;
  
  RobotController controller = kh_value( controllersList, controllerIndex );
  
  controller->dofsList[ 0 ].measuresList[ CONTROL_POSITION ] = controller->jointMeasuresTable[ 0 ][ CONTROL_POSITION ];
  controller->dofsList[ 0 ].measuresList[ CONTROL_VELOCITY ] = controller->jointMeasuresTable[ 0 ][ CONTROL_VELOCITY ];
  controller->dofsList[ 0 ].measuresList[ CONTROL_ACCELERATION ] = controller->jointMeasuresTable[ 0 ][ CONTROL_ACCELERATION ];
  controller->dofsList[ 0 ].measuresList[ CONTROL_FORCE ] = controller->jointMeasuresTable[ 0 ][ CONTROL_FORCE ];
  
  controller->jointSetpointsTable[ 0 ][ CONTROL_POSITION ] = controller->dofsList[ 0 ].setpointsList[ CONTROL_POSITION ];
  controller->jointSetpointsTable[ 0 ][ CONTROL_VELOCITY ] = controller->dofsList[ 0 ].setpointsList[ CONTROL_VELOCITY ];
  controller->jointSetpointsTable[ 0 ][ CONTROL_ACCELERATION ] = controller->dofsList[ 0 ].setpointsList[ CONTROL_ACCELERATION ];
  
  double positionError = controller->jointSetpointsTable[ 0 ][ CONTROL_POSITION ] - controller->jointMeasuresTable[ 0 ][ CONTROL_POSITION ];
  controller->jointSetpointsTable[ 0 ][ CONTROL_FORCE ] = controller->dofsList[ 0 ].stiffness * positionError;
  
  /*for( size_t jointIndex = 0; jointIndex < controller->dofsNumber; jointIndex++ )
  {
    Actuator joint = controller->jointsList + jointIndex;
    
    DEBUG_PRINT( "calibrating controller joint %p", joint );
    ActuatorControl.Calibrate( joint );
  }*/
  
  return true;
}

inline double* RobotControl_GetDoFMeasuresList( int controllerID, size_t dofIndex )
{
  khint_t controllerIndex = kh_get( RobotControlInt, controllersList, (khint_t) controllerID );
  if( controllerIndex == kh_end( controllersList ) ) return NULL;
  
  RobotController controller = kh_value( controllersList, controllerIndex );
  
  if( dofIndex >= controller->dofsNumber ) return NULL;
    
  return (double*) controller->dofsList[ dofIndex ].measuresList;
}

inline double* RobotControl_GetDoFSetpointsList( int controllerID, size_t dofIndex )
{
  khint_t controllerIndex = kh_get( RobotControlInt, controllersList, (khint_t) controllerID );
  if( controllerIndex == kh_end( controllersList ) ) return NULL;
  
  RobotController controller = kh_value( controllersList, controllerIndex );
  
  if( dofIndex >= controller->dofsNumber ) return NULL;
    
  return (double*) controller->dofsList[ dofIndex ].setpointsList;
}

inline void RobotControl_SetDoFImpedance( int controllerID, size_t dofIndex, double stiffness, double damping )
{
  khint_t controllerIndex = kh_get( RobotControlInt, controllersList, (khint_t) controllerID );
  if( controllerIndex == kh_end( controllersList ) ) return;
  
  RobotController controller = kh_value( controllersList, controllerIndex );
  
  if( dofIndex >= controller->dofsNumber ) return;
  
  controller->dofsList[ dofIndex ].stiffness = ( stiffness > 0.0 ) ? stiffness : 0.0;
  controller->dofsList[ dofIndex ].damping = ( damping > 0.0 ) ? damping : 0.0;
}

inline const char* RobotControl_GetDoFName( int controllerID, size_t dofIndex )
{
  khint_t controllerIndex = kh_get( RobotControlInt, controllersList, (khint_t) controllerID );
  if( controllerIndex == kh_end( controllersList ) ) return NULL;
  
  RobotController controller = kh_value( controllersList, controllerIndex );
  
  if( dofIndex >= controller->dofsNumber ) return NULL;
  
  return (const char*) controller->dofsList[ dofIndex ].name;
}

inline size_t RobotControl_GetDoFsNumber( int controllerID )
{
  khint_t controllerIndex = kh_get( RobotControlInt, controllersList, (khint_t) controllerID );
  if( controllerIndex == kh_end( controllersList ) ) return 0;
  
  RobotController controller = kh_value( controllersList, controllerIndex );
  
  return controller->dofsNumber;
}


const char* PARAMETER_NAMES[ CONTROL_VARS_NUMBER ] = { "reference", "stiffness", "damping" };
static inline RobotController LoadControllerData( const char* configFileName )
{
  //static char searchPath[ PARSER_MAX_KEY_PATH_LENGTH ];
  
  //bool loadError = false;
  
  RobotController newController = (RobotController) malloc( sizeof(RobotControllerData) );
  memset( newController, 0, sizeof(RobotControllerData) );
  
  /*FileParser parser = JSONParser;
  int configFileID = parser.LoadFile( configFileName );
  if( configFileID != -1 )
  {
    if( (newController->actuator.interface = GetActuatorInterface( parser.GetStringValue( configFileID, "actuator.type" ) )) != NULL )
    {
      if( (newController->actuator.ID = newController->actuator.interface->InitController( parser.GetStringValue( configFileID, "actuator.name" ) )) == -1 ) loadError = true;
    }
    else loadError = true;
    
    if( (newController->ref_RunControl = ImpedanceControl_GetFunction( parser.GetStringValue( configFileID, "control.function" ) )) == NULL ) loadError = true;
    
    for( size_t parameterIndex = 0; parameterIndex < CONTROL_VARS_NUMBER; parameterIndex++ )
    {
      sprintf( searchPath, "control.parameters.%s.normalized_curve", PARAMETER_NAMES[ parameterIndex ] );
      newController->parametersList[ parameterIndex ].normalizedCurve = Spline3Interp.LoadCurve( parser.GetStringValue( configFileID, searchPath ) );
      sprintf( searchPath, "control.parameters.%s.amplitude", PARAMETER_NAMES[ parameterIndex ] );
      newController->parametersList[ parameterIndex ].amplitude = parser.GetRealValue( configFileID, searchPath );
    }
    
    parser.UnloadFile( configFileID );
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
  }*/
  
  DEBUG_PRINT( "Trying to create robot controller %s", configFileName ); 
  
  newController->dofsNumber = 1;
  
  newController->jointsList = (Actuator*) calloc( newController->dofsNumber, sizeof(Actuator) );
  newController->dofsList = (DoF) calloc( newController->dofsNumber, sizeof(DoFData) );
  newController->jointMeasuresTable = (double**) calloc( newController->dofsNumber, sizeof(double*) );
  newController->jointSetpointsTable = (double**) calloc( newController->dofsNumber, sizeof(double*) );
  for( size_t jointIndex = 0; jointIndex < newController->dofsNumber; jointIndex++ )
  {
    newController->jointMeasuresTable[ jointIndex ] = ActuatorControl.GetMeasuresList( newController->jointsList[ jointIndex ] );
    newController->jointSetpointsTable[ jointIndex ] = ActuatorControl.GetSetpointsList( newController->jointsList[ jointIndex ] );
  }
  
  newController->jointsList[ 0 ] = ActuatorControl.InitController( "actuators/knee" );
  
  memset( &(newController->dofsList[ 0 ]), 0, sizeof(DoFData) );
  sprintf( newController->dofsList[ 0 ].name, "Theta" );
  
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
    
  free( controller );

  DEBUG_PRINT( "robot controller %p discarded", controller );
}

#endif /* ROBOT_CONTROL_H */ 
