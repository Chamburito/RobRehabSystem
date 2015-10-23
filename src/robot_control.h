#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include "actuator_control.h"
#include "robot_mechanics_interface.h"

#include "impedance_control.h"
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
  double measuresList[ CONTROL_VARS_NUMBER ];
  double parametersList[ CONTROL_VARS_NUMBER ];
}
DoFData;

typedef DoFData* DoF;

typedef struct _RobotControllerData
{
  RobotMechanicsInterface mechanism;
  Actuator jointsList;
  DoF axesList;
  size_t axesNumber;
}
RobotControllerData;

typedef RobotControllerData* RobotController;

KHASH_MAP_INIT_INT( RobotControlInt, RobotController )
khash_t( RobotControlInt )* controllersList = NULL;

#define ROBOT_CONTROL_FUNCTIONS( namespace, function_init ) \
        function_init( RobotController, namespace, InitController, const char* ) \
        function_init( void, namespace, EndController, RobotController ) \
        function_init( void, namespace, Enable, RobotController ) \
        function_init( void, namespace, Disable, RobotController ) \
        function_init( void, namespace, Reset, RobotController ) \
        function_init( void, namespace, Calibrate, RobotController ) \
        function_init( bool, namespace, IsEnabled, RobotController ) \
        function_init( bool, namespace, HasError, RobotController ) \
        function_init( bool, namespace, Update, RobotController ) \
        function_init( double*, namespace, GetDoFMeasuresList, RobotController, size_t ) \
        function_init( double*, namespace, GetDoFSetpointsList, RobotController, size_t ) \
        function_init( size_t, namespace, GetDoFsNumber, RobotController ) \
        function_init( const char*, namespace, GetDoFName, RobotController, size_t )

INIT_NAMESPACE_INTERFACE( RobotControl, ROBOT_CONTROL_FUNCTIONS )


static inline RobotController LoadControllerData( const char* );
static inline void UnloadControllerData( RobotController );

const int ROBOT_CONTROLLER_INVALID_ID = -1;

RobotController RobotControl_InitController( const char* configFileName )
{
  DEBUG_EVENT( 0, "Initializing Axis Controller %s on thread %x", configFileName, THREAD_ID );
  
  if( controllersList == NULL ) controllersList = kh_init( RobotControlInt );
  
  int configKey = (int) kh_str_hash_func( configFileName );
  
  int insertionStatus;
  khint_t newControllerIndex = kh_put( RobotControlInt, controllersList, configKey, &insertionStatus );
  if( insertionStatus > 0 )
  {
    kh_value( controllersList, newControllerIndex ) = LoadControllerData( configFileName );
    if( kh_value( controllersList, newControllerIndex ) == NULL )
    {
      RobotControl_EndController( (int) newControllerIndex );
      return ROBOT_CONTROLLER_INVALID_ID;
    }
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
  
  for( size_t jointIndex = 0; jointIndex < controller->axesNumber; jointIndex++ )
  {
    Actuator joint = controller->jointsList + jointIndex;
    
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
  
  for( size_t jointIndex = 0; jointIndex < controller->axesNumber; jointIndex++ )
  {
    Actuator joint = controller->jointsList + jointIndex;
  
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
  
  for( size_t jointIndex = 0; jointIndex < controller->axesNumber; jointIndex++ )
  {
    Actuator joint = controller->jointsList + jointIndex;
    
    DEBUG_PRINT( "reseting controller joint %p", joint );
    ActuatorControl.Reset( joint );
  }
}

inline bool RobotControl_IsEnabled( RobotController controller )
{
  if( controller == NULL ) return false;
    
  return controller->actuator.interface->IsEnabled( controller->actuator.ID );
}

inline bool RobotControl_HasError( RobotController controller )
{
  if( controller == NULL ) return false;
    
  return controller->actuator.interface->HasError( controller->actuator.ID );
}

inline void RobotControl_Calibrate( RobotController controller )
{
  if( controller == NULL ) return;
  
  DEBUG_PRINT( "calibrating controller %p", controller );

  controller->actuator.interface->Calibrate( controller->actuator.ID );
}

inline double* RobotControl_GetMeasuresList( RobotController controller )
{
  if( controller == NULL ) return NULL;
    
  return (double*) controller->measuresList;
}

inline double* RobotControl_GetSetpointsList( RobotController controller )
{
  if( controller == NULL ) return NULL;
    
  return (double*) controller->parametersList;
}

inline void RobotControl_SetSetpoint( RobotController controller, double setpoint )
{
  if( controller == NULL ) return;
    
  controller->setpoint = setpoint;
}

inline void RobotControl_SetImpedance( RobotController controller, double referenceStiffness, double referenceDamping )
{
  if( controller == NULL ) return;

  //Spline3Interp.SetScale( controller->parameterCurvesList[ CONTROL_STIFFNESS ], ( referenceStiffness > 0.0 ) ? referenceStiffness : 0.0 );
  //Spline3Interp.SetScale( controller->parameterCurvesList[ CONTROL_DAMPING ], ( referenceDamping > 0.0 ) ? referenceDamping : 0.0 );
  
  Spline3Interp.SetOffset( controller->parameterCurvesList[ CONTROL_STIFFNESS ], ( referenceStiffness > 0.0 ) ? referenceStiffness : 0.0 );
  Spline3Interp.SetOffset( controller->parameterCurvesList[ CONTROL_DAMPING ], ( referenceDamping > 0.0 ) ? referenceDamping : 0.0 );
}

const enum ActuatorVariables AXIS_OPERATION_MODES[ CONTROL_VARS_NUMBER ] = { AXIS_POSITION, AXIS_VELOCITY, AXIS_FORCE };
inline void RobotControl_SetControlMode( RobotController controller, enum ControlVariables mode )
{
  if( controller == NULL ) return;
  
  if( mode < 0 || mode >= CONTROL_VARS_NUMBER ) return;
    
  controller->actuator.interface->SetOperationMode( controller->actuator.ID, AXIS_OPERATION_MODES[ mode ] );
  controller->controlMode = mode;
}


const char* PARAMETER_NAMES[ CONTROL_VARS_NUMBER ] = { "reference", "stiffness", "damping" };
static inline RobotController LoadControllerData( const char* configFileName )
{
  //static char searchPath[ FILE_PARSER_MAX_PATH_LENGTH ];
  
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
  
  DEBUG_PRINT( "Trying to create axis controller %s", configFileName ); 
  
  newController->actuator.interface = &SEActuatorOperations;
  newController->actuator.ID = newController->actuator.interface->Init( "SEA Teste" );
  newController->positionFilter = SimpleKalman_CreateFilter( 0.0 );
  newController->controlMode = CONTROL_VELOCITY;
  newController->actuator.interface->SetOperationMode( newController->actuator.ID, AXIS_VELOCITY );
  
  newController->parameterCurvesList[ CONTROL_REFERENCE ] = Spline3Interp.LoadCurve( "spline3_curves/knee_walk_position" );
  newController->parameterCurvesList[ CONTROL_STIFFNESS ] = Spline3Interp.LoadCurve( NULL );
  newController->parameterCurvesList[ CONTROL_DAMPING ] = Spline3Interp.LoadCurve( NULL );
  
  newController->isRunning = true;
  newController->ref_RunControl = RunForcePIControl;
  newController->controlThread = Threading.StartThread( AsyncControl, newController, THREAD_JOINABLE );
  
  DEBUG_PRINT( "axis controller %s created", configFileName );
  
  Timing_Delay( 1000 );
  
  newController->actuator.interface->Enable( newController->actuator.ID );
  
  return newController;
}

static inline void UnloadControllerData( RobotController controller )
{
  if( controller != NULL )
  {
    Timing_Delay( 2000 );
    
    // Destroy axis data structures
    controller->isRunning = false;
    
    if( controller->controlThread != -1 )
      Threading.WaitExit( controller->controlThread, 5000 );
    
    DEBUG_PRINT( "ending actuator %d", controller->actuator.ID );
    
    controller->actuator.interface->End( controller->actuator.ID );
    
    DEBUG_PRINT( "discarding filter %p", controller->positionFilter );
    SimpleKalman_DiscardFilter( controller->positionFilter );
      
    for( size_t parameterIndex = 0; parameterIndex < CONTROL_VARS_NUMBER; parameterIndex++ )
    {
      DEBUG_PRINT( "discarding curve %u: %p", parameterIndex, controller->parameterCurvesList[ parameterIndex ] );
      Spline3Interp.UnloadCurve( controller->parameterCurvesList[ parameterIndex ] );
    }
  }
}


/////////////////////////////////////////////////////////////////////////////////
/////                         ASYNCRONOUS CONTROL                           /////
/////////////////////////////////////////////////////////////////////////////////

const double CONTROL_SAMPLING_INTERVAL = 0.005;           // Sampling interval

// Method that runs the control functions asyncronously
static void* AsyncControl( void* args )
{
  RobotController controller = (RobotController) args;
  
  // Try to correct errors
  controller->actuator.interface->Reset( controller->actuator.ID );
  
  unsigned long execTime, elapsedTime;
  
  controller->isRunning = true;
  
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "starting to run control for actuator %d on thread %x", controller->actuator.ID, THREAD_ID );
  
  while( controller->isRunning )
  {
    DEBUG_UPDATE( "running control for actuator %d", controller->actuator.ID );
    
    execTime = Timing_GetExecTimeMilliseconds();
    
    UpdateControlMeasures( controller );
    
    UpdateControlSetpoints( controller );
    
    RunControl( controller );
    
    elapsedTime = Timing_GetExecTimeMilliseconds() - execTime;
    DEBUG_UPDATE( "control pass for actuator %d (before delay): elapsed time: %u ms", controller->actuator.ID, elapsedTime );
    if( elapsedTime < (int) ( 1000 * CONTROL_SAMPLING_INTERVAL ) ) Timing_Delay( 1000 * CONTROL_SAMPLING_INTERVAL - elapsedTime );
    DEBUG_UPDATE( "control pass for actuator %d (after delay): elapsed time: %u ms", controller->actuator.ID, Timing_GetExecTimeMilliseconds() - execTime );
  }
  
  DEBUG_PRINT( "ending control thread %x", THREAD_ID );
   
  return NULL;
}

static inline void UpdateControlMeasures( RobotController controller )
{
  DEBUG_UPDATE( "reading axes from actuator %d", controller->actuator.ID );
  double* measuresList = controller->actuator.interface->ReadAxes( controller->actuator.ID );
  if( measuresList != NULL )
  {
    DEBUG_UPDATE( "filtering measures from actuator %d", controller->actuator.ID );
    double* filteredMeasures = SimpleKalman_Update( controller->positionFilter, measuresList[ AXIS_POSITION ], CONTROL_SAMPLING_INTERVAL );

    controller->measuresList[ CONTROL_POSITION ] = filteredMeasures[ KALMAN_VALUE ];
    controller->measuresList[ CONTROL_VELOCITY ] = filteredMeasures[ KALMAN_DERIVATIVE ];
    controller->measuresList[ CONTROL_ACCELERATION ] = filteredMeasures[ KALMAN_DERIVATIVE_2 ];

    controller->measuresList[ CONTROL_FORCE ] = measuresList[ AXIS_FORCE ];
  
    //DEBUG_PRINT( "measures p: %.3f - v: %.3f - a: %.3f - f: %.3f", controller->measuresList[ CONTROL_POSITION ], controller->measuresList[ CONTROL_VELOCITY ], 
    //                                                               controller->measuresList[ CONTROL_ACCELERATION ], controller->measuresList[ CONTROL_FORCE ] );
  }
}

static inline void UpdateControlSetpoints( RobotController controller )
{
  for( size_t parameterIndex = 0; parameterIndex < CONTROL_VARS_NUMBER; parameterIndex++ )
  {
    DEBUG_UPDATE( "get parameter %u at point %g", parameterIndex, controller->setpoint );
    controller->parametersList[ parameterIndex ] = Spline3Interp.GetValue( controller->parameterCurvesList[ parameterIndex ], controller->setpoint );
    DEBUG_UPDATE( "got parameter %u: %g", parameterIndex, controller->parametersList[ parameterIndex ] );
  }
  
  DEBUG_PRINT( "got parameters: %.5f %.5f", controller->parametersList[ CONTROL_REFERENCE ], controller->parametersList[ CONTROL_STIFFNESS ] );
}

static inline void RunControl( RobotController controller )
{
  // If the motor is being actually controlled, call control pass algorhitm
  
  if( controller->actuator.interface->IsEnabled( controller->actuator.ID ) )
  {
    double* controlOutputsList = controller->ref_RunControl( controller->measuresList, controller->parametersList, CONTROL_SAMPLING_INTERVAL );
    //DEBUG_PRINT( "force: %.3f - control: %.3f", controller->measuresList[ CONTROL_FORCE ], controlOutputsList[ controller->controlMode ] );
    
    controller->actuator.interface->SetSetpoint( controller->actuator.ID, controlOutputsList[ controller->controlMode ] );
  }
}

#endif /* ROBOT_CONTROL_H */ 
