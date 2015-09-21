#ifndef AXIS_CONTROL_H
#define AXIS_CONTROL_H

#include "actuator/actuator_types.h"

#include "impedance_control.h"
#include "spline3_interpolation.h"

#include "file_parsing/json_parser.h"
#include "klib/khash.h"

#include "filters.h"

#include "debug/async_debug.h"
#include "debug/data_logging.h"

/////////////////////////////////////////////////////////////////////////////////
/////                            CONTROL DEVICES                            /////
/////////////////////////////////////////////////////////////////////////////////

const size_t DEVICE_NAME_MAX_LENGTH = 16;

typedef struct _AxisController
{
  struct { ActuatorInterface interface; int ID; } actuator;
  ImpedanceControlFunction ref_RunControl;                        // Control method definition structure
  Thread_Handle controlThread;                                       // Processing thread handle
  double measuresList[ CONTROL_VARS_NUMBER ];
  SimpleKalmanFilter* positionFilter;
  double parametersList[ CONTROL_PARAMS_NUMBER ];
  Splined3Curve* parameterCurvesList[ CONTROL_PARAMS_NUMBER ];
  double setpoint, maxSetpoint, minSetpoint;
  enum ControlVariables controlMode;
  bool isRunning;                                                   // Is control thread running ?
}
AxisController;

KHASH_MAP_INIT_INT( AxisControlInt, AxisController* )
static khash_t( AxisControlInt )* axisControllersList = NULL;

#define NAMESPACE AxisControl

/*#define FUNCTION_HEADER( rvalue, namespace, name, _VA_ARGS_ ) static rvalue namespace##_##name( _VA_ARGS_ );
#define FUNCTION_POINTER( rvalue, namespace, name, _VA_ARGS_ ) rvalue (*name)( _VA_ARGS_ );
#define FUNCTION_INIT( rvalue, namespace, name, _VA_ARGS_ ) .name = namespace##_##name,*/

#define NAMESPACE_FUNCTIONS( namespace ) \
        NAMESPACE_FUNCTION( int, namespace, InitController, const char* ) \
        NAMESPACE_FUNCTION( void, namespace, EndController, int ) \
        NAMESPACE_FUNCTION( void, namespace, Enable, int ) \
        NAMESPACE_FUNCTION( void, namespace, Disable, int ) \
        NAMESPACE_FUNCTION( void, namespace, Reset, int ) \
        NAMESPACE_FUNCTION( void, namespace, Calibrate, int ) \
        NAMESPACE_FUNCTION( bool, namespace, IsEnabled, int ) \
        NAMESPACE_FUNCTION( bool, namespace, HasError, int ) \
        NAMESPACE_FUNCTION( double*, namespace, GetMeasuresList, int ) \
        NAMESPACE_FUNCTION( double*, namespace, GetSetpointsList, int ) \
        NAMESPACE_FUNCTION( void, namespace, SetImpedance, int, double, double ) \
        NAMESPACE_FUNCTION( void, namespace, SetSetpoint, int, double ) \
        NAMESPACE_FUNCTION( void, namespace, SetControlMode, int, enum ControlMeasures ) \
        NAMESPACE_FUNCTION( size_t, namespace, GetDevicesNumber, void )

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

/*static int AxisControl_InitController( const char* );
static void AxisControl_EndController( int );
static void AxisControl_Enable( int );
static void AxisControl_Disable( int );
static void AxisControl_Reset( int );
static void AxisControl_Calibrate( int );
static bool AxisControl_IsEnabled( int );
static bool AxisControl_HasError( int );
static double* AxisControl_GetMeasuresList( int );
static double* AxisControl_GetSetpointsList( int );
static void AxisControl_SetImpedance( int, double, double );
static void AxisControl_SetSetpoint( int, double );
static size_t AxisControl_GetDevicesNumber();

const struct
{
  int (*InitController)( const char* );
  void (*EndController)( int );
  void (*Enable)( int );
  void (*Disable)( int );
  void (*Reset)( int );
  void (*Calibrate)( int );
  bool (*IsEnabled)( int );
  bool (*HasError)( int );
  double* (*GetMeasuresList)( int );
  double* (*GetSetpointsList)( int );
  void (*SetImpedance)( int, double, double );
  void (*SetSetpoint)( int, double );
  size_t (*GetDevicesNumber)( void );
}
AxisControl = { .InitController = AxisControl_InitController, .EndController = AxisControl_EndController, .Enable = AxisControl_Enable, 
                .Disable = AxisControl_Disable, .Reset = AxisControl_Reset, .Calibrate = AxisControl_Calibrate, .IsEnabled = AxisControl_IsEnabled, 
                .GetMeasuresList = AxisControl_GetMeasuresList, .GetSetpointsList = AxisControl_GetSetpointsList, 
                .SetImpedance = AxisControl_SetImpedance, .SetSetpoint = AxisControl_SetSetpoint, .GetDevicesNumber = AxisControl_GetDevicesNumber };*/

static inline AxisController* LoadControllerData( const char* );
static inline void UnloadControllerData( AxisController* );
                
static void* AsyncControl( void* );
static inline void UpdateControlMeasures( AxisController* );
static inline void UpdateControlSetpoints( AxisController* );
static inline void RunControl( AxisController* );

int AxisControl_InitController( const char* configFileName )
{
  DEBUG_EVENT( 0, "Initializing Axis Controller on thread %x", THREAD_ID );
  
  if( axisControllersList == NULL ) axisControllersList = kh_init( AxisControlInt );
  
  int configKey = (int) kh_str_hash_func( configFileName );
  
  int insertionStatus;
  khint_t newControllerID = kh_put( AxisControlInt, axisControllersList, configKey, &insertionStatus );
  if( insertionStatus > 0 )
  {
    kh_value( axisControllersList, newControllerID ) = LoadControllerData( configFileName );
    if( kh_value( axisControllersList, newControllerID ) == NULL )
    {
      AxisControl_EndController( (int) newControllerID );
      return -1;
    }
  }
  
  Timing_Delay( 2000 );
  
  AxisController* newController = kh_value( axisControllersList, newControllerID );
  
  DEBUG_PRINT( "axis controller %s created (iterator %u)", configFileName, newControllerID );
  
  newController->actuator.interface->SetSetpoint( newController->actuator.ID, 0.0 );
  newController->actuator.interface->Calibrate( newController->actuator.ID );
  //newController->actuator.interface->Enable( newController->actuator.ID );
  
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "Axis Controller %d initialized", (int) newControllerID );
  
  return (int) newControllerID;
}

void AxisControl_EndController( int controllerID )
{
  DEBUG_EVENT( 0, "ending Axis Controller %d", controllerID );

  if( kh_exist( axisControllersList, (khint_t) controllerID ) )
  {
    AxisControl_Disable( controllerID );
    
    UnloadControllerData( kh_value( axisControllersList, (khint_t) controllerID ) );
    
    kh_del( AxisControlInt, axisControllersList, controllerID );
    
    if( kh_size( axisControllersList ) == 0 )
    {
      kh_destroy( AxisControlInt, axisControllersList );
      axisControllersList = NULL;
    }
  }
}

static size_t AxisControl_GetDevicesNumber()
{
  return kh_size( axisControllersList );
}

static void AxisControl_Enable( int controllerID )
{
  if( kh_exist( axisControllersList, (khint_t) controllerID ) )
  {
    AxisController* controller = kh_value( axisControllersList, (khint_t) controllerID );
    
    //controller->actuator.interface->Reset( controller->actuator.ID );
    
    DEBUG_PRINT( "enabling controller %d", controllerID );
    
    if( controller->ref_RunControl != NULL )
     controller->actuator.interface->Enable( controller->actuator.ID );
  
    controller->measuresList[ CONTROL_ERROR ] = 0.0;
    
    DEBUG_PRINT( "controller %d enabled", controllerID );
  }
}

static void AxisControl_Disable( int controllerID )
{
  DEBUG_EVENT( 0, "disabling Axis Controller %d", controllerID );
  
  if( kh_exist( axisControllersList, (khint_t) controllerID ) )
  {
    AxisController* controller = kh_value( axisControllersList, (khint_t) controllerID );
    
    controller->actuator.interface->Disable( controller->actuator.ID );
  
    controller->measuresList[ CONTROL_ERROR ] = 0.0;
  }
}

static void AxisControl_Reset( int controllerID )
{
  if( kh_exist( axisControllersList, (khint_t) controllerID ) )
  {
    AxisController* controller = kh_value( axisControllersList, (khint_t) controllerID );
    
    if( controller->actuator.interface->HasError( controller->actuator.ID ) ) 
      controller->actuator.interface->Reset( controller->actuator.ID );
  
    controller->measuresList[ CONTROL_ERROR ] = 0.0;
  }
}

static bool AxisControl_IsEnabled( int controllerID )
{
  if( kh_exist( axisControllersList, (khint_t) controllerID ) )
  {
    AxisController* controller = kh_value( axisControllersList, (khint_t) controllerID );
    
    return controller->actuator.interface->IsEnabled( controller->actuator.ID );
  }
  
  return false;
}

static bool AxisControl_HasError( int controllerID )
{
  if( kh_exist( axisControllersList, (khint_t) controllerID ) )
  {
    AxisController* controller = kh_value( axisControllersList, (khint_t) controllerID );
    
    return controller->actuator.interface->HasError( controller->actuator.ID );
  }
  
  return false;
}

static void AxisControl_Calibrate( int controllerID )
{
  if( kh_exist( axisControllersList, (khint_t) controllerID ) )
  {
    AxisController* controller = kh_value( axisControllersList, (khint_t) controllerID );

    controller->actuator.interface->Calibrate( controller->actuator.ID );
  }
}

static double* AxisControl_GetMeasuresList( int controllerID )
{
  if( kh_exist( axisControllersList, (khint_t) controllerID ) )
  {
    AxisController* controller = kh_value( axisControllersList, (khint_t) controllerID );
    
    return (double*) controller->measuresList;
  }
  
  return NULL;
}

static double* AxisControl_GetSetpointsList( int controllerID )
{
  if( kh_exist( axisControllersList, (khint_t) controllerID ) )
  {
    AxisController* controller = kh_value( axisControllersList, (khint_t) controllerID );
    
    return (double*) controller->parametersList;
  }
  
  return NULL;
}

static void AxisControl_SetSetpoint( int controllerID, double setpoint )
{
  if( kh_exist( axisControllersList, (khint_t) controllerID ) )
  {
    AxisController* controller = kh_value( axisControllersList, (khint_t) controllerID );
    
    controller->setpoint = setpoint;
    if( controller->setpoint > controller->maxSetpoint ) controller->setpoint = controller->maxSetpoint;
    else if( controller->setpoint < controller->minSetpoint ) controller->setpoint = controller->minSetpoint;
  }
}

static void AxisControl_SetImpedance( int controllerID, double referenceStiffness, double referenceDamping )
{
  if( kh_exist( axisControllersList, (khint_t) controllerID ) )
  {
    AxisController* controller = kh_value( axisControllersList, (khint_t) controllerID );

    Spline3Interp.SetScale( controller->parameterCurvesList[ CONTROL_STIFFNESS ], ( referenceStiffness > 0.0 ) ? referenceStiffness : 0.0 );
    Spline3Interp.SetScale( controller->parameterCurvesList[ CONTROL_DAMPING ], ( referenceDamping > 0.0 ) ? referenceDamping : 0.0 );
  }
}

const enum ActuatorVariables ACTUATOR_OPERATION_MODES[ CONTROL_SETPOINTS_NUMBER ] = { ACTUATOR_POSITION, ACTUATOR_VELOCITY, ACTUATOR_FORCE };
static void AxisControl_SetControlMode( int controllerID, enum ControlVariables mode )
{
  if( !kh_exist( axisControllersList, (khint_t) controllerID ) ) return;
  
  if( mode < 0 || mode >= CONTROL_SETPOINTS_NUMBER ) return;
  
  AxisController* controller = kh_value( axisControllersList, (khint_t) controllerID );
    
  controller->actuator.interface->SetOperationMode( controller->actuator.ID, ACTUATOR_OPERATION_MODES[ mode ] );
  controller->controlMode = mode;
}


const char* PARAMETER_NAMES[ CONTROL_PARAMS_NUMBER ] = { "reference", "stiffness", "damping" };
static inline AxisController* LoadControllerData( const char* configFileName )
{
  //static char searchPath[ FILE_PARSER_MAX_PATH_LENGTH ];
  
  //bool loadError = false;
  
  AxisController* newController = (AxisController*) malloc( sizeof(AxisController) );
  memset( newController, 0, sizeof(AxisController) );
  
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
    
    for( size_t parameterIndex = 0; parameterIndex < CONTROL_PARAMS_NUMBER; parameterIndex++ )
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
  
  newController->actuator.interface = &SEActuatorOperations;
  newController->actuator.ID = newController->actuator.interface->Init( "SEA Teste" );
  newController->positionFilter = SimpleKalman_CreateFilter( 0.0 );
  newController->controlMode = CONTROL_FORCE;
  for( size_t parameterIndex = 0; parameterIndex < CONTROL_PARAMS_NUMBER; parameterIndex++ )
    newController->parameterCurvesList[ parameterIndex ] = Spline3Interp.LoadCurve( NULL );
  
  newController->isRunning = true;
  newController->ref_RunControl = RunDefaultControl;
  newController->controlThread = Thread_Start( AsyncControl, newController, THREAD_JOINABLE );
      
  return newController;
}

static inline void UnloadControllerData( AxisController* controller )
{
  if( controller != NULL )
  {
    Timing_Delay( 2000 );
    
    // Destroy axis data structures
    controller->isRunning = false;
    
    if( controller->controlThread != -1 )
      Thread_WaitExit( controller->controlThread, 5000 );
    
    DEBUG_PRINT( "ending actuator %d", controller->actuator.ID );
    
    controller->actuator.interface->End( controller->actuator.ID );
    
    DEBUG_PRINT( "discarding filter %p", controller->positionFilter );
    SimpleKalman_DiscardFilter( controller->positionFilter );
      
    for( size_t parameterIndex = 0; parameterIndex < CONTROL_PARAMS_NUMBER; parameterIndex++ )
    {
      DEBUG_PRINT( "discarding curve %u: %p", parameterIndex, controller->parametersList[ parameterIndex ].normalizedCurve );
      Spline3Interp.UnloadCurve( controller->parametersList[ parameterIndex ].normalizedCurve );
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
  AxisController* controller = (AxisController*) args;
  
  // Try to correct errors
  controller->actuator.interface->Reset( controller->actuator.ID );
  
  unsigned long execTime, elapsedTime;
  
  controller->isRunning = true;
  
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "starting to run control for actuator %d on thread %x", controller->actuator.ID, THREAD_ID );
  
  while( controller->isRunning )
  {
    //DEBUG_UPDATE( "running control for actuator %d", controller->actuator.ID );
    
    execTime = Timing_GetExecTimeMilliseconds();
    
    UpdateControlMeasures( controller );
    
    UpdateControlParameters( controller );
    
    RunControl( controller );
    
    elapsedTime = Timing_GetExecTimeMilliseconds() - execTime;
    DEBUG_UPDATE( "control pass for actuator %d (before delay): elapsed time: %u ms", controller->actuator.ID, elapsedTime );
    if( elapsedTime < (int) ( 1000 * CONTROL_SAMPLING_INTERVAL ) ) Timing_Delay( 1000 * CONTROL_SAMPLING_INTERVAL - elapsedTime );
    DEBUG_UPDATE( "control pass for actuator %d (after delay): elapsed time: %u ms", controller->actuator.ID, Timing_GetExecTimeMilliseconds() - execTime );
  }
  
  DEBUG_PRINT( "ending control thread %x", THREAD_ID );
   
  Thread_Exit( 0 );
  return NULL;
}

static inline void UpdateControlMeasures( AxisController* controller )
{
  controller->actuator.interface->ReadAxes( controller->actuator.ID );

  double sensorPosition = controller->actuator.interface->GetMeasure( controller->actuator.ID, ACTUATOR_POSITION );

  double* filteredMeasures = SimpleKalman_Update( controller->positionFilter, sensorPosition, CONTROL_SAMPLING_INTERVAL );

  controller->measuresList[ CONTROL_POSITION ] = filteredMeasures[ KALMAN_VALUE ];
  controller->measuresList[ CONTROL_VELOCITY ] = filteredMeasures[ KALMAN_DERIVATIVE ];
  controller->measuresList[ CONTROL_ACCELERATION ] = filteredMeasures[ KALMAN_DERIVATIVE_2 ];

  controller->measuresList[ CONTROL_FORCE ] = controller->actuator.interface->GetMeasure( controller->actuator.ID, ACTUATOR_FORCE );
  
  DEBUG_PRINT( "position: %g", controller->measuresList[ CONTROL_POSITION ] );   
}

static inline void UpdateControlParameters( AxisController* controller )
{
  for( size_t parameterIndex = 0; parameterIndex < CONTROL_PARAMS_NUMBER; parameterIndex++ )
    controller->parametersList[ parameterIndex ] = Spline3Interp.GetValue( controller->parameterCurvesList[ parameterIndex ], controller->setpoint );
}

static inline void RunControl( AxisController* controller )
{
  // If the motor is being actually controlled, call control pass algorhitm
  if( controller->actuator.interface->IsEnabled( controller->actuator.ID ) )
  {
    //double controlOutput = controller->ref_RunControl( controller->measuresList, controller->parametersList, CONTROL_SAMPLING_INTERVAL, controller->controlMode );
    
    double controlOutput = - 0.5 * controller->measuresList[ CONTROL_FORCE ];
    
    //DEBUG_PRINT( "force: %.3f - control: %.3f", controller->measuresList[ CONTROL_FORCE ], controlOutput );
    
    controller->actuator.interface->SetSetpoint( controller->actuator.ID, controlOutput );
  }
}

#endif /* AXIS_CONTROL_H */ 
