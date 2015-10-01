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

typedef struct _AxisControllerData
{
  struct { ActuatorInterface interface; int ID; } actuator;
  ImpedanceControlFunction ref_RunControl;                        // Control method definition structure
  Thread controlThread;                                       // Processing thread handle
  double measuresList[ CONTROL_VARS_NUMBER ];
  SimpleKalmanFilter* positionFilter;
  double parametersList[ CONTROL_PARAMS_NUMBER ];
  Splined3Curve parameterCurvesList[ CONTROL_PARAMS_NUMBER ];
  double setpoint;
  enum ControlVariables controlMode;
  bool isRunning;                                                   // Is control thread running ?
}
AxisControllerData;

typedef AxisControllerData* AxisController;


#define AXIS_CONTROL_FUNCTIONS( namespace, function_init ) \
        function_init( AxisController, namespace, InitController, const char* ) \
        function_init( void, namespace, EndController, AxisController ) \
        function_init( void, namespace, Enable, AxisController ) \
        function_init( void, namespace, Disable, AxisController ) \
        function_init( void, namespace, Reset, AxisController ) \
        function_init( void, namespace, Calibrate, AxisController ) \
        function_init( bool, namespace, IsEnabled, AxisController ) \
        function_init( bool, namespace, HasError, AxisController ) \
        function_init( double*, namespace, GetMeasuresList, AxisController ) \
        function_init( double*, namespace, GetSetpointsList, AxisController ) \
        function_init( void, namespace, SetImpedance, AxisController, double, double ) \
        function_init( void, namespace, SetSetpoint, AxisController, double ) \
        function_init( void, namespace, SetControlMode, AxisController, enum ControlVariables )

INIT_NAMESPACE_INTERFACE( AxisControl, AXIS_CONTROL_FUNCTIONS )


static inline AxisController LoadControllerData( const char* );
static inline void UnloadControllerData( AxisController );
                
static void* AsyncControl( void* );
static inline void UpdateControlMeasures( AxisController );
static inline void UpdateControlParameters( AxisController );
static inline void RunControl( AxisController );

AxisController AxisControl_InitController( const char* configFileName )
{
  /*DEBUG_EVENT( 0, "Initializing Axis Controller %s on thread %x", configFileName, THREAD_ID );
  
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
  
  AxisController newController = kh_value( axisControllersList, newControllerID );
  
  DEBUG_PRINT( "axis controller %s created (iterator %u)", configFileName, newControllerID );
  
  newController->actuator.interface->SetSetpoint( newController->actuator.ID, 0.0 );
  newController->actuator.interface->Calibrate( newController->actuator.ID );
  
  DEBUG_PRINT( "Axis Controller %d initialized", (int) newControllerID );
  
  return (int) newControllerID;*/
  
  return LoadControllerData( configFileName );
}

void AxisControl_EndController( /*int controllerID*/AxisController controller )
{
  /*DEBUG_EVENT( 0, "ending Axis Controller %d", controllerID );

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
  }*/
  
  if( controller == NULL ) return;

  Timing_Delay( 2000 );

  // Destroy axis data structures
  controller->isRunning = false;

  if( controller->controlThread != INVALID_THREAD_HANDLE )
    Threading_WaitExit( controller->controlThread, 5000 );

  DEBUG_PRINT( "ending actuator %d", controller->actuator.ID );

  controller->actuator.interface->End( controller->actuator.ID );

  DEBUG_PRINT( "discarding filter %p", controller->positionFilter );
  SimpleKalman_DiscardFilter( controller->positionFilter );

  for( size_t parameterIndex = 0; parameterIndex < CONTROL_PARAMS_NUMBER; parameterIndex++ )
  {
    DEBUG_PRINT( "discarding curve %u: %p", parameterIndex, controller->parameterCurvesList[ parameterIndex ] );
    Spline3Interp.UnloadCurve( controller->parameterCurvesList[ parameterIndex ] );
  }
}

inline void AxisControl_Enable( AxisController controller )
{
  if( controller == NULL ) return;

  //controller->actuator.interface->Reset( controller->actuator.ID );

  if( controller->ref_RunControl != NULL )
    controller->actuator.interface->Enable( controller->actuator.ID );

  controller->measuresList[ CONTROL_ERROR ] = 0.0;
}

inline void AxisControl_Disable( AxisController controller )
{
  if( controller == NULL ) return;
    
  controller->actuator.interface->Disable( controller->actuator.ID );
  
  controller->measuresList[ CONTROL_ERROR ] = 0.0;
}

inline void AxisControl_Reset( AxisController controller )
{
  if( controller == NULL ) return;
    
  if( controller->actuator.interface->HasError( controller->actuator.ID ) ) 
    controller->actuator.interface->Reset( controller->actuator.ID );
  
  controller->measuresList[ CONTROL_ERROR ] = 0.0;
}

inline bool AxisControl_IsEnabled( AxisController controller )
{
  if( controller == NULL ) return false;
    
  return controller->actuator.interface->IsEnabled( controller->actuator.ID );
}

inline bool AxisControl_HasError( AxisController controller )
{
  if( controller == NULL ) return false;
    
  return controller->actuator.interface->HasError( controller->actuator.ID );
}

inline void AxisControl_Calibrate( AxisController controller )
{
  if( controller == NULL ) return;

  controller->actuator.interface->Calibrate( controller->actuator.ID );
}

inline double* AxisControl_GetMeasuresList( AxisController controller )
{
  if( controller == NULL ) return NULL;
    
  return (double*) controller->measuresList;
}

inline double* AxisControl_GetSetpointsList( AxisController controller )
{
  if( controller == NULL ) return NULL;
    
  return (double*) controller->parametersList;
}

inline void AxisControl_SetSetpoint( AxisController controller, double setpoint )
{
  if( controller == NULL ) return;
    
  controller->setpoint = setpoint;
}

inline void AxisControl_SetImpedance( AxisController controller, double referenceStiffness, double referenceDamping )
{
  if( controller == NULL ) return;

  //Spline3Interp.SetScale( controller->parameterCurvesList[ CONTROL_STIFFNESS ], ( referenceStiffness > 0.0 ) ? referenceStiffness : 0.0 );
  //Spline3Interp.SetScale( controller->parameterCurvesList[ CONTROL_DAMPING ], ( referenceDamping > 0.0 ) ? referenceDamping : 0.0 );
  
  Spline3Interp.SetOffset( controller->parameterCurvesList[ CONTROL_STIFFNESS ], ( referenceStiffness > 0.0 ) ? referenceStiffness : 0.0 );
  Spline3Interp.SetOffset( controller->parameterCurvesList[ CONTROL_DAMPING ], ( referenceDamping > 0.0 ) ? referenceDamping : 0.0 );
}

const enum ActuatorVariables AXIS_OPERATION_MODES[ CONTROL_SETPOINTS_NUMBER ] = { AXIS_POSITION, AXIS_VELOCITY, AXIS_FORCE };
inline void AxisControl_SetControlMode( AxisController controller, enum ControlVariables mode )
{
  if( controller == NULL ) return;
  
  if( mode < 0 || mode >= CONTROL_SETPOINTS_NUMBER ) return;
    
  controller->actuator.interface->SetOperationMode( controller->actuator.ID, AXIS_OPERATION_MODES[ mode ] );
  controller->controlMode = mode;
}


const char* PARAMETER_NAMES[ CONTROL_PARAMS_NUMBER ] = { "reference", "stiffness", "damping" };
static inline AxisController LoadControllerData( const char* configFileName )
{
  //static char searchPath[ FILE_PARSER_MAX_PATH_LENGTH ];
  
  //bool loadError = false;
  
  AxisController newController = (AxisController) malloc( sizeof(AxisControllerData) );
  memset( newController, 0, sizeof(AxisControllerData) );
  
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
  newController->controlThread = Threading_StartThread( AsyncControl, newController, THREAD_JOINABLE );
  
  DEBUG_PRINT( "axis controller %s created", configFileName );
  
  Timing_Delay( 1000 );
  
  newController->actuator.interface->Enable( newController->actuator.ID );
  
  return newController;
}

static inline void UnloadControllerData( AxisController controller )
{
  if( controller != NULL )
  {
    Timing_Delay( 2000 );
    
    // Destroy axis data structures
    controller->isRunning = false;
    
    if( controller->controlThread != -1 )
      Threading_WaitExit( controller->controlThread, 5000 );
    
    DEBUG_PRINT( "ending actuator %d", controller->actuator.ID );
    
    controller->actuator.interface->End( controller->actuator.ID );
    
    DEBUG_PRINT( "discarding filter %p", controller->positionFilter );
    SimpleKalman_DiscardFilter( controller->positionFilter );
      
    for( size_t parameterIndex = 0; parameterIndex < CONTROL_PARAMS_NUMBER; parameterIndex++ )
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
  AxisController controller = (AxisController) args;
  
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
    
    UpdateControlParameters( controller );
    
    RunControl( controller );
    
    elapsedTime = Timing_GetExecTimeMilliseconds() - execTime;
    DEBUG_UPDATE( "control pass for actuator %d (before delay): elapsed time: %u ms", controller->actuator.ID, elapsedTime );
    if( elapsedTime < (int) ( 1000 * CONTROL_SAMPLING_INTERVAL ) ) Timing_Delay( 1000 * CONTROL_SAMPLING_INTERVAL - elapsedTime );
    DEBUG_UPDATE( "control pass for actuator %d (after delay): elapsed time: %u ms", controller->actuator.ID, Timing_GetExecTimeMilliseconds() - execTime );
  }
  
  DEBUG_PRINT( "ending control thread %x", THREAD_ID );
   
  return NULL;
}

static inline void UpdateControlMeasures( AxisController controller )
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

static inline void UpdateControlParameters( AxisController controller )
{
  for( size_t parameterIndex = 0; parameterIndex < CONTROL_PARAMS_NUMBER; parameterIndex++ )
  {
    DEBUG_UPDATE( "get parameter %u at point %g", parameterIndex, controller->setpoint );
    controller->parametersList[ parameterIndex ] = Spline3Interp.GetValue( controller->parameterCurvesList[ parameterIndex ], controller->setpoint );
    DEBUG_UPDATE( "got parameter %u: %g", parameterIndex, controller->parametersList[ parameterIndex ] );
  }
  
  //DEBUG_PRINT( "got parameters: %.5f %.5f", controller->parametersList[ CONTROL_REFERENCE ], controller->parametersList[ CONTROL_STIFFNESS ] );
}

static inline void RunControl( AxisController controller )
{
  // If the motor is being actually controlled, call control pass algorhitm
  
  if( controller->actuator.interface->IsEnabled( controller->actuator.ID ) )
  {
    double* controlOutputsList = controller->ref_RunControl( controller->measuresList, controller->parametersList, CONTROL_SAMPLING_INTERVAL );
    //DEBUG_PRINT( "force: %.3f - control: %.3f", controller->measuresList[ CONTROL_FORCE ], controlOutputsList[ controller->controlMode ] );
    
    controller->actuator.interface->SetSetpoint( controller->actuator.ID, controlOutputsList[ controller->controlMode ] );
  }
}

#endif /* AXIS_CONTROL_H */ 
