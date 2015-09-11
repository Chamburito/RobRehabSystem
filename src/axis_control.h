#ifndef AES_CONTROL_H
#define AES_CONTROL_H

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
  double measuresList[ CONTROL_MEASURES_NUMBER ];
  SimpleKalmanFilter* positionFilter;
  double setpointsList[ CONTROL_SETPOINTS_NUMBER ];
  struct {
    Splined3Curve* normalizedCurve;
    double amplitude;
  } parametersList[ CONTROL_SETPOINTS_NUMBER ];
  double setpoint;
  bool isRunning;                                                   // Is control thread running ?
}
AxisController;

KHASH_MAP_INIT_INT( ControlInt, AxisController* )
static khash_t( ControlInt )* controllersList = NULL;

static int InitController( const char* );
static void EndController( int );
static void Enable( int );
static void Disable( int );
static void Reset( int );
static void Calibrate( int );
static bool IsEnabled( int );
static double* GetMeasuresList( int );
static double* GetSetpointsList( int );
static void SetImpedance( int, double, double );
static void SetSetpoint( int, double );
static size_t GetDevicesNumber();

const struct
{
  int (*InitController)( const char* );
  void (*EndController)( int );
  void (*Enable)( int );
  void (*Disable)( int );
  void (*Reset)( int );
  void (*Calibrate)( int );
  bool (*IsEnabled)( int );
  double* (*GetMeasuresList)( int );
  double* (*GetSetpointsList)( int );
  void (*SetImpedance)( int, double, double );
  void (*SetSetpoint)( int, double );
  size_t (*GetDevicesNumber)( void );
}
AxisControl = { .InitController = InitController, .EndController = EndController, .Enable = Enable, .Disable = Disable, .Reset = Reset, .Calibrate = Calibrate, .IsEnabled = IsEnabled, 
                .GetMeasuresList = GetMeasuresList, .GetSetpointsList = GetSetpointsList, .SetImpedance = SetImpedance, .SetSetpoint = SetSetpoint, .GetDevicesNumber = GetDevicesNumber };

static inline AxisController* LoadControllerData( const char* );
static inline void UnloadControllerData( AxisController* );
                
static void* AsyncControl( void* );
static inline void UpdateControlMeasures( AxisController* );
static inline void UpdateControlSetpoints( AxisController* );
static inline void RunControl( AxisController* );

int InitController( const char* configFileName )
{
  DEBUG_EVENT( 0, "Initializing Axis Controller on thread %x", THREAD_ID );
  
  if( controllersList == NULL ) controllersList = kh_init( ControlInt );
  
  int configKey = (int) kh_str_hash_func( configFileName );
  
  int insertionStatus;
  khint_t newControllerID = kh_put( ControlInt, controllersList, configKey, &insertionStatus );
  if( insertionStatus > 0 )
  {
    kh_value( controllersList, newControllerID ) = LoadControllerData( configFileName );
    if( kh_value( controllersList, newControllerID ) == NULL )
    {
      EndController( (int) newControllerID );
      return -1;
    }
  }
  
  Timing_Delay( 2000 );
  
  AxisController* newController = kh_value( controllersList, newControllerID );
  
  DEBUG_PRINT( "axis controller %s created (iterator %u)", configFileName, newControllerID );
  
  //newController->actuator.interface->Calibrate( newController->actuator.ID );
  newController->actuator.interface->Enable( newController->actuator.ID );
  //newController->actuator.interface->SetSetpoint( newController->actuator.ID, 0 );
  
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "Axis Controller %d initialized", (int) newControllerID );
  
  return (int) newControllerID;
}

void EndController( int controllerID )
{
  DEBUG_EVENT( 0, "ending Axis Controller %d", controllerID );

  if( kh_exist( controllersList, controllerID ) )
  {
    UnloadControllerData( kh_value( controllersList, controllerID ) );
    
    kh_del( ControlInt, controllersList, controllerID );
    
    if( kh_size( controllersList ) == 0 )
    {
      kh_destroy( ControlInt, controllersList );
      controllersList = NULL;
    }
  }
}

static size_t GetDevicesNumber()
{
  return kh_size( controllersList );
}

static void Enable( int controllerID )
{
  if( kh_exist( controllersList, controllerID ) )
  {
    AxisController* controller = kh_value( controllersList, controllerID );
    
    //controller->actuator.interface->Reset( controller->actuator.ID );
    
    DEBUG_PRINT( "enabling controller %d", controllerID );
    
    if( controller->ref_RunControl != NULL )
     controller->actuator.interface->Enable( controller->actuator.ID );
  
    controller->measuresList[ CONTROL_ERROR ] = 0.0;
    
    DEBUG_PRINT( "controller %d enabled", controllerID );
  }
}

static void Disable( int controllerID )
{
  if( kh_exist( controllersList, controllerID ) )
  {
    AxisController* controller = kh_value( controllersList, controllerID );
    
    controller->actuator.interface->Disable( controller->actuator.ID );
  
    controller->measuresList[ CONTROL_ERROR ] = 0.0;
  }
}

static void Reset( int controllerID )
{
  if( kh_exist( controllersList, controllerID ) )
  {
    AxisController* controller = kh_value( controllersList, controllerID );
    
    if( controller->actuator.interface->HasError( controller->actuator.ID ) )
      controller->actuator.interface->Reset( controller->actuator.ID );
  
    controller->measuresList[ CONTROL_ERROR ] = 0.0;
  }
}

static bool IsEnabled( int controllerID )
{
  if( kh_exist( controllersList, controllerID ) )
  {
    AxisController* controller = kh_value( controllersList, controllerID );
    
    return controller->actuator.interface->IsEnabled( controller->actuator.ID );
  }
  
  return false;
}

static void Calibrate( int controllerID )
{
  if( kh_exist( controllersList, controllerID ) )
  {
    AxisController* controller = kh_value( controllersList, controllerID );

    controller->actuator.interface->Calibrate( controller->actuator.ID );
  }
}

static double* GetMeasuresList( int controllerID )
{
  if( kh_exist( controllersList, controllerID ) )
  {
    AxisController* controller = kh_value( controllersList, controllerID );
    
    return (double*) controller->measuresList;
  }
  
  return NULL;
}

static double* GetSetpointsList( int controllerID )
{
  if( kh_exist( controllersList, controllerID ) )
  {
    AxisController* controller = kh_value( controllersList, controllerID );
    
    return (double*) controller->setpointsList;
  }
  
  return NULL;
}

static void SetSetpoint( int controllerID, double setpoint )
{
  if( kh_exist( controllersList, controllerID ) )
  {
    AxisController* controller = kh_value( controllersList, controllerID );
    
    controller->setpoint = setpoint;
  }
}

static void SetImpedance( int controllerID, double referenceStiffness, double referenceDamping )
{
  if( kh_exist( controllersList, controllerID ) )
  {
    AxisController* controller = kh_value( controllersList, controllerID );

    controller->parametersList[ CONTROL_STIFFNESS ].amplitude = ( referenceStiffness > 0.0 ) ? referenceStiffness : 0.0;
    controller->parametersList[ CONTROL_DAMPING ].amplitude = ( referenceDamping > 0.0 ) ? referenceDamping : 0.0;
  }
}


const char* PARAMETER_NAMES[ CONTROL_SETPOINTS_NUMBER ] = { "reference", "stiffness", "damping" };
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
    
    for( size_t parameterIndex = 0; parameterIndex < CONTROL_SETPOINTS_NUMBER; parameterIndex++ )
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
  
  newController->isRunning = true;
  newController->controlThread = Thread_Start( AsyncControl, newController, THREAD_JOINABLE );
      
  return newController;
}

static inline void UnloadControllerData( AxisController* controller )
{
  if( controller != NULL )
  {
    Timing_Delay( 2000 );
    
    controller->ref_RunControl = RunDefaultControl;
    
    // Destroy axis data structures
    controller->isRunning = false;
    
    if( controller->controlThread != -1 )
      Thread_WaitExit( controller->controlThread, 5000 );
    
    controller->actuator.interface->End( controller->actuator.ID );
    
    SimpleKalman_DiscardFilter( controller->positionFilter );
      
    for( size_t parameterIndex = 0; parameterIndex < CONTROL_SETPOINTS_NUMBER; parameterIndex++ )
      Spline3Interp.UnloadCurve( controller->parametersList[ parameterIndex ].normalizedCurve );
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
   
  Thread_Exit( 0 );
  return NULL;
}

static inline void UpdateControlMeasures( AxisController* controller )
{
  controller->actuator.interface->ReadAxes( controller->actuator.ID );

  double sensorPosition = controller->actuator.interface->GetMeasure( controller->actuator.ID, AXIS_POSITION );

  double* filteredMeasures = SimpleKalman_Update( controller->positionFilter, sensorPosition, CONTROL_SAMPLING_INTERVAL );

  controller->measuresList[ CONTROL_POSITION ] = filteredMeasures[ KALMAN_VALUE ];
  controller->measuresList[ CONTROL_VELOCITY ] = filteredMeasures[ KALMAN_DERIVATIVE ];
  controller->measuresList[ CONTROL_ACCELERATION ] = filteredMeasures[ KALMAN_DERIVATIVE_2 ];

  controller->measuresList[ CONTROL_FORCE ] = controller->actuator.interface->GetMeasure( controller->actuator.ID, AXIS_FORCE );
  
  //DEBUG_PRINT( "delta: %.3f - torque: %.3f", sensorPosition - actuatorPosition, controller->measuresList[ CONTROL_FORCE ] );   
}

static inline void UpdateControlSetpoints( AxisController* controller )
{
  for( size_t setpointIndex = 0; setpointIndex < CONTROL_SETPOINTS_NUMBER; setpointIndex++ )
  {
    double parameterNormalizedValue = Spline3Interp.GetValue( controller->parametersList[ setpointIndex ].normalizedCurve, controller->setpoint );
    double parameterAmplitude = controller->parametersList[ setpointIndex ].amplitude;
    
    controller->setpointsList[ setpointIndex ] = parameterAmplitude * parameterNormalizedValue;
  }
}

static inline void RunControl( AxisController* controller )
{
  // If the motor is being actually controlled, call control pass algorhitm
  if( controller->actuator.interface->IsEnabled( controller->actuator.ID ) )
  {
    //double controlOutput = controller->ref_RunControl( controller->measuresList, controller->setpointsList, CONTROL_SAMPLING_INTERVAL );
    
    controller->actuator.interface->SetSetpoint( controller->actuator.ID, 0/*controlOutput*/ );
  }
}

#endif /* AES_CONTROL_H */ 
