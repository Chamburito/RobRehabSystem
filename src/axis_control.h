#ifndef AXIS_CONTROL_H
#define AXIS_CONTROL_H

#include "impedance_control.h"
#include "spline3_interpolation.h"

#include "aes_actuator/actuator.h"

#include "async_debug.h"

#include "filters.h"

#include "data_logging.h"

#include "utils/file_parsing/json_parser.h"

/////////////////////////////////////////////////////////////////////////////////
/////                            CONTROL DEVICES                            /////
/////////////////////////////////////////////////////////////////////////////////

typedef struct _AxisController
{
  ActuatorInterface* actuator;
  int deviceID;
  ImpedanceControlFunction ref_RunControl;                        // Control method definition structure
  Thread_Handle controlThread;                                       // Processing thread handle
  double parametersList[ CONTROL_PARAMS_NUMBER ];
  Splined3Curve* parameterCurvesList[ CONTROL_PARAMS_NUMBER ];
  double setpoint;
  double maxReach, minReach;
  double maxStiffness, maxDamping;
  bool isRunning;                                                   // Is control thread running ?
}
AxisController;

KHASH_MAP_INIT_INT( Control, AxisController )
static khash_t( Control )* controllersList = NULL;

static void* AsyncControl( void* );

static int InitController( const char* );
static void EndController( int );
static size_t GetControllersNumber();
static void EnableActuator( int );
static void DisableActuator( int );
static void ResetActuator( int );
static double* GetControllerMeasures( int );
static double* GetControllerParameters( int );
static void SetControllerSetpoint( int );
static void SetControllerImpedance( int );
static void SetControllerOffset( int );

const struct 
{
  int (*InitController)( const char* );
  void (*EndController)( int );
  size_t (*GetControllersNumber)();
  void (*EnableActuator)( int );
  void (*DisableActuator)( int );
  void (*ResetActuator)( int );
  double* (*GetControllerMeasures)( int );
  double* (*GetControllerParameters)( int );
  void (*SetControllerSetpoint)( int );
  void (*SetControllerImpedance)( int );
  void (*SetControllerOffset)( int );
}
AxisControl = { 
                InitController, 
                EndController, 
                GetControllersNumber, 
                EnableActuator, 
                DisableActuator, 
                ResetActuator, 
                GetControllerMeasures, 
                GetControllerParameters, 
                SetControllerSetpoint, 
                SetControllerImpedance, 
                SetControllerOffset 
              };

// Axes configuration loading auxiliary function
static int InitController( const char* deviceName )
{ 
  FileParserInterface parser = JSONParserInterface;
  int configFileID = parser.OpenFile( "axis_controllers" );
  if( configFileID != -1 )
  {
    if( !HasKey( configFileID, deviceName ) ) return -1;
    
    if( controllersList == NULL ) controllersList = kh_init( Control );
    
    int insertionStatus;
    khint_t newControllerID = kh_put( Control, controllersList, (int) deviceName, &insertionStatus );
    
    if( insertionStatus == -1 ) return -1;
    
    AxisController* newController = &(kh_value( controllersList, newControllerID ));
    
    memset( newController, 0, sizeof(AxisController) );
    
    newController->controlThread = -1;
    
    newController->actuator = &AESInterface;
    
    parser.SetBaseKey( configFileID, deviceName );
    
    newController->deviceID = newController->actuator->Init( parser.GetStringValue( configFileID, "actuator" ) );
    
    newController->ref_RunControl = ImpedanceControl_GetFunction( parser.GetStringValue( configFileID, "control_function" ) );
    
    newController->parameterCurvesList[ CONTROL_SETPOINT ] = Spline3Interp_LoadCurve( parser.GetStringValue( configFileID, "parameter_curves.setpoint" ) );
    newController->parameterCurvesList[ CONTROL_STIFFNESS ] = Spline3Interp_LoadCurve( parser.GetStringValue( configFileID, "parameter_curves.stiffness" ) );
    newController->parameterCurvesList[ CONTROL_DAMPING ] = Spline3Interp_LoadCurve( parser.GetStringValue( configFileID, "parameter_curves.damping" ) );
    
    newController->maxReach = parser.GetRealValue( configFileID, "reach.max" );
    newController->minReach = parser.GetRealValue( configFileID, "reach.min" );
    
    parser.CloseFile( configFileID );
    
    newController->controlThread = Thread_Start( AsyncControl, (void*) newController, THREAD_JOINABLE );
    
    if( newController->deviceID == -1 || newController->controlThread == -1 )
    {
      newController->isRunning = false;
      Thread_WaitExit( newController->controlThread, 5000 );
      
      newController->actuator->End( newController->deviceID );
      
      return -1;
    }
    
    return (int) newControllerID;
  }
  
  return -1;
}

// EPOS devices and CAN network shutdown
static void EndController( int controllerID )
{
  if( !kh_exist( controllersList, (khint_t) controllerID ) ) return;
  
  AxisController* controller = &(kh_value( controllersList, (khint_t) controllerID ));
  
  controller->isRunning = false;
    
  if( controller->controlThread != -1 ) Thread_WaitExit( controller->controlThread, 5000 );
    
  controller->actuator->End( controller->deviceID );
      
  for( size_t parameterIndex = 0; parameterIndex < CONTROL_PARAMS_NUMBER; parameterIndex++ )
    Spline3Interp_UnloadCurve( controller->parameterCurvesList[ parameterIndex ] );
  
  kh_del( Control, controllersList, controllerID );
  
  if( kh_size( controllersList ) == 0 ) kh_destroy( Control, controllersList );
  
  DEBUG_EVENT( 0, "axis control ended on thread %x", THREAD_ID );
}

static inline size_t GetDevicesNumber()
{
  if( controllersList == NULL ) return 0;
  
  return kh_size( controllersList );
}

static inline void EnableActuator( int controllerID )
{
  if( !kh_exist( controllersList, (khint_t) controllerID ) ) return;
  
  AxisController* controller = &(kh_value( controllersList, (khint_t) controllerID ));
  
  if( controller->ref_RunControl != NULL )
    controller->actuator->Enable( controller->deviceID );
  
  //controller->measuresList[ CONTROL_ERROR ] = 0.0;
}

static inline void DisableActuator( int controllerID )
{
  if( !kh_exist( controllersList, (khint_t) controllerID ) ) return;
  
  AxisController* controller = &(kh_value( controllersList, (khint_t) controllerID ));
  
  controller->actuator->Disable( controller->deviceID );
  
  //controller->measuresList[ CONTROL_ERROR ] = 0.0;
}

static inline void ResetActuator( int controllerID )
{
  if( !kh_exist( controllersList, (khint_t) controllerID ) ) return;
  
  AxisController* controller = &(kh_value( controllersList, (khint_t) controllerID ));
  
  if( controller->actuator->HasError( controller->deviceID ) )
    controller->actuator->Reset( controller->deviceID );
  
  //controller->measuresList[ CONTROL_ERROR ] = 0.0;
}

extern inline bool* AxisControl_GetMotorStatus( size_t deviceID )
{
  static bool statesList[ AXIS_STATES_NUMBER ];
  
  if( !CheckController( deviceID ) ) return NULL;
  
  for( size_t stateIndex = 0; stateIndex < AXIS_STATES_NUMBER; stateIndex++ )
    statesList[ stateIndex ] = MotorDrive_CheckState( controllersList[ deviceID ].actuator->drive, stateIndex );
  
  return (bool*) statesList;
}

extern inline double* AxisControl_GetMeasuresList( size_t deviceID )
{
  if( !CheckController( deviceID ) ) return NULL;

  return (double*) controllersList[ deviceID ].measuresList;
}

extern inline double* AxisControl_GetParametersList( size_t deviceID )
{
  if( !CheckController( deviceID ) ) return NULL;

  return (double*) controllersList[ deviceID ].parametersList;
}

extern inline void AxisControl_SetSetpoint( size_t deviceID, double setpoint )
{
  if( !CheckController( deviceID ) ) return;
  
  controllersList[ deviceID ].setpoint = setpoint;
}

extern inline void AxisControl_SetImpedance( size_t deviceID, double referenceStiffness, double referenceDamping )
{
  if( !CheckController( deviceID ) ) return;
  
  AxisController* controller = &(controllersList[ deviceID ]);

  controller->maxStiffness = ( referenceStiffness > 0.0 ) ? referenceStiffness : 0.0;
  controller->maxDamping = ( referenceDamping > 0.0 ) ? referenceDamping : 0.0;
}

extern inline void AxisControl_SetOffset( size_t deviceID, double positionOffset )
{
  if( !CheckController( deviceID ) ) return;
  
  AxisController* controller = &(controllersList[ deviceID ]);

  controller->positionOffset = positionOffset;
  controller->deformationOffset = MotorDrive_GetMeasure( controller->sensor, AXIS_POSITION ) - MotorDrive_GetMeasure( controller->actuator->drive, AXIS_POSITION );
  
  DEBUG_PRINT( "offset: %g - %g", controller->positionOffset, controller->deformationOffset );
}

extern inline const char* AxisControl_GetDeviceName( size_t deviceID )
{
  if( !CheckController( deviceID ) ) return NULL;
  
  return (char*) controllersList[ deviceID ].name;
}

/////////////////////////////////////////////////////////////////////////////////
/////                         ASYNCRONOUS CONTROL                           /////
/////////////////////////////////////////////////////////////////////////////////

const double CONTROL_SAMPLING_INTERVAL = 0.005;           // Sampling interval

static inline void UpdateControlMeasures( AxisController* controller )
{
  MotorDrive_ReadValues( controller->sensor );
  if( controller->actuator->drive != controller->sensor ) MotorDrive_ReadValues( controller->actuator->drive );

  double sensorPosition = MotorDrive_GetMeasure( controller->sensor, AXIS_POSITION );

  double* filteredMeasures = SimpleKalman_Update( controller->positionFilter, sensorPosition, CONTROL_SAMPLING_INTERVAL );

  controller->measuresList[ CONTROL_POSITION ] = filteredMeasures[ KALMAN_VALUE ] - controller->positionOffset;
  controller->measuresList[ CONTROL_VELOCITY ] = filteredMeasures[ KALMAN_DERIVATIVE ];
  controller->measuresList[ CONTROL_ACCELERATION ] = filteredMeasures[ KALMAN_DERIVATIVE_2 ];

  double actuatorPosition = MotorDrive_GetMeasure( controller->actuator->drive, AXIS_POSITION );
  double deformation = sensorPosition - actuatorPosition - controller->deformationOffset;
  controller->measuresList[ CONTROL_FORCE ] = Spline3Interp_GetValue( controller->interactionForceCurve, deformation );
  
  //DEBUG_PRINT( "delta: %.3f - torque: %.3f", sensorPosition - actuatorPosition, controller->measuresList[ CONTROL_FORCE ] );   
}

static inline void UpdateControlParameters( AxisController* controller )
{
  double* parametersList = (double*) controller->parametersList;
  Splined3Curve** parameterCurvesList = (Splined3Curve**) controller->parameterCurvesList;
  
  parametersList[ CONTROL_SETPOINT ] = Spline3Interp_GetValue( parameterCurvesList[ CONTROL_SETPOINT ], controller->setpoint );
  if( parametersList[ CONTROL_SETPOINT ] > controller->maxReach ) parametersList[ CONTROL_SETPOINT ] = controller->maxReach;
  else if( parametersList[ CONTROL_SETPOINT ] < controller->minReach ) parametersList[ CONTROL_SETPOINT ] = controller->minReach;
  
  parametersList[ CONTROL_STIFFNESS ] = controller->maxStiffness * Spline3Interp_GetValue( parameterCurvesList[ CONTROL_STIFFNESS ], controller->setpoint );
  if( parametersList[ CONTROL_STIFFNESS ] > controller->maxStiffness ) parametersList[ CONTROL_STIFFNESS ] = controller->maxStiffness;
  else if( parametersList[ CONTROL_STIFFNESS ] < 0.0 ) parametersList[ CONTROL_STIFFNESS ] = 0.0;
  
  parametersList[ CONTROL_DAMPING ] = controller->maxDamping * Spline3Interp_GetValue( parameterCurvesList[ CONTROL_DAMPING ], controller->setpoint );
  if( parametersList[ CONTROL_DAMPING ] > controller->maxDamping ) parametersList[ CONTROL_DAMPING ] = controller->maxDamping;
  else if( parametersList[ CONTROL_DAMPING ] < 0.0 ) parametersList[ CONTROL_DAMPING ] = 0.0;
}

static inline void RunControl( AxisController* controller )
{
  static double controlOutput;
  
  // If the motor is being actually controlled, call control pass algorhitm
  if( Motor_IsActive( controller->actuator ) )
  {
    controlOutput = controller->ref_RunControl( controller->measuresList, controller->parametersList, CONTROL_SAMPLING_INTERVAL );
    
    Motor_SetSetpoint( controller->actuator, controller->operationMode, controlOutput );
  }

  Motor_WriteConfig( controller->actuator );
}

// Method that runs the control functions asyncronously
static void* AsyncControl( void* args )
{
  AxisController* controller = (AxisController*) args;
  
  // Try to correct errors
  if( controller->actuator != NULL ) 
  {
    MotorDrive_Reset( controller->actuator->drive );
    MotorDrive_Reset( controller->sensor );
    
    Motor_SetOperationMode( controller->actuator, controller->operationMode );
  
    unsigned long execTime, elapsedTime;
  
    controller->isRunning = true;
  
    DEBUG_EVENT( 0, "starting to run control for %s AES", controller->name );
  
    while( controller->isRunning )
    {
      DEBUG_UPDATE( "running control for %s AES", controller->name );
    
      execTime = Timing_GetExecTimeMilliseconds();
      
      UpdateControlMeasures( controller );
      
      UpdateControlParameters( controller );
      
      RunControl( controller );
      
      elapsedTime = Timing_GetExecTimeMilliseconds() - execTime;
      DEBUG_UPDATE( "control pass for device %d (before delay): elapsed time: %u ms", controller->deviceID, elapsedTime );
      if( elapsedTime < (int) ( 1000 * CONTROL_SAMPLING_INTERVAL ) ) Timing_Delay( 1000 * CONTROL_SAMPLING_INTERVAL - elapsedTime );
      DEBUG_UPDATE( "control pass for device %d (after delay): elapsed time: %u ms", controller->deviceID, Timing_GetExecTimeMilliseconds() - execTime );
    }
  }
  
  Thread_Exit( 0 );
  return NULL;
}

#endif /* AXIS_CONTROL_H */
