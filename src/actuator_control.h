#ifndef ACTUATOR_CONTROL_H
#define ACTUATOR_CONTROL_H 

#include "interface.h"

#include "axis/axis_motor.h"
#include "axis/axis_sensor.h"

#include "control_interface.h"
#include "filters.h"

#include "config_parser.h"
#include "plugin_loader.h"

#include "debug/async_debug.h"

typedef struct _ActuatorData
{
  AxisMotor motor;
  AxisSensor encoder, forceSensor;
  KalmanFilter positionFilter, forceFilter;
  double measuresList[ CONTROL_VARS_NUMBER ];
  double setpointsList[ CONTROL_VARS_NUMBER ];
  double controlError;
  ControlInterface control;
  Thread controlThread;
  enum ControlVariables controlMode;
  bool isControlRunning;
}
ActuatorData;

typedef ActuatorData* Actuator;


#define ACTUATOR_FUNCTIONS( namespace, function_init ) \
        function_init( Actuator, namespace, InitController, const char* ) \
        function_init( void, namespace, EndController, Actuator ) \
        function_init( void, namespace, Enable, Actuator ) \
        function_init( void, namespace, Disable, Actuator ) \
        function_init( void, namespace, Reset, Actuator ) \
        function_init( void, namespace, Calibrate, Actuator ) \
        function_init( bool, namespace, IsEnabled, Actuator ) \
        function_init( bool, namespace, HasError, Actuator ) \
        function_init( double*, namespace, GetMeasuresList, Actuator ) \
        function_init( double*, namespace, GetSetpointsList, Actuator ) \
        function_init( void, namespace, SetOperationMode, Actuator, enum ControlVariables )

INIT_NAMESPACE_INTERFACE( ActuatorControl, ACTUATOR_FUNCTIONS )


static void* AsyncControl( void* );

Actuator ActuatorControl_InitController( const char* configFileName )
{
  DEBUG_PRINT( "trying to create series elastic actuator %s", configFileName );
  
  bool loadError = false;
  
  Actuator newActuator = (Actuator) malloc( sizeof(ActuatorData) );
  memset( newActuator, 0, sizeof(ActuatorData) );

  int configFileID = ConfigParser.LoadFileData( configFileName );
  if( configFileID != -1 )
  {
    if( (newActuator->motor = AxisMotors.Init( parser.GetStringValue( configFileID, "motor", "" ) )) == NULL ) loadError = true;
    if( (newActuator->encoder = AxisSensors.Init( parser.GetStringValue( configFileID, "encoder", "" ) )) == NULL ) loadError = true;
    if( (newActuator->forceSensor = AxisSensors.Init( parser.GetStringValue( configFileID, "force_sensor", "" ) )) == NULL ) loadError = true;
    
    bool pluginLoaded;
    GET_PLUGIN_INTERFACE( CONTROL_INTERFACE_FUNCTIONS, ConfigParser.GetStringValue( configFileID, "control_function", "" ), newActuator->control, pluginLoaded );
    if( pluginLoaded ) newActuator->controlThread = Threading.StartThread( AsyncControl, newActuator, THREAD_JOINABLE );
    else loadError = true;
    
    newActuator->positionFilter = SimpleKalman.CreateFilter( 3, 0.0 );
    newActuator->forceFilter = SimpleKalman.CreateFilter( 1, 0.0 );
    
    parser.UnloadData( configFileID );
  }
  else
  {
    loadError = true;
    DEBUG_PRINT( "configuration file for series elastic actuator %s not found", configFileName );
  }
  
  if( loadError )
  {
    ActuatorControl_EndController( newActuator );
    return NULL;
  }
  
  DEBUG_PRINT( "created series elastic actuator %s", configFileName );
  
  return newActuator;
}

void ActuatorControl_EndController( Actuator actuator )
{
  DEBUG_PRINT( "ending actuator %p", actuator );
  
  if( actuator == NULL ) return;
  
  AxisMotors.End( actuator->motor );
  AxisSensors.End( actuator->encoder );
  AxisSensors.End( actuator->forceSensor );
  
  SimpleKalman.DiscardFilter( actuator->positionFilter );
  SimpleKalman.DiscardFilter( actuator->forceFilter );
}

void ActuatorControl_Enable( Actuator actuator )
{
  if( actuator == NULL ) return;
    
  AxisMotors.Enable( actuator->motor );
}

void ActuatorControl_Disable( Actuator actuator )
{
  if( actuator == NULL ) return;
    
  AxisMotors.Disable( actuator->motor );
}

void ActuatorControl_Reset( Actuator actuator )
{
  if( actuator == NULL ) return;
    
  AxisMotors.Reset( actuator->motor );
  AxisSensors.Reset( actuator->encoder );
  AxisSensors.Reset( actuator->forceSensor );
}

void ActuatorControl_Calibrate( Actuator actuator )
{
  if( actuator == NULL ) return;

  AxisMotors.SetOffset( actuator->motor );
  AxisSensors.SetOffset( actuator->encoder );
  AxisSensors.SetOffset( actuator->forceSensor );
}

bool ActuatorControl_IsEnabled( Actuator actuator )
{
  if( actuator == NULL ) return false;
    
  return AxisMotors.IsEnabled( actuator->motor );
}

bool ActuatorControl_HasError( Actuator actuator )
{
  if( actuator == NULL ) return false;
    
  if( AxisMotors.HasError( actuator->motor ) ) return true;
  else if( AxisSensors.HasError( actuator->encoder ) ) return true;
  else if( AxisSensors.HasError( actuator->forceSensor ) ) return true;
  
  return false;
}

double* ActuatorControl_GetMeasuresList( Actuator actuator )
{
  if( actuator == NULL ) return NULL; 
  
  return (double*) actuator->measuresList;
}

double* ActuatorControl_GetSetpointsList( Actuator actuator )
{
  if( actuator == NULL ) return NULL; 
  
  return (double*) actuator->setpointsList;
}

const enum AxisMotorVariables MOTOR_OPERATION_MODES[ CONTROL_VARS_NUMBER ] = { AXIS_POSITION, AXIS_VELOCITY, AXIS_FORCE };
void ActuatorControl_SetOperationMode( Actuator actuator, enum ControlVariables mode )
{
  if( actuator == NULL ) return;
  
  if( mode < 0 || mode >= CONTROL_VARS_NUMBER ) return;
  
  AxisMotors.SetOperationMode( actuator->motor, MOTOR_OPERATION_MODES[ mode ] );
  actuator->controlMode = mode;
}



/////////////////////////////////////////////////////////////////////////////////
/////                         ASYNCRONOUS CONTROL                           /////
/////////////////////////////////////////////////////////////////////////////////

const double CONTROL_SAMPLING_INTERVAL = 0.005;           // Sampling interval

static void UpdateControlMeasures( Actuator );
static void UpdateControlSetpoints( Actuator );
static void RunControl( Actuator );

// Method that runs the control functions asyncronously
static void* AsyncControl( void* data )
{
  Actuator actuator = (Actuator) data;
  
  // Try to correct errors
  ActuatorControl_Reset( actuator );
  
  unsigned long execTime, elapsedTime;
  
  actuator->isControlRunning = true;
  
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "starting to run control for actuator %p on thread %x", actuator, THREAD_ID );
  
  while( actuator->isControlRunning )
  {
    DEBUG_UPDATE( "running control for actuator %p motor %p", actuator, actuator->motor );
    
    execTime = Timing_GetExecTimeMilliseconds();
    
    UpdateControlMeasures( actuator );
    
    UpdateControlSetpoints( actuator );
    
    RunControl( actuator );
    
    elapsedTime = Timing_GetExecTimeMilliseconds() - execTime;
    DEBUG_UPDATE( "control pass for actuator %p (before delay): elapsed time: %u ms", actuator, elapsedTime );
    if( elapsedTime < (int) ( 1000 * CONTROL_SAMPLING_INTERVAL ) ) Timing_Delay( 1000 * CONTROL_SAMPLING_INTERVAL - elapsedTime );
    DEBUG_UPDATE( "control pass for actuator %p (after delay): elapsed time: %u ms", actuator, Timing_GetExecTimeMilliseconds() - execTime );
  }
  
  DEBUG_PRINT( "ending control thread %x", THREAD_ID );
   
  return NULL;
}

static inline void UpdateControlMeasures( Actuator actuator )
{
  DEBUG_UPDATE( "reading axes from actuator %p", actuator );
  double* motorMeasuresList = AxisMotors.ReadMeasures( actuator->motor );
  if( motorMeasuresList != NULL )
  {
    double position = AxisSensors.Read( actuator->encoder, motorMeasuresList );
    double forceMeasure = AxisSensors.Read( actuator->forceSensor, motorMeasuresList );
    
    DEBUG_UPDATE( "filtering measures from actuator %p", actuator );
    double* filteredMeasuresList = SimpleKalman.Update( actuator->positionFilter, position, CONTROL_SAMPLING_INTERVAL );

    actuator->measuresList[ CONTROL_POSITION ] = filteredMeasuresList[ 0 ];
    actuator->measuresList[ CONTROL_VELOCITY ] = filteredMeasuresList[ 1 ];
    //actuator->measuresList[ CONTROL_ACCELERATION ] = filteredMeasuresList[ 2 ];

    filteredMeasuresList = SimpleKalman.Update( actuator->positionFilter, forceMeasure, CONTROL_SAMPLING_INTERVAL );
    actuator->measuresList[ CONTROL_FORCE ] = filteredMeasuresList[ 0 ];
  
    DEBUG_PRINT( "measures p: %.3f - v: %.3f - f: %.3f", actuator->measuresList[ CONTROL_POSITION ], actuator->measuresList[ CONTROL_VELOCITY ], actuator->measuresList[ CONTROL_FORCE ] );
  }
}

static inline void UpdateControlSetpoints( Actuator actuator )
{
  actuator->setpointsList[ CONTROL_POSITION ] += actuator->setpointsList[ CONTROL_VELOCITY ] * CONTROL_SAMPLING_INTERVAL;
  //actuator->setpointsList[ CONTROL_VELOCITY ] += actuator->setpointsList[ CONTROL_ACCELERATION ] * CONTROL_SAMPLING_INTERVAL;
  
  DEBUG_PRINT( "got parameters: %.5f %.5f", actuator->setpointsList[ CONTROL_POSITION ], actuator->setpointsList[ CONTROL_VELOCITY ] );
}

static inline void RunControl( Actuator actuator )
{
  // If the motor is being actually controlled, call control pass algorhitm
  
  if( AxisMotors.IsEnabled( actuator->motor ) )
  {
    double* controlOutputsList = actuator->control.Run( actuator->measuresList, actuator->setpointsList, CONTROL_SAMPLING_INTERVAL, &(actuator->controlError) );
    //DEBUG_PRINT( "force: %.3f - control: %.3f", actuator->measuresList[ CONTROL_FORCE ], controlOutputsList[ actuator->controlMode ] );
    
    AxisMotors.WriteControl( actuator->motor, controlOutputsList[ actuator->controlMode ] );
  }
}

#endif // ACTUATOR_CONTROL_H
