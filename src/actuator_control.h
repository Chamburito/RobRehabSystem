#ifndef ACTUATOR_CONTROL_H
#define ACTUATOR_CONTROL_H 

#include "interface.h"

#include "config_parser.h"

#include "motors.h"
#include "sensors.h"
#include "kalman_filters.h"

#include "control_interface.h"
#include "plugin_loader.h"

#include "debug/async_debug.h"

const double CONTROL_PASS_INTERVAL = 0.005;

typedef struct _ActuatorData
{
  Motor motor;
  Sensor positionSensor, velocitySensor, forceSensor;
  enum SignalProcessingPhase sensorState;
  KalmanFilter positionFilter;
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
        function_init( void, namespace, SetOffset, Actuator ) \
        function_init( void, namespace, Calibrate, Actuator ) \
        function_init( bool, namespace, IsEnabled, Actuator ) \
        function_init( bool, namespace, HasError, Actuator ) \
        function_init( double*, namespace, GetMeasuresList, Actuator ) \
        function_init( double*, namespace, GetSetpointsList, Actuator )

INIT_NAMESPACE_INTERFACE( ActuatorControl, ACTUATOR_FUNCTIONS )


static void* AsyncControl( void* );

const char* CONTROL_MODE_NAMES[ CONTROL_VARS_NUMBER ] = { "POSITION", "VELOCITY", "FORCE" };
Actuator ActuatorControl_InitController( const char* configFileName )
{
  DEBUG_PRINT( "trying to create series elastic actuator %s", configFileName );
  
  Actuator newActuator = NULL;
  
  int configFileID = ConfigParsing.LoadConfigFile( configFileName );
  if( configFileID != PARSED_DATA_INVALID_ID )
  {
    ParserInterface parser = ConfigParsing.GetParser();
  
    newActuator = (Actuator) malloc( sizeof(ActuatorData) );
    memset( newActuator, 0, sizeof(ActuatorData) );

    bool loadSuccess = true;

    if( (newActuator->motor = Motors.Init( parser.GetStringValue( configFileID, "", "motor" ) )) == NULL ) loadSuccess = false;
    if( (newActuator->positionSensor = Sensors.Init( parser.GetStringValue( configFileID, "", "sensors.position" ) )) == NULL ) loadSuccess = false;
    if( (newActuator->velocitySensor = Sensors.Init( parser.GetStringValue( configFileID, "", "sensors.velocity" ) )) == NULL ) loadSuccess = false;
    if( (newActuator->forceSensor = Sensors.Init( parser.GetStringValue( configFileID, "", "sensors.force" ) )) == NULL ) loadSuccess = false;

    newActuator->sensorState = SIGNAL_PROCESSING_PHASE_MEASUREMENT;
    
    newActuator->positionFilter = Kalman.CreateFilter( 2 );
    Kalman.SetVariablesCoupling( newActuator->positionFilter, 1, 0, CONTROL_PASS_INTERVAL );
    
    newActuator->controlMode = CONTROL_POSITION;
    char* controlModeName = parser.GetStringValue( configFileID, CONTROL_MODE_NAMES[ CONTROL_POSITION ], "control.variable" );
    for( int controlModeIndex = 0; controlModeIndex < CONTROL_VARS_NUMBER; controlModeIndex++ )
    {
      if( strcmp( controlModeName, CONTROL_MODE_NAMES[ controlModeIndex ] ) == 0 ) newActuator->controlMode = controlModeIndex;
    }
    
    GET_PLUGIN_INTERFACE( CONTROL_FUNCTIONS, parser.GetStringValue( configFileID, "", "control.function" ), newActuator->control, loadSuccess );
    if( loadSuccess ) newActuator->controlThread = Threading.StartThread( AsyncControl, newActuator, THREAD_JOINABLE );

    parser.UnloadData( configFileID );

    if( !loadSuccess )
    {
      ActuatorControl_EndController( newActuator );
      return NULL;
    }

    DEBUG_PRINT( "created series elastic actuator %s", configFileName );

    ActuatorControl_Enable( newActuator );
  }
  else
    DEBUG_PRINT( "configuration for series elastic actuator %s is not available", configFileName );
  
  return newActuator;
}

void ActuatorControl_EndController( Actuator actuator )
{
  DEBUG_PRINT( "ending actuator %p", actuator );
  
  if( actuator == NULL ) return;
  
  Kalman.DiscardFilter( actuator->positionFilter );
  
  Motors.End( actuator->motor );
  Sensors.End( actuator->positionSensor );
  Sensors.End( actuator->velocitySensor );
  Sensors.End( actuator->forceSensor );
}

void ActuatorControl_Enable( Actuator actuator )
{
  if( actuator == NULL ) return;
  
  Motors.Enable( actuator->motor );     
}

void ActuatorControl_Disable( Actuator actuator )
{
  if( actuator == NULL ) return;
    
  Motors.Disable( actuator->motor );
}

void ActuatorControl_Reset( Actuator actuator )
{
  if( actuator == NULL ) return;
    
  Motors.Reset( actuator->motor );
  
  Sensors.Reset( actuator->positionSensor );
  Sensors.Reset( actuator->velocitySensor );
  Sensors.Reset( actuator->forceSensor );
  
  Kalman.Reset( actuator->positionFilter );
}

void ActuatorControl_SetOffset( Actuator actuator )
{
  if( actuator == NULL ) return;

  if( actuator->sensorState == SIGNAL_PROCESSING_PHASE_OFFSET )
  {
    Sensors.SetState( actuator->positionSensor, SIGNAL_PROCESSING_PHASE_MEASUREMENT );
    Sensors.SetState( actuator->forceSensor, SIGNAL_PROCESSING_PHASE_MEASUREMENT );
  }
  else
  {
    Sensors.SetState( actuator->positionSensor, SIGNAL_PROCESSING_PHASE_OFFSET );
    Sensors.SetState( actuator->forceSensor, SIGNAL_PROCESSING_PHASE_OFFSET );
  }
}

void ActuatorControl_Calibrate( Actuator actuator )
{
  if( actuator == NULL ) return;

  if( actuator->sensorState == SIGNAL_PROCESSING_PHASE_CALIBRATION )
  {
    Sensors.SetState( actuator->positionSensor, SIGNAL_PROCESSING_PHASE_MEASUREMENT );
    Sensors.SetState( actuator->forceSensor, SIGNAL_PROCESSING_PHASE_MEASUREMENT );
  }
  else
  {  
    Sensors.SetState( actuator->positionSensor, SIGNAL_PROCESSING_PHASE_CALIBRATION );
    Sensors.SetState( actuator->forceSensor, SIGNAL_PROCESSING_PHASE_CALIBRATION );
  }
}

bool ActuatorControl_IsEnabled( Actuator actuator )
{
  if( actuator == NULL ) return false;
    
  return Motors.IsEnabled( actuator->motor );
}

bool ActuatorControl_HasError( Actuator actuator )
{
  if( actuator == NULL ) return false;
    
  if( Motors.HasError( actuator->motor ) ) return true;
  else if( Sensors.HasError( actuator->positionSensor ) ) return true;
  else if( Sensors.HasError( actuator->forceSensor ) ) return true;
  
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

void ActuatorControl_SetOperationMode( Actuator actuator, enum ControlVariables mode )
{
  if( actuator == NULL ) return;
  
  if( mode < 0 || mode >= CONTROL_VARS_NUMBER ) return;
  
  actuator->controlMode = mode;
}



/////////////////////////////////////////////////////////////////////////////////
/////                         ASYNCRONOUS CONTROL                           /////
/////////////////////////////////////////////////////////////////////////////////

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
    if( elapsedTime < (int) ( 1000 * CONTROL_PASS_INTERVAL ) ) Timing_Delay( 1000 * CONTROL_PASS_INTERVAL - elapsedTime );
    DEBUG_UPDATE( "control pass for actuator %p (after delay): elapsed time: %u ms", actuator, Timing_GetExecTimeMilliseconds() - execTime );
  }
  
  DEBUG_PRINT( "ending control thread %lx", THREAD_ID );
   
  return NULL;
}

static inline void UpdateControlMeasures( Actuator actuator )
{
  DEBUG_UPDATE( "reading measures from actuator %p", actuator );
  
  Kalman.SetMeasure( actuator->positionFilter, 0, Sensors.Update( actuator->positionSensor ) );
  Kalman.SetMeasure( actuator->positionFilter, 1, Sensors.Update( actuator->velocitySensor ) );
  (void) Kalman.Predict( actuator->positionFilter );
  double* fusedPositionSignal = Kalman.Update( actuator->positionFilter );
  if( fusedPositionSignal != NULL )
  {
    actuator->measuresList[ CONTROL_POSITION ] = fusedPositionSignal[ 0 ];
    actuator->measuresList[ CONTROL_VELOCITY ] = fusedPositionSignal[ 1 ];
    
    actuator->measuresList[ CONTROL_FORCE ] = Sensors.Update( actuator->forceSensor );
  }
}

static inline void UpdateControlSetpoints( Actuator actuator )
{
  actuator->setpointsList[ CONTROL_POSITION ] += actuator->setpointsList[ CONTROL_VELOCITY ] * CONTROL_PASS_INTERVAL;
  //actuator->setpointsList[ CONTROL_VELOCITY ] += actuator->setpointsList[ CONTROL_ACCELERATION ] * CONTROL_PASS_INTERVAL;
  
  //DEBUG_PRINT( "got parameters: %.5f %.5f", actuator->setpointsList[ CONTROL_POSITION ], actuator->setpointsList[ CONTROL_VELOCITY ] );
}

static inline void RunControl( Actuator actuator )
{
  // If the motor is being actually controlled, call control pass algorhitm
  
  if( Motors.IsEnabled( actuator->motor ) )
  {
    //DEBUG_PRINT( "force setpoint: %g", actuator->setpointsList[ CONTROL_FORCE ] ); 
    double* controlOutputsList = actuator->control.Run( actuator->measuresList, actuator->setpointsList, CONTROL_PASS_INTERVAL, &(actuator->controlError) );
    //DEBUG_PRINT( "force setpoint: %.3f - control: %.3f", actuator->setpointsList[ CONTROL_FORCE ], controlOutputsList[ actuator->controlMode ] );
    
    Motors.WriteControl( actuator->motor, controlOutputsList[ actuator->controlMode ] );
  }
}

#endif // ACTUATOR_CONTROL_H
