#ifndef ACTUATOR_H
#define ACTUATOR_H 

#include "actuator_control/interface.h"
#include "plugin_loader.h"

#include "config_parser.h"

#include "motors.h"
#include "sensors.h"
#include "kalman_filters.h"

#include "debug/async_debug.h"

typedef struct _ActuatorData
{
  Motor motor;
  Sensor* sensorsList;
  size_t sensorsNumber;
  enum SignalProcessingPhase sensorState;
  KalmanFilter sensorFilter;
  double measuresList[ CONTROL_VARS_NUMBER ];
  double setpointsList[ CONTROL_VARS_NUMBER ];
  double controlError;
  ActuatorControlInterface control;
  Controller controller;
  enum ControlVariables controlMode;
  int logID;
}
ActuatorData;

typedef ActuatorData* Actuator;


#define ACTUATOR_FUNCTIONS( namespace, function_init ) \
        function_init( Actuator, namespace, Init, const char* ) \
        function_init( void, namespace, End, Actuator ) \
        function_init( void, namespace, Enable, Actuator ) \
        function_init( void, namespace, Disable, Actuator ) \
        function_init( void, namespace, Reset, Actuator ) \
        function_init( void, namespace, SetOffset, Actuator ) \
        function_init( void, namespace, Calibrate, Actuator ) \
        function_init( bool, namespace, IsEnabled, Actuator ) \
        function_init( bool, namespace, HasError, Actuator ) \
        function_init( double*, namespace, GetMeasuresList, Actuator ) \
        function_init( double*, namespace, GetSetpointsList, Actuator ) \
        function_init( double, namespace, SetSetpoint, Actuator, enum ControlVariables, double ) \
        function_init( void, namespace, UpdateMeasures, Actuator ) \
        function_init( void, namespace, RunControl, Actuator )

INIT_NAMESPACE_INTERFACE( Actuators, ACTUATOR_FUNCTIONS )


const double CONTROL_PASS_INTERVAL = 0.005;

const char* CONTROL_MODE_NAMES[ CONTROL_OUTPUTS_NUMBER ] = { "POSITION", "VELOCITY", "ACCELERATION", "FORCE" };
char filePath[ PARSER_MAX_FILE_PATH_LENGTH ];
Actuator Actuators_Init( const char* configFileName )
{
  DEBUG_PRINT( "trying to create series elastic actuator %s", configFileName );
  
  Actuator newActuator = NULL;
  
  sprintf( filePath, "actuators/%s", configFileName );
  int configFileID = ConfigParsing.LoadConfigFile( filePath );
  if( configFileID != PARSED_DATA_INVALID_ID )
  {
    ParserInterface parser = ConfigParsing.GetParser();
  
    newActuator = (Actuator) malloc( sizeof(ActuatorData) );
    memset( newActuator, 0, sizeof(ActuatorData) );

    bool loadSuccess = true;

    if( (newActuator->sensorsNumber = parser.GetListSize( configFileID, "sensors" )) > 0 )
    {
      newActuator->sensorFilter = Kalman.CreateFilter( CONTROL_OUTPUTS_NUMBER );
      Kalman.SetVariablesCoupling( newActuator->sensorFilter, CONTROL_VELOCITY, CONTROL_POSITION, CONTROL_PASS_INTERVAL );
      Kalman.SetVariablesCoupling( newActuator->sensorFilter, CONTROL_ACCELERATION, CONTROL_POSITION, CONTROL_PASS_INTERVAL * CONTROL_PASS_INTERVAL / 2.0 );
      Kalman.SetVariablesCoupling( newActuator->sensorFilter, CONTROL_ACCELERATION, CONTROL_VELOCITY, CONTROL_PASS_INTERVAL );
      
      newActuator->sensorsList = (Sensor*) calloc( newActuator->sensorsNumber, sizeof(Sensor) );
      for( size_t sensorIndex = 0; sensorIndex < newActuator->sensorsNumber; sensorIndex++ )
      {
        newActuator->sensorsList[ sensorIndex ] = Sensors.Init( parser.GetStringValue( configFileID, "", "sensors.%lu.id", sensorIndex ) ); 
        char* sensorType = parser.GetStringValue( configFileID, "", "sensors.%lu.input_variable", sensorIndex );
        for( int controlModeIndex = 0; controlModeIndex < CONTROL_OUTPUTS_NUMBER; controlModeIndex++ )
        {
          if( strcmp( sensorType, CONTROL_MODE_NAMES[ controlModeIndex ] ) == 0 ) Kalman.AddInput( newActuator->sensorFilter, controlModeIndex );
        }
      }
      
      newActuator->sensorState = SIGNAL_PROCESSING_PHASE_MEASUREMENT;
    }
    
    if( (newActuator->motor = Motors.Init( parser.GetStringValue( configFileID, "", "motor.id" ) )) == NULL ) loadSuccess = false;
    
    newActuator->controlMode = CONTROL_POSITION;
    char* controlModeName = parser.GetStringValue( configFileID, CONTROL_MODE_NAMES[ CONTROL_POSITION ], "motor.output_variable" );
    for( int controlModeIndex = 0; controlModeIndex < CONTROL_OUTPUTS_NUMBER; controlModeIndex++ )
    {
      if( strcmp( controlModeName, CONTROL_MODE_NAMES[ controlModeIndex ] ) == 0 ) newActuator->controlMode = controlModeIndex;
    }
    
    newActuator->logID = DATA_LOG_INVALID_ID;
    if( parser.GetBooleanValue( configFileID, false, "log_data" ) )
    {
      newActuator->logID = DataLogging_InitLog( configFileName, 8, 1000 );
      DataLogging_SetDataPrecision( newActuator->logID, 6 );
    } 
    
    sprintf( filePath, "actuator_control/%s", parser.GetStringValue( configFileID, "", "controller" ) );
    GET_PLUGIN_INTERFACE( ACTUATOR_CONTROL_FUNCTIONS, filePath, newActuator->control, loadSuccess );
    if( loadSuccess ) newActuator->controller = newActuator->control.InitController();

    parser.UnloadData( configFileID );

    if( !loadSuccess )
    {
      Actuators_End( newActuator );
      return NULL;
    }

    DEBUG_PRINT( "created series elastic actuator %s", configFileName );

    Actuators_Enable( newActuator );
  }
  else
    DEBUG_PRINT( "configuration for series elastic actuator %s is not available", configFileName );
  
  return newActuator;
}

void Actuators_End( Actuator actuator )
{
  DEBUG_PRINT( "ending actuator %p", actuator );
  
  if( actuator == NULL ) return;
  
  actuator->control.EndController( actuator->controller );
  
  Kalman.DiscardFilter( actuator->sensorFilter );
  
  Motors.End( actuator->motor );
  for( size_t sensorIndex = 0; sensorIndex < actuator->sensorsNumber; sensorIndex++ )
    Sensors.End( actuator->sensorsList[ sensorIndex ] );
  
  if( actuator->logID != 0 ) DataLogging_EndLog( actuator->logID );
}

void Actuators_Enable( Actuator actuator )
{
  if( actuator == NULL ) return;
  
  Motors.Enable( actuator->motor );     
}

void Actuators_Disable( Actuator actuator )
{
  if( actuator == NULL ) return;
    
  Motors.Disable( actuator->motor );
}

void Actuators_Reset( Actuator actuator )
{
  if( actuator == NULL ) return;
    
  Motors.Reset( actuator->motor );
  
  for( size_t sensorIndex = 0; sensorIndex < actuator->sensorsNumber; sensorIndex++ )
    Sensors.Reset( actuator->sensorsList[ sensorIndex ] );
  
  Kalman.Reset( actuator->sensorFilter );
}

void Actuators_SetOffset( Actuator actuator )
{
  if( actuator == NULL ) return;

  if( actuator->sensorState == SIGNAL_PROCESSING_PHASE_OFFSET )
  {
    for( size_t sensorIndex = 0; sensorIndex < actuator->sensorsNumber; sensorIndex++ )
      Sensors.SetState( actuator->sensorsList[ sensorIndex ], SIGNAL_PROCESSING_PHASE_MEASUREMENT );
  }
  else
  {
    for( size_t sensorIndex = 0; sensorIndex < actuator->sensorsNumber; sensorIndex++ )
      Sensors.SetState( actuator->sensorsList[ sensorIndex ], SIGNAL_PROCESSING_PHASE_OFFSET );
  }
}

void Actuators_Calibrate( Actuator actuator )
{
  if( actuator == NULL ) return;

  if( actuator->sensorState == SIGNAL_PROCESSING_PHASE_CALIBRATION )
  {
    for( size_t sensorIndex = 0; sensorIndex < actuator->sensorsNumber; sensorIndex++ )
      Sensors.SetState( actuator->sensorsList[ sensorIndex ], SIGNAL_PROCESSING_PHASE_MEASUREMENT );
  }
  else
  {
    for( size_t sensorIndex = 0; sensorIndex < actuator->sensorsNumber; sensorIndex++ )
      Sensors.SetState( actuator->sensorsList[ sensorIndex ], SIGNAL_PROCESSING_PHASE_CALIBRATION ); 
  }
}

bool Actuators_IsEnabled( Actuator actuator )
{
  if( actuator == NULL ) return false;
    
  return Motors.IsEnabled( actuator->motor );
}

bool Actuators_HasError( Actuator actuator )
{
  if( actuator == NULL ) return false;
    
  if( Motors.HasError( actuator->motor ) ) return true;
  
  for( size_t sensorIndex = 0; sensorIndex < actuator->sensorsNumber; sensorIndex++ )
  {
    if( Sensors.HasError( actuator->sensorsList[ sensorIndex ] ) ) return true;
  }
  
  return false;
}

double* Actuators_GetMeasuresList( Actuator actuator )
{
  if( actuator == NULL ) return NULL; 
  
  return (double*) actuator->measuresList;
}

double* Actuators_GetSetpointsList( Actuator actuator )
{
  if( actuator == NULL ) return NULL; 
  
  return (double*) actuator->setpointsList;
}

double Actuators_SetSetpoint( Actuator actuator, enum ControlVariables variable, double value )
{
  if( actuator == NULL ) return 0.0; 
  
  if( variable < 0 || variable >= CONTROL_VARS_NUMBER ) return 0.0;
  
  actuator->setpointsList[ variable ] = value;
  
  return actuator->setpointsList[ variable ];
}

void Actuators_UpdateMeasures( Actuator actuator )
{
  if( actuator == NULL ) return;
  
  DEBUG_UPDATE( "reading measures from actuator %p", actuator );
  
  for( size_t sensorIndex = 0; sensorIndex < actuator->sensorsNumber; sensorIndex++ )
  {
    double sensorMeasure = Sensors.Update( actuator->sensorsList[ sensorIndex ] );
    Kalman.SetInput( actuator->sensorFilter, sensorIndex, sensorMeasure );
  }

  (void) Kalman.Predict( actuator->sensorFilter );
  double* fusedSensorSignal = Kalman.Update( actuator->sensorFilter );
  if( fusedSensorSignal != NULL )
  {
    actuator->measuresList[ CONTROL_POSITION ] = fusedSensorSignal[ CONTROL_POSITION ];
    actuator->measuresList[ CONTROL_VELOCITY ] = fusedSensorSignal[ CONTROL_VELOCITY ];
    actuator->measuresList[ CONTROL_ACCELERATION ] = fusedSensorSignal[ CONTROL_ACCELERATION ];
    actuator->measuresList[ CONTROL_FORCE ] = fusedSensorSignal[ CONTROL_FORCE ];
    
    //DEBUG_PRINT( "fused: position: %g - velocity: %g", fusedSensorSignal[ 0 ], fusedSensorSignal[ 1 ] );
  }
}

void Actuators_RunControl( Actuator actuator )
{
  if( actuator == NULL ) return;
  
  if( actuator->setpointsList[ CONTROL_STIFFNESS ] < 0.0 ) actuator->setpointsList[ CONTROL_STIFFNESS ] = 0.0;
  if( actuator->setpointsList[ CONTROL_DAMPING ] < 0.0 ) actuator->setpointsList[ CONTROL_DAMPING ] = 0.0;
  
  //DEBUG_PRINT( "got parameters: %.5f %.5f", actuator->setpointsList[ CONTROL_POSITION ], actuator->setpointsList[ CONTROL_VELOCITY ] );
  
  double* controlOutputsList = actuator->control.RunStep( actuator->controller, actuator->measuresList, actuator->setpointsList, CONTROL_PASS_INTERVAL, &(actuator->controlError) );
  
  if( actuator->logID != DATA_LOG_INVALID_ID ) 
  {
    DataLogging_RegisterValues( actuator->logID, 8, Timing.GetExecTimeSeconds(),
                                                    actuator->measuresList[ CONTROL_POSITION ], actuator->measuresList[ CONTROL_VELOCITY ], actuator->measuresList[ CONTROL_FORCE ],
                                                    actuator->setpointsList[ CONTROL_POSITION ], actuator->setpointsList[ CONTROL_VELOCITY ], actuator->setpointsList[ CONTROL_FORCE ],
                                                    controlOutputsList[ actuator->controlMode ] );
  }
  
  // If the motor is being actually controlled, call control pass algorhitm
  if( Motors.IsEnabled( actuator->motor ) ) Motors.WriteControl( actuator->motor, controlOutputsList[ actuator->controlMode ] );
}

/////////////////////////////////////////////////////////////////////////////////
/////                         ASYNCRONOUS CONTROL                           /////
/////////////////////////////////////////////////////////////////////////////////

/*static void UpdateControlMeasures( Actuator );
static void UpdateControlSetpoints( Actuator );
static void RunControl( Actuator );

// Method that runs the control functions asyncronously
static void* AsyncControl( void* data )
{
  Actuator actuator = (Actuator) data;
  
  // Try to correct errors
  Actuators_Reset( actuator );
  
  unsigned long execTime, elapsedTime;
  
  actuator->isControlRunning = true;
  
  DEBUG_PRINT( "starting to run control for actuator %p on thread %x", actuator, THREAD_ID );
  
  while( actuator->isControlRunning )
  {
    DEBUG_UPDATE( "running control for actuator %p motor %p", actuator, actuator->motor );
    
    execTime = Timing.GetExecTimeMilliseconds();
    
    if( actuator->logID != DATA_LOG_INVALID_ID ) DataLogging_RegisterValues( actuator->logID, 1, Timing.GetExecTimeSeconds() );
    
    UpdateControlMeasures( actuator );
    
    UpdateControlSetpoints( actuator );
    
    RunControl( actuator );
    
    elapsedTime = Timing.GetExecTimeMilliseconds() - execTime;
    DEBUG_UPDATE( "control pass for actuator %p (before delay): elapsed time: %u ms", actuator, elapsedTime );
    if( elapsedTime < (int) ( 1000 * CONTROL_PASS_INTERVAL ) ) Timing.Delay( 1000 * CONTROL_PASS_INTERVAL - elapsedTime );
    DEBUG_UPDATE( "control pass for actuator %p (after delay): elapsed time: %u ms", actuator, Timing.GetExecTimeMilliseconds() - execTime );
  }
  
  DEBUG_PRINT( "ending control thread %lx", THREAD_ID );
   
  return NULL;
}*/

#endif // ACTUATOR_H
