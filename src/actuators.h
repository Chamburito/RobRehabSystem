#ifndef ACTUATOR_H
#define ACTUATOR_H 

#include "time/timing.h"

#include "actuator_control/interface.h"
#include "plugin_loader.h"

#include "config_parser.h"

#include "motors.h"
#include "sensors.h"
#include "kalman_filters.h"

#include "debug/async_debug.h"

typedef struct _ActuatorData
{
  DECLARE_MODULE_INTERFACE_REF( ACTUATOR_CONTROL_INTERFACE );
  Controller controller;
  enum ControlVariables controlMode;
  Motor motor;
  Sensor* sensorsList;
  size_t sensorsNumber;
  enum SignalProcessingPhase sensorState;
  KalmanFilter sensorFilter;
  double measuresList[ CONTROL_VARS_NUMBER ];
  double setpointsList[ CONTROL_VARS_NUMBER ];
  double controlError;
  int logID;
}
ActuatorData;

typedef ActuatorData* Actuator;


#define ACTUATOR_INTERFACE( namespace, function_init ) \
        function_init( Actuator, namespace, Init, const char* ) \
        function_init( void, namespace, End, Actuator ) \
        function_init( void, namespace, Enable, Actuator ) \
        function_init( void, namespace, Disable, Actuator ) \
        function_init( void, namespace, Reset, Actuator ) \
        function_init( void, namespace, SetOffset, Actuator ) \
        function_init( void, namespace, Calibrate, Actuator ) \
        function_init( bool, namespace, IsEnabled, Actuator ) \
        function_init( bool, namespace, HasError, Actuator ) \
        function_init( double, namespace, SetSetpoint, Actuator, enum ControlVariables, double ) \
        function_init( double*, namespace, UpdateMeasures, Actuator, double* ) \
        function_init( double, namespace, RunControl, Actuator, double*, double* )

INIT_NAMESPACE_INTERFACE( Actuators, ACTUATOR_INTERFACE )


const char* CONTROL_MODE_NAMES[ CONTROL_MODES_NUMBER ] = { "POSITION", "VELOCITY", "FORCE", "ACCELERATION" };
Actuator Actuators_Init( const char* configFileName )
{
  char filePath[ DATA_IO_MAX_FILE_PATH_LENGTH ];
  
  DEBUG_PRINT( "trying to create series elastic actuator %s", configFileName );
  
  Actuator newActuator = NULL;
  
  sprintf( filePath, "actuators/%s", configFileName );
  int configFileID = ConfigParsing.LoadConfigFile( filePath );
  if( configFileID != DATA_INVALID_ID )
  {
    ConfigParser parser = ConfigParsing.GetParser();
  
    newActuator = (Actuator) malloc( sizeof(ActuatorData) );
    memset( newActuator, 0, sizeof(ActuatorData) );

    bool loadSuccess = true;

    if( (newActuator->sensorsNumber = parser.GetListSize( configFileID, "sensors" )) > 0 )
    {
      newActuator->sensorFilter = Kalman.CreateFilter( CONTROL_MODES_NUMBER );
      Kalman.SetVariablesCoupling( newActuator->sensorFilter, CONTROL_POSITION, CONTROL_VELOCITY, CONTROL_PASS_INTERVAL );
      Kalman.SetVariablesCoupling( newActuator->sensorFilter, CONTROL_POSITION, CONTROL_ACCELERATION, CONTROL_PASS_INTERVAL * CONTROL_PASS_INTERVAL / 2.0 );
      Kalman.SetVariablesCoupling( newActuator->sensorFilter, CONTROL_VELOCITY, CONTROL_ACCELERATION, CONTROL_PASS_INTERVAL );
      
      newActuator->sensorsList = (Sensor*) calloc( newActuator->sensorsNumber, sizeof(Sensor) );
      for( size_t sensorIndex = 0; sensorIndex < newActuator->sensorsNumber; sensorIndex++ )
      {
        newActuator->sensorsList[ sensorIndex ] = Sensors.Init( parser.GetStringValue( configFileID, "", "sensors.%lu.id", sensorIndex ) ); 
        char* sensorType = parser.GetStringValue( configFileID, "", "sensors.%lu.input_variable", sensorIndex );
        for( int controlModeIndex = 0; controlModeIndex < CONTROL_MODES_NUMBER; controlModeIndex++ )
        {
          if( strcmp( sensorType, CONTROL_MODE_NAMES[ controlModeIndex ] ) == 0 ) 
          {
            DEBUG_PRINT( "Found %s sensor for actuator %s", CONTROL_MODE_NAMES[ controlModeIndex ], configFileName );
            Kalman.AddInput( newActuator->sensorFilter, controlModeIndex );
          }
        }
      }
      
      newActuator->sensorState = SIGNAL_PROCESSING_PHASE_MEASUREMENT;
    }
    
    if( (newActuator->motor = Motors.Init( parser.GetStringValue( configFileID, "", "motor.id" ) )) == NULL ) loadSuccess = false;
    
    newActuator->controlMode = CONTROL_POSITION;
    char* controlModeName = parser.GetStringValue( configFileID, (char*) CONTROL_MODE_NAMES[ CONTROL_POSITION ], "motor.output_variable" );
    for( int controlModeIndex = 0; controlModeIndex < CONTROL_MODES_NUMBER; controlModeIndex++ )
    {
      if( strcmp( controlModeName, CONTROL_MODE_NAMES[ controlModeIndex ] ) == 0 ) newActuator->controlMode = controlModeIndex;
    }
    
    newActuator->logID = DATA_LOG_INVALID_ID;
    if( parser.GetBooleanValue( configFileID, false, "log_data" ) )
    {
      sprintf( filePath, "actuators/%s", configFileName );
      newActuator->logID = DataLogging_InitLog( filePath, 9, 1000 );
      DataLogging_SetDataPrecision( newActuator->logID, 6 );
    } 
    
    sprintf( filePath, "actuator_control/%s", parser.GetStringValue( configFileID, "", "controller" ) );
    GET_PLUGIN_IMPLEMENTATION( ACTUATOR_CONTROL_INTERFACE, filePath, newActuator, &loadSuccess );
    if( loadSuccess ) newActuator->controller = newActuator->InitController();

    parser.UnloadData( configFileID );

    if( !loadSuccess )
    {
      Actuators_End( newActuator );
      return NULL;
    }

    DEBUG_PRINT( "created series elastic actuator %s", configFileName );

    Actuators_Reset( newActuator );
  }
  else
    DEBUG_PRINT( "configuration for series elastic actuator %s is not available", configFileName );
  
  return newActuator;
}

void Actuators_End( Actuator actuator )
{
  if( actuator == NULL ) return;
  
  actuator->EndController( actuator->controller );
  
  Kalman.DiscardFilter( actuator->sensorFilter );
  
  Motors.End( actuator->motor );
  for( size_t sensorIndex = 0; sensorIndex < actuator->sensorsNumber; sensorIndex++ )
    Sensors.End( actuator->sensorsList[ sensorIndex ] );
  
  if( actuator->logID != DATA_LOG_INVALID_ID ) DataLogging_EndLog( actuator->logID );
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

double Actuators_SetSetpoint( Actuator actuator, enum ControlVariables variable, double value )
{
  if( actuator == NULL ) return 0.0; 
  
  if( variable >= CONTROL_VARS_NUMBER ) return 0.0;
  
  actuator->setpointsList[ variable ] = value;
  
  return actuator->setpointsList[ variable ];
}

double* Actuators_UpdateMeasures( Actuator actuator, double* measuresList )
{
  if( actuator == NULL ) return NULL;
  
  DEBUG_UPDATE( "reading measures from actuator %p", actuator );
  
  for( size_t sensorIndex = 0; sensorIndex < actuator->sensorsNumber; sensorIndex++ )
  {
    double sensorMeasure = Sensors_Update( actuator->sensorsList[ sensorIndex ] );
    Kalman_SetInput( actuator->sensorFilter, sensorIndex, sensorMeasure );
  }

  (void) Kalman_Predict( actuator->sensorFilter, actuator->measuresList );
  (void) Kalman_Update( actuator->sensorFilter, NULL, actuator->measuresList );
  
  actuator->measuresList[ CONTROL_POSITION ] = Sensors.Update( actuator->sensorsList[ 0 ] );
  actuator->measuresList[ CONTROL_VELOCITY ] = Sensors.Update( actuator->sensorsList[ 1 ] );
  actuator->measuresList[ CONTROL_FORCE ] = Sensors.Update( actuator->sensorsList[ 2 ] );
  
  //DEBUG_PRINT( "position: %g - velocity: %g - force: %g", actuator->measuresList[ CONTROL_POSITION ], actuator->measuresList[ CONTROL_VELOCITY ], actuator->measuresList[ CONTROL_FORCE ] );
  
  if( measuresList == NULL ) return (double*) actuator->measuresList;
  
  measuresList[ CONTROL_POSITION ] = actuator->measuresList[ CONTROL_POSITION ];
  measuresList[ CONTROL_VELOCITY ] = actuator->measuresList[ CONTROL_VELOCITY ];
  measuresList[ CONTROL_ACCELERATION ] = actuator->measuresList[ CONTROL_ACCELERATION ];
  measuresList[ CONTROL_FORCE ] = actuator->measuresList[ CONTROL_FORCE ];
  
  return measuresList;
}

double Actuators_RunControl( Actuator actuator, double* measuresList, double* setpointsList )
{
  if( actuator == NULL ) return 0.0;
  
  if( measuresList == NULL ) measuresList = (double*) actuator->measuresList;
  if( setpointsList == NULL ) setpointsList = (double*) actuator->setpointsList;
  
  if( setpointsList[ CONTROL_STIFFNESS ] < 0.0 ) setpointsList[ CONTROL_STIFFNESS ] = 0.0;
  if( setpointsList[ CONTROL_DAMPING ] < 0.0 ) setpointsList[ CONTROL_DAMPING ] = 0.0;
  
  //DEBUG_PRINT( "got parameters: %.5f %.5f", actuator->setpointsList[ CONTROL_POSITION ], actuator->setpointsList[ CONTROL_VELOCITY ] );
  
  double* controlOutputsList = actuator->RunControlStep( actuator->controller, measuresList, setpointsList, CONTROL_PASS_INTERVAL, &(actuator->controlError) );
  
  if( actuator->logID != DATA_LOG_INVALID_ID ) 
  {
    DataLogging.RegisterValues( actuator->logID, 9, Timing.GetExecTimeSeconds(),
                                                    measuresList[ CONTROL_POSITION ], measuresList[ CONTROL_VELOCITY ], measuresList[ CONTROL_FORCE ],
                                                    setpointsList[ CONTROL_POSITION ], setpointsList[ CONTROL_VELOCITY ], setpointsList[ CONTROL_FORCE ],
                                                    actuator->controlError, controlOutputsList[ actuator->controlMode ] );
  }
  
  // If the motor is being actually controlled, call control pass algorhitm
  if( Motors.IsEnabled( actuator->motor ) ) Motors.WriteControl( actuator->motor, controlOutputsList[ actuator->controlMode ] );
  
  return controlOutputsList[ actuator->controlMode ];
}

#endif // ACTUATOR_H
