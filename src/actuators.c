////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  This file is part of RobRehabSystem.                                      //
//                                                                            //
//  RobRehabSystem is free software: you can redistribute it and/or modify    //
//  it under the terms of the GNU Lesser General Public License as published  //
//  by the Free Software Foundation, either version 3 of the License, or      //
//  (at your option) any later version.                                       //
//                                                                            //
//  RobRehabSystem is distributed in the hope that it will be useful,         //
//  but WITHOUT ANY WARRANTY; without even the implied warranty of            //
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the              //
//  GNU Lesser General Public License for more details.                       //
//                                                                            //
//  You should have received a copy of the GNU Lesser General Public License  //
//  along with Foobar. If not, see <http://www.gnu.org/licenses/>.            //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////


#include "config_parser.h"

#include "motors.h"
#include "sensors.h"
#include "kalman_filters.h"

#include "time/timing.h"

#include "debug/async_debug.h"
#include "debug/data_logging.h"

#include "actuator_control/interface.h"

#include "actuators.h"


struct _ActuatorData
{
  DECLARE_MODULE_INTERFACE_REF( ACTUATOR_CONTROL_INTERFACE );
  Controller controller;
  enum ControlVariable controlMode;
  Motor motor;
  Sensor* sensorsList;
  size_t sensorsNumber;
  enum SignalProcessingPhase sensorState;
  KalmanFilter sensorFilter;
  ControlVariables measures;
  ControlVariables setpoints;
  double controlError;
  int logID;
};

DEFINE_NAMESPACE_INTERFACE( Actuators, ACTUATOR_INTERFACE )


const char* CONTROL_MODE_NAMES[ CONTROL_MODES_NUMBER ] = { "POSITION", "VELOCITY", "FORCE", "ACCELERATION" };
Actuator Actuators_Init( const char* configFileName )
{
  char filePath[ DATA_IO_MAX_FILE_PATH_LENGTH ];
  
  DEBUG_PRINT( "trying to create actuator %s", configFileName );
  
  Actuator newActuator = NULL;
  
  sprintf( filePath, "actuators/%s", configFileName );
  int configFileID = ConfigParsing.LoadConfigFile( filePath );
  if( configFileID != DATA_INVALID_ID )
  {
    newActuator = (Actuator) malloc( sizeof(ActuatorData) );
    memset( newActuator, 0, sizeof(ActuatorData) );

    bool loadSuccess = true;

    if( (newActuator->sensorsNumber = ConfigParsing.GetParser()->GetListSize( configFileID, "sensors" )) > 0 )
    {
      newActuator->sensorFilter = Kalman.CreateFilter( CONTROL_MODES_NUMBER );
      Kalman.SetVariablesCoupling( newActuator->sensorFilter, CONTROL_POSITION, CONTROL_VELOCITY, CONTROL_PASS_INTERVAL );
      Kalman.SetVariablesCoupling( newActuator->sensorFilter, CONTROL_POSITION, CONTROL_ACCELERATION, CONTROL_PASS_INTERVAL * CONTROL_PASS_INTERVAL / 2.0 );
      Kalman.SetVariablesCoupling( newActuator->sensorFilter, CONTROL_VELOCITY, CONTROL_ACCELERATION, CONTROL_PASS_INTERVAL );
      
      newActuator->sensorsList = (Sensor*) calloc( newActuator->sensorsNumber, sizeof(Sensor) );
      for( size_t sensorIndex = 0; sensorIndex < newActuator->sensorsNumber; sensorIndex++ )
      {
        newActuator->sensorsList[ sensorIndex ] = Sensors.Init( ConfigParsing.GetParser()->GetStringValue( configFileID, "", "sensors.%lu.id", sensorIndex ) ); 
        char* sensorType = ConfigParsing.GetParser()->GetStringValue( configFileID, "", "sensors.%lu.input_variable", sensorIndex );
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
    
    if( (newActuator->motor = Motors.Init( ConfigParsing.GetParser()->GetStringValue( configFileID, "", "motor.id" ) )) == NULL ) loadSuccess = false;
    
    newActuator->controlMode = CONTROL_POSITION;
    char* controlModeName = ConfigParsing.GetParser()->GetStringValue( configFileID, (char*) CONTROL_MODE_NAMES[ CONTROL_POSITION ], "motor.output_variable" );
    for( int controlModeIndex = 0; controlModeIndex < CONTROL_MODES_NUMBER; controlModeIndex++ )
    {
      if( strcmp( controlModeName, CONTROL_MODE_NAMES[ controlModeIndex ] ) == 0 ) newActuator->controlMode = controlModeIndex;
    }
    
    newActuator->logID = DATA_LOG_INVALID_ID;
    if( ConfigParsing.GetParser()->GetBooleanValue( configFileID, false, "log_data" ) )
    {
      sprintf( filePath, "actuators/%s", configFileName );
      newActuator->logID = DataLogging.InitLog( filePath, 9, 1000 );
      DataLogging.SetDataPrecision( newActuator->logID, 6 );
    } 
    
    sprintf( filePath, "actuator_control/%s", ConfigParsing.GetParser()->GetStringValue( configFileID, "", "controller" ) );
    LOAD_MODULE_IMPLEMENTATION( ACTUATOR_CONTROL_INTERFACE, filePath, newActuator, &loadSuccess );
    if( loadSuccess ) newActuator->controller = newActuator->InitController();

    ConfigParsing.GetParser()->UnloadData( configFileID );

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
  
  if( actuator->logID != DATA_LOG_INVALID_ID ) DataLogging.EndLog( actuator->logID );
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

double Actuators_SetSetpoint( Actuator actuator, enum ControlVariable variable, double value )
{
  if( actuator == NULL ) return 0.0; 
  
  if( variable >= CONTROL_VARS_NUMBER ) return 0.0;
  
  double* setpointsList = (double*) &(actuator->setpoints);
  setpointsList[ variable ] = value;
  
  return setpointsList[ variable ];
}

ControlVariables* Actuators_UpdateMeasures( Actuator actuator, ControlVariables* measuresBuffer )
{
  if( actuator == NULL ) return NULL;
  
  DEBUG_UPDATE( "reading measures from actuator %p", actuator );
  
  for( size_t sensorIndex = 0; sensorIndex < actuator->sensorsNumber; sensorIndex++ )
  {
    double sensorMeasure = Sensors.Update( actuator->sensorsList[ sensorIndex ] );
    Kalman.SetInput( actuator->sensorFilter, sensorIndex, sensorMeasure );
  }

  (void) Kalman.Predict( actuator->sensorFilter, (double*) &(actuator->measures) );
  (void) Kalman.Update( actuator->sensorFilter, NULL, (double*) &(actuator->measures) );
  
  actuator->measures.position = Sensors.Update( actuator->sensorsList[ 0 ] );
  actuator->measures.velocity = Sensors.Update( actuator->sensorsList[ 1 ] );
  actuator->measures.force = Sensors.Update( actuator->sensorsList[ 2 ] );
  
  //DEBUG_PRINT( "position: %g - velocity: %g - force: %g", actuator->measures[ CONTROL_POSITION ], actuator->measures[ CONTROL_VELOCITY ], actuator->measures[ CONTROL_FORCE ] );
  
  if( measuresBuffer == NULL ) return &(actuator->measures);
  
  measuresBuffer->position = actuator->measures.position;
  measuresBuffer->velocity = actuator->measures.velocity;
  measuresBuffer->acceleration = actuator->measures.acceleration;
  measuresBuffer->force = actuator->measures.force;
  
  return measuresBuffer;
}

double Actuators_RunControl( Actuator actuator, ControlVariables* measures, ControlVariables* setpoints )
{
  if( actuator == NULL ) return 0.0;
  
  if( measures == NULL ) measures = &(actuator->measures);
  if( setpoints == NULL ) setpoints = &(actuator->setpoints);
  
  if( setpoints->stiffness < 0.0 ) setpoints->stiffness = 0.0;
  if( setpoints->damping < 0.0 ) setpoints->damping = 0.0;
  
  //DEBUG_PRINT( "got parameters: %.5f %.5f", actuator->setpoints[ CONTROL_POSITION ], actuator->setpoints[ CONTROL_VELOCITY ] );
  
  double* controlOutputsList = (double*) actuator->RunControlStep( actuator->controller, measures, setpoints, &(actuator->controlError) );
  
  if( actuator->logID != DATA_LOG_INVALID_ID ) 
  {
    DataLogging.RegisterValues( actuator->logID, 9, Timing.GetExecTimeSeconds(), 
                                                    measures->position, measures->velocity, measures->force,
                                                    setpoints->position, setpoints->velocity, setpoints->force,
                                                    actuator->controlError, controlOutputsList[ actuator->controlMode ] );
  }
  
  // If the motor is being actually controlled, call control pass algorhitm
  if( Motors.IsEnabled( actuator->motor ) ) Motors.WriteControl( actuator->motor, controlOutputsList[ actuator->controlMode ] );
  
  return controlOutputsList[ actuator->controlMode ];
}
