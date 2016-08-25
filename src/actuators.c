////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  Copyright (c) 2016 Leonardo José Consoni                                  //
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
//  along with RobRehabSystem. If not, see <http://www.gnu.org/licenses/>.    //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////


#include "configuration.h"

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
  enum ControlState controlState;
  Motor motor;
  Sensor* sensorsList;
  size_t sensorsNumber;
  KalmanFilter sensorFilter;
  ControlVariablesList measuresList;
  ControlVariablesList setpointsList;
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
  int configFileID = Configuration.LoadConfigFile( filePath );
  if( configFileID != DATA_INVALID_ID )
  {
    newActuator = (Actuator) malloc( sizeof(ActuatorData) );
    memset( newActuator, 0, sizeof(ActuatorData) );

    bool loadSuccess = true;

    if( (newActuator->sensorsNumber = Configuration.GetIOHandler()->GetListSize( configFileID, "sensors" )) > 0 )
    {
      newActuator->sensorFilter = Kalman.CreateFilter( CONTROL_MODES_NUMBER );
      Kalman.SetVariablesCoupling( newActuator->sensorFilter, CONTROL_POSITION, CONTROL_VELOCITY, CONTROL_PASS_INTERVAL );
      Kalman.SetVariablesCoupling( newActuator->sensorFilter, CONTROL_POSITION, CONTROL_ACCELERATION, CONTROL_PASS_INTERVAL * CONTROL_PASS_INTERVAL / 2.0 );
      Kalman.SetVariablesCoupling( newActuator->sensorFilter, CONTROL_VELOCITY, CONTROL_ACCELERATION, CONTROL_PASS_INTERVAL );
      
      newActuator->sensorsList = (Sensor*) calloc( newActuator->sensorsNumber, sizeof(Sensor) );
      for( size_t sensorIndex = 0; sensorIndex < newActuator->sensorsNumber; sensorIndex++ )
      {
        newActuator->sensorsList[ sensorIndex ] = Sensors.Init( Configuration.GetIOHandler()->GetStringValue( configFileID, "", "sensors.%lu.id", sensorIndex ), 0x00 ); 
        char* sensorType = Configuration.GetIOHandler()->GetStringValue( configFileID, "", "sensors.%lu.input_variable", sensorIndex );
        for( int controlModeIndex = 0; controlModeIndex < CONTROL_MODES_NUMBER; controlModeIndex++ )
        {
          if( strcmp( sensorType, CONTROL_MODE_NAMES[ controlModeIndex ] ) == 0 ) 
          {
            DEBUG_PRINT( "Found %s sensor for actuator %s", CONTROL_MODE_NAMES[ controlModeIndex ], configFileName );
            Kalman.AddInput( newActuator->sensorFilter, controlModeIndex );
          }
        }
      }
    }
    
    if( (newActuator->motor = Motors.Init( Configuration.GetIOHandler()->GetStringValue( configFileID, "", "motor.id" ) )) == NULL ) loadSuccess = false;
    
    newActuator->controlMode = CONTROL_POSITION;
    char* controlModeName = Configuration.GetIOHandler()->GetStringValue( configFileID, (char*) CONTROL_MODE_NAMES[ CONTROL_POSITION ], "motor.output_variable" );
    for( int controlModeIndex = 0; controlModeIndex < CONTROL_MODES_NUMBER; controlModeIndex++ )
    {
      if( strcmp( controlModeName, CONTROL_MODE_NAMES[ controlModeIndex ] ) == 0 ) newActuator->controlMode = controlModeIndex;
    }
    
    newActuator->logID = DATA_LOG_INVALID_ID;
    if( Configuration.GetIOHandler()->GetBooleanValue( configFileID, false, "log_data" ) )
    {
      sprintf( filePath, "actuators/%s", configFileName );
      newActuator->logID = DataLogging.InitLog( filePath, 9, 1000 );
      DataLogging.SetDataPrecision( newActuator->logID, 6 );
    } 
    
    sprintf( filePath, "actuator_control/%s", Configuration.GetIOHandler()->GetStringValue( configFileID, "", "controller" ) );
    LOAD_MODULE_IMPLEMENTATION( ACTUATOR_CONTROL_INTERFACE, filePath, newActuator, &loadSuccess );
    if( loadSuccess ) newActuator->controller = newActuator->InitController();
    
    newActuator->controlState = CONTROL_OPERATION;

    Configuration.GetIOHandler()->UnloadData( configFileID );

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

bool Actuators_SetControlState( Actuator actuator, enum ControlState controlState )
{
  if( actuator == NULL ) return false;
  
  if( controlState == actuator->controlState ) return false;

  enum SignalProcessingPhase sensorsState = SIGNAL_PROCESSING_PHASE_MEASUREMENT;
  if( controlState == CONTROL_OFFSET ) sensorsState = SIGNAL_PROCESSING_PHASE_OFFSET;
  else if( controlState == CONTROL_CALIBRATION ) sensorsState = SIGNAL_PROCESSING_PHASE_CALIBRATION;
  
  for( size_t sensorIndex = 0; sensorIndex < actuator->sensorsNumber; sensorIndex++ )
    Sensors.SetState( actuator->sensorsList[ sensorIndex ], sensorsState );
  
  actuator->controlState = controlState;
  
  return true;
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
  
  actuator->setpointsList[ variable ] = value;
  
  return actuator->setpointsList[ variable ];
}

double* Actuators_UpdateMeasures( Actuator actuator, double* measuresBuffer )
{
  if( actuator == NULL ) return NULL;
  
  DEBUG_UPDATE( "reading measures from actuator %p", actuator );
  
  for( size_t sensorIndex = 0; sensorIndex < actuator->sensorsNumber; sensorIndex++ )
  {
    double sensorMeasure = Sensors.Update( actuator->sensorsList[ sensorIndex ], NULL );
    Kalman.SetInput( actuator->sensorFilter, sensorIndex, sensorMeasure );
  }

  (void) Kalman.Predict( actuator->sensorFilter, (double*) actuator->measuresList );
  (void) Kalman.Update( actuator->sensorFilter, NULL, (double*) actuator->measuresList );
  
  //DEBUG_PRINT( "position: %.3f - velocity: %.3f - force: %.3f", actuator->measuresList[ CONTROL_POSITION ], actuator->measuresList[ CONTROL_VELOCITY ], actuator->measuresList[ CONTROL_FORCE ] );
  
  //actuator->measures.position = Sensors.Update( actuator->sensorsList[ 0 ] );
  //actuator->measures.velocity = Sensors.Update( actuator->sensorsList[ 1 ] );
  //actuator->measures.force = Sensors.Update( actuator->sensorsList[ 2 ] );
  
  if( measuresBuffer == NULL ) return actuator->measuresList;
  
  measuresBuffer[ CONTROL_POSITION ] = actuator->measuresList[ CONTROL_POSITION ];
  measuresBuffer[ CONTROL_VELOCITY ] = actuator->measuresList[ CONTROL_VELOCITY ];
  measuresBuffer[ CONTROL_ACCELERATION ] = actuator->measuresList[ CONTROL_ACCELERATION ];
  measuresBuffer[ CONTROL_FORCE ] = actuator->measuresList[ CONTROL_FORCE ];
  
  return measuresBuffer;
}

double Actuators_RunControl( Actuator actuator, double* measuresList, double* setpointsList )
{
  if( actuator == NULL ) return 0.0;
  
  if( measuresList == NULL ) measuresList = (double*) actuator->measuresList;
  if( setpointsList == NULL ) setpointsList = (double*) actuator->setpointsList;
  
  if( setpointsList[ CONTROL_STIFFNESS ] < 0.0 ) setpointsList[ CONTROL_STIFFNESS ] = 0.0;
  if( setpointsList[ CONTROL_DAMPING ] < 0.0 ) setpointsList[ CONTROL_DAMPING ] = 0.0;
  
  //DEBUG_PRINT( "got parameters: %.5f %.5f", actuator->setpoints[ CONTROL_POSITION ], actuator->setpoints[ CONTROL_VELOCITY ] );
  
  double* controlOutputsList = (double*) actuator->RunControlStep( actuator->controller, measuresList, setpointsList, &(actuator->controlError) );
  
  if( actuator->logID != DATA_LOG_INVALID_ID ) 
  {
    DataLogging.RegisterValues( actuator->logID, 9, Timing.GetExecTimeSeconds(), 
                                                    measuresList[ CONTROL_POSITION ], measuresList[ CONTROL_VELOCITY ], measuresList[ CONTROL_FORCE ],
                                                    setpointsList[ CONTROL_POSITION ], setpointsList[ CONTROL_VELOCITY ], setpointsList[ CONTROL_FORCE ],
                                                    actuator->controlError, controlOutputsList[ actuator->controlMode ] );
  }
  
  // If the motor is being actually controlled, write its control output
  if( Motors.IsEnabled( actuator->motor ) && actuator->controlState != CONTROL_OFFSET ) 
    Motors.WriteControl( actuator->motor, controlOutputsList[ actuator->controlMode ] );
  
  return controlOutputsList[ actuator->controlMode ];
}
