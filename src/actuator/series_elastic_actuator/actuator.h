#ifndef ACTUATOR_CONTROL_H
#define ACTUATOR_CONTROL_H 

#include "interface.h"

#include "axis/axis_motor.h"
#include "axis/axis_sensor.h"

#include "impedance_control.h"
#include "spline3_interpolation.h"
#include "filters.h"

#include "klib/khash.h"
#include "config_parser.h"

#include "debug/async_debug.h"

typedef struct _ActuatorData
{
  AxisMotor motor;
  AxisSensor sensor;
  double measuresList[ CONTROL_VARS_NUMBER ];
  double parametersList[ CONTROL_VARS_NUMBER ];
  ImpedanceControlFunction ref_RunControl;
  Thread controlThread;
  enum AxisVariables operationMode;
}
ActuatorData;

typedef ActuatorData* Actuator;

KHASH_MAP_INIT_INT( ActuatorInt, Actuator )
static khash_t( ActuatorInt )* actuatorsList = NULL;


#define ACTUATOR_FUNCTIONS( namespace, function_init ) \
        function_init( Actuator, namespace, Init, const char* ) \
        function_init( void, namespace, End, Actuator ) \
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


static inline Actuator LoadActuatorData( const char* );
static inline void UnloadActuatorData( Actuator );
                                         
Actuator ActuatorControl_Init( const char* configFileName )
{
  DEBUG_PRINT( "trying to create series elastic actuator %s", configFileName );
  
  static char searchPath[ FILE_PARSER_MAX_PATH_LENGTH ];
  
  bool loadError = false;
  
  Actuator newActuator = (Actuator) malloc( sizeof(ActuatorData) );
  memset( newActuator, 0, sizeof(ActuatorData) );

  int configFileID = ConfigParser.LoadFile( configFileName );
  if( configFileID != -1 )
  {
    for( size_t axisIndex = 0; axisIndex < AXES_NUMBER; axisIndex++ )
    {
      sprintf( searchPath, "axes.%s", AXIS_NAMES[ axisIndex ] );
      parser.SetBaseKey( configFileID, searchPath );
      
      if( (axesList[ axisIndex ]->interface = GetAxisInterface( parser.GetStringValue( configFileID, "interface_type" ) )) != NULL )
      {
        if( (axesList[ axisIndex ]->ID = axesList[ axisIndex ]->interface->Connect( parser.GetStringValue( configFileID, "name" ) )) == -1 ) loadError = true;
      }
      else loadError = true;
      
      axesList[ axisIndex ]->gearConversionFactor = parser.GetRealValue( configFileID, "gear_reduction" );
    }
    
    parser.SetBaseKey( configFileID, "" );
    if( (newActuator->interactionForceCurve = Spline3Interp.LoadCurve( parser.GetStringValue( configFileID, "spring_force_curve" ) )) == NULL ) loadError = true;
    
    parser.UnloadFile( configFileID );
  }
  else
  {
    loadError = true;
    DEBUG_PRINT( "configuration file for series elastic actuator %s not found", configFileName );
  }
  
  if( loadError )
  {
    ActuatorControl_End( newActuator );
    return NULL;
  }
  
  DEBUG_PRINT( "created series elastic actuator %s (iterator: %u)", configFileName, newActuatorID );
  
  return newActuator;
}

void ActuatorControl_End( Actuator actuator )
{
  DEBUG_PRINT( "ending actuator %p", actuator );
  
  if( actuator == NULL ) continue;
  
  AxisMotors_End( actuator->motor );
  AxisSensors_End( actuator->sensor );
}

void ActuatorControl_Enable( Actuator actuator )
{
  if( kh_exist( actuatorsList, (khint_t) actuatorID ) )
  {
    Actuator actuator = kh_value( actuatorsList, (khint_t) actuatorID );
    
    DEBUG_PRINT( "enabling actuator %u (key: %d)", (khint_t) actuatorID, kh_key( actuatorsList, (khint_t) actuatorID ) );
    
    AxisMotors_Enable( actuator->motor );
  }
}

void ActuatorControl_Disable( Actuator actuator )
{
  DEBUG_EVENT( 0, "disabling SE actuator %d", actuatorID );
  
  if( kh_exist( actuatorsList, (khint_t) actuatorID ) )
  {
    Actuator actuator = kh_value( actuatorsList, (khint_t) actuatorID );
    
    AxisMotors_Disable( actuator->motor );
  }
}

void ActuatorControl_Reset( Actuator actuator )
{
  if( kh_exist( actuatorsList, (khint_t) actuatorID ) )
  {
    Actuator actuator = kh_value( actuatorsList, (khint_t) actuatorID );
    
    AxisMotors_Reset( actuator->motor );
    AxisSensors_Reset( actuator->sensor );
  }
}

void ActuatorControl_Calibrate( Actuator actuator )
{
  if( kh_exist( actuatorsList, (khint_t) actuatorID ) )
  {
    Actuator actuator = kh_value( actuatorsList, (khint_t) actuatorID );
    
    AxisMotors_SetOffset( actuator->motor );
    AxisSensors_SetOffset( actuator->sensor );
  }
}

bool ActuatorControl_IsEnabled( Actuator actuator )
{
  if( kh_exist( actuatorsList, (khint_t) actuatorID ) )
  {
    Actuator actuator = kh_value( actuatorsList, (khint_t) actuatorID );
    
    return ( AxisMotors_IsEnabled( actuator->motor ) );
  }
  
  return false;
}

bool ActuatorControl_HasError( Actuator actuator )
{
  if( kh_exist( actuatorsList, (khint_t) actuatorID ) )
  {
    Actuator actuator = kh_value( actuatorsList, (khint_t) actuatorID );
    
    if( AxisMotors_HasError( actuator->motor ) ) return true;
    else if( AxisSensors_HasError( actuator->sensor ) ) return true;
  }
  
  return false;
}

double* ActuatorControl_GetMeasuresList( Actuator actuator )
{
  if( !kh_exist( actuatorsList, (khint_t) actuatorID ) ) return NULL; 
    
  Actuator actuator = kh_value( actuatorsList, (khint_t) actuatorID );
  
  double* motorMeasuresList = AxisMotors_ReadMeasures( actuator->motor );
  double sensorMeasure = AxisSensors_Read( actuator->sensor );
  if( motorMeasuresList != NULL && sensorMeasure != 0.0 )
  {
    actuator->measuresList[ AXIS_POSITION ] = sensorMeasure / actuator->gearConversionFactor;
    
    double deformation = sensorMeasure - motorMeasuresList[ AXIS_POSITION ];         // mm

    actuator->measuresList[ AXIS_FORCE ] = Spline3Interp.GetValue( actuator->interactionForceCurve, deformation );
    //actuator->measuresList[ AXIS_FORCE ] *= actuator->gearConversionFactor;
    //actuator->measuresList[ AXIS_FORCE ] = -104.0 * deformation;
    
    //DEBUG_PRINT( "motor: %.3f - sensor: %.3f - force: %.3f", motorMeasuresList[ AXIS_POSITION ], sensorMeasure, actuator->measuresList[ AXIS_FORCE ] );
  }
  
  return (double*) actuator->measuresList;
}

double* ActuatorControl_GetMeasuresList( Actuator actuator )
{
  static double setpointsList[ AXIS_VARS_NUMBER ];
  
  if( !kh_exist( actuatorsList, (khint_t) actuatorID ) ) return;

  Actuator actuator = kh_value( actuatorsList, (khint_t) actuatorID );
  
  setpointsList[ AXIS_POSITION ] = setpoint * actuator->gearConversionFactor;
  setpointsList[ AXIS_VELOCITY ] = setpoint * actuator->gearConversionFactor;
  setpointsList[ AXIS_FORCE ] = setpoint / actuator->gearConversionFactor;
  
  AxisMotors_WriteControl( actuator->motor, setpointsList[ actuator->operationMode ] );
}

void ActuatorControl_SetOperationMode( Actuator actuator, enum ControlVariables mode )
{
  if( !kh_exist( actuatorsList, (khint_t) actuatorID ) ) return;
  
  if( mode < 0 || mode >= AXIS_VARS_NUMBER ) return;
    
  Actuator actuator = kh_value( actuatorsList, (khint_t) actuatorID );
  
  AxisMotors_SetOperationMode( actuator->motor, mode );
  
  actuator->operationMode = mode;
}


#endif // ACTUATOR_CONTROL_H
