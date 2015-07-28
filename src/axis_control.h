#ifndef AXIS_CONTROL_H
#define AXIS_CONTROL_H

#include "impedance_control.h"
#include "spline3_interpolation.h"

#include "async_debug.h"

#include "filters.h"

#include "data_logging.h"

/////////////////////////////////////////////////////////////////////////////////
/////                            CONTROL DEVICES                            /////
/////////////////////////////////////////////////////////////////////////////////

const size_t DEVICE_NAME_MAX_LENGTH = 16;

typedef struct _AxisController
{
  char name[ DEVICE_NAME_MAX_LENGTH ];
  ActuatorInterface* actuatorInterface;
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

static AxisController* controllersList = NULL;
static size_t devicesNumber = 0;

static void* AsyncControl( void* );

// Axes configuration loading auxiliary function
static void LoadControllersConfig()
{
  char readBuffer[ 64 ];
  
  AxisController* newController = NULL;
  
  FILE* configFile = fopen( "../config/aes_controls_config.txt", "r" );
  if( configFile != NULL ) 
  {
    while( fscanf( configFile, "%s", readBuffer ) != EOF )
    {
      if( strcmp( readBuffer, "BEGIN_AES_CONTROL" ) == 0 )
      {
        controllersList = (AxisController*) realloc( controllersList, sizeof(AxisController) * ( devicesNumber + 1 ) );
  
        newController = &(controllersList[ devicesNumber ]);
        
        fscanf( configFile, "%s", newController->name );
  
        /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "found %s axis control config", newController->name );
        
        newController->actuator = NULL;
        newController->sensor = NULL;
        for( size_t dimensionIndex = 0; dimensionIndex < CONTROL_DIMS_NUMBER; dimensionIndex++ )
          newController->measuresList[ dimensionIndex ] = 0.0;
        for( size_t parameterIndex = 0; parameterIndex < CONTROL_PARAMS_NUMBER; parameterIndex++ )
        {
          newController->parametersList[ parameterIndex ] = 0.0;
          newController->parameterCurvesList[ parameterIndex ] = NULL;
        }
        newController->interactionForceCurve = NULL;
        newController->maxReach = newController->minReach = 0.0;
        newController->maxStiffness = newController->maxDamping;
        newController->ref_RunControl = NULL;
        newController->controlThread = -1;
      }
      
      if( newController == NULL ) continue;
      
      if( strcmp( readBuffer, "motor:" ) == 0 )
      {
        unsigned int nodeID, encoderResolution;
        double gearReduction;
        fscanf( configFile, "%u %u %lf", &nodeID, &encoderResolution, &gearReduction );
        
        DEBUG_EVENT( 1, "found %s motor on node ID %u with encoder resolution %u and gear reduction %g", newController->name, nodeID, encoderResolution, gearReduction );
        
        newController->actuator = Motor_Connect( nodeID );
        MotorDrive_SetEncoderResolution( newController->actuator->drive, encoderResolution );
        MotorDrive_SetGearReduction( newController->actuator->drive, gearReduction );
      }
      else if( strcmp( readBuffer, "encoder:" ) == 0 )
      {
        unsigned int nodeID, encoderResolution;
        double gearReduction;
        fscanf( configFile, "%u %u %lf", &nodeID, &encoderResolution, &gearReduction );
        
        DEBUG_EVENT( 2, "found %s sensor on node ID %u with encoder resolution %u and gear reduction %g", newController->name, nodeID, encoderResolution, gearReduction );
        
        newController->sensor = MotorDrive_Connect( nodeID );
        MotorDrive_SetEncoderResolution( newController->sensor, encoderResolution );
        MotorDrive_SetGearReduction( newController->sensor, gearReduction );
      }
      else if( strcmp( readBuffer, "min_max_reach:" ) == 0 )
      {
        fscanf( configFile, "%lf %lf", &(newController->minReach), &(newController->maxReach) );
        
        /*DEBUG_EVENT( 8,*/DEBUG_PRINT( "found %s reach value limits: %g <-> %g", newController->name, newController->minReach, newController->maxReach );
      }
      else if( strcmp( readBuffer, "control_type:" ) == 0 )
      {
        fscanf( configFile, "%s", readBuffer );
        
        /*DEBUG_EVENT( 3,*/DEBUG_PRINT( "setting %s control function to %s", newController->name, readBuffer );
        
        newController->ref_RunControl = ImpedanceControl_GetFunction( readBuffer );
      }
      else if( strcmp( readBuffer, "operation_mode:" ) == 0 )
      {
        fscanf( configFile, "%s", readBuffer );
        
        /*DEBUG_EVENT( 7,*/DEBUG_PRINT( "setting %s operation mode to %s", newController->name, readBuffer );
        
        if( strcmp( readBuffer, "POSITION" ) == 0 )
          newController->operationMode = AXIS_OP_MODE_POSITION;
        else if( strcmp( readBuffer, "VELOCITY" ) == 0 )
          newController->operationMode = AXIS_OP_MODE_VELOCITY;
      }
      else if( strcmp( readBuffer, "force_curve:" ) == 0 )
      {
        fscanf( configFile, "%s", readBuffer );
        
        /*DEBUG_EVENT( 4,*/DEBUG_PRINT( "setting %s interaction force curve %s", newController->name, readBuffer );
        
        newController->interactionForceCurve = Spline3Interp_LoadCurve( readBuffer );
      }
      else if( strcmp( readBuffer, "parameter_curves:" ) == 0 )
      {
        for( size_t parameterIndex = 0; parameterIndex < CONTROL_PARAMS_NUMBER; parameterIndex++ )
        {
          fscanf( configFile, "%s", readBuffer );
        
          /*DEBUG_EVENT( 6,*/DEBUG_PRINT( "setting %s parameter curve %s", newController->name, readBuffer );
        
          newController->parameterCurvesList[ parameterIndex ] = Spline3Interp_LoadCurve( readBuffer );
        }
      }
      else if( strcmp( readBuffer, "END_AES_CONTROL" ) == 0 )
      {
        newController->controlThread = Thread_Start( AsyncControl, (void*) newController, THREAD_JOINABLE );
        
        DEBUG_EVENT( 5, "running %s control on thread %x", newController->name, newController->controlThread );
        
        if( newController->actuator == NULL || newController->controlThread == -1 )
        {
          newController->isRunning = false;
    
          Thread_WaitExit( newController->controlThread, 5000 );
    
          Motor_Disconnect( newController->actuator );
          MotorDrive_Disconnect( newController->sensor );
        }
        else
        {
          if( newController->sensor == NULL )
            newController->sensor = newController->actuator->drive;
          
          newController->positionFilter = SimpleKalman_CreateFilter( 0.0 );
          
                    devicesNumber++;
        }
      }
      else if( strcmp( readBuffer, "#" ) == 0 )
      {
        char dummyChar;
        
        do { 
          if( fscanf( configFile, "%c", &dummyChar ) == EOF ) 
            break; 
        } while( dummyChar != '\n' ); 
      }
    }
    
    fclose( configFile );
  }
}

// EPOS devices and CAN network initialization
void AxisControl_Init()
{
  DEBUG_EVENT( 0, "Initializing axis control on thread %x", THREAD_ID );
  
  LoadControllersConfig();
  
  DEBUG_EVENT( 0, "axis control initialized on thread %x", THREAD_ID );
}

// EPOS devices and CAN network shutdown
void AxisControl_End()
{
  DEBUG_EVENT( 0, "Ending axis control on thread %x", THREAD_ID );

  Timing_Delay( 2000 );
  
  if( controllersList != NULL )
  {
    // Destroy axes data structures
    for( size_t deviceID = 0; deviceID < devicesNumber; deviceID++ )
    {
      controllersList[ deviceID ].isRunning = false;
    
      if( controllersList[ deviceID ].controlThread != -1 )
        Thread_WaitExit( controllersList[ deviceID ].controlThread, 5000 );
    
      Motor_Disconnect( controllersList[ deviceID ].actuator );
      MotorDrive_Disconnect( controllersList[ deviceID ].sensor );
      
      SimpleKalman_DiscardFilter( controllersList[ deviceID ].positionFilter );
      
      for( size_t parameterIndex = 0; parameterIndex < CONTROL_PARAMS_NUMBER; parameterIndex++ )
        Spline3Interp_UnloadCurve( controllersList[ deviceID ].parameterCurvesList[ parameterIndex ] );
    }
  
    free( controllersList );
  }
  
  DEBUG_EVENT( 0, "axis control ended on thread %x", THREAD_ID );
}

int AxisControl_GetAxisID( const char* axisName )
{
  for( size_t deviceID = 0; deviceID < devicesNumber; deviceID++ )
  {
    if( strcmp( controllersList[ deviceID ].name, axisName ) == 0 )
      return (int) deviceID;
  }
  
  return -1;
}

extern inline size_t AxisControl_GetDevicesNumber()
{
  return devicesNumber;
}

static inline bool CheckController( size_t deviceID )
{
  if( (int) deviceID < 0 || deviceID >= devicesNumber ) return false;
  
  if( controllersList[ deviceID ].actuator == NULL ) return false;
  
  return true;
}

extern inline void AxisControl_EnableMotor( size_t deviceID )
{
  if( !CheckController( deviceID ) ) return;
  
  if( controllersList[ deviceID ].ref_RunControl != NULL )
    Motor_Enable( controllersList[ deviceID ].actuator );
  
  controllersList[ deviceID ].measuresList[ CONTROL_ERROR ] = 0.0;
}

extern inline void AxisControl_DisableMotor( size_t deviceID )
{
  if( !CheckController( deviceID ) ) return;
  
  Motor_Disable( controllersList[ deviceID ].actuator );
  
  controllersList[ deviceID ].measuresList[ CONTROL_ERROR ] = 0.0;
}

extern inline void AxisControl_Reset( size_t deviceID )
{
  if( !CheckController( deviceID ) ) return;
  
  if( MotorDrive_CheckState( controllersList[ deviceID ].actuator->drive, FAULT ) )
  {
    MotorDrive_Reset( controllersList[ deviceID ].actuator->drive );
    MotorDrive_Reset( controllersList[ deviceID ].sensor );
  }
  
  controllersList[ deviceID ].measuresList[ CONTROL_ERROR ] = 0.0;
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
      DEBUG_UPDATE( "control pass for %s AES (before delay): elapsed time: %u ms", controller->name, elapsedTime );
      if( elapsedTime < (int) ( 1000 * CONTROL_SAMPLING_INTERVAL ) ) Timing_Delay( 1000 * CONTROL_SAMPLING_INTERVAL - elapsedTime );
      DEBUG_UPDATE( "control pass for %s AES (after delay): elapsed time: %u ms", controller->name, Timing_GetExecTimeMilliseconds() - execTime );
    }
  }
  
  Thread_Exit( 0 );
  return NULL;
}

#endif /* AXIS_CONTROL_H */
