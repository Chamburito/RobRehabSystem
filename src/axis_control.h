#ifndef CONTROL_H
#define CONTROL_H

#include "axis.h"
#include "axis_control_functions.h"
#include "splined3_curves.h"

#include "async_debug.h"

#include "filters.h"

#include "data_logging.h"

/////////////////////////////////////////////////////////////////////////////////
/////                            CONTROL DEVICES                            /////
/////////////////////////////////////////////////////////////////////////////////

//CAN database addressing
const char* CAN_DATABASE = "database";
const char* CAN_CLUSTER = "NETCAN";

typedef struct _AESController
{
  char name[ DEVICE_NAME_MAX_LENGTH ];
  Motor* actuator;                                                  // Active axis
  unsigned int operationMode;                                       // Actuator operation mode (Position or velocity)
  MotorDrive* sensor;                                               // Passive (extra measurement) axis
  ImpedanceControlFunction ref_RunControl;                        // Control method definition structure
  Thread_Handle controlThread;                                       // Processing thread handle
  double measuresList[ CONTROL_DIMS_NUMBER ];
  SimpleKalmanFilter* positionFilter;
  double parametersList[ CONTROL_PARAMS_NUMBER ];
  Splined3Curve* parameterCurvesList[ CONTROL_PARAMS_NUMBER ];
  double maxReach, minReach;
  double maxStiffness, maxDamping, physicalStiffness;               // Real stiffness and damping of the actuator
  bool isRunning;                                                   // Is control thread running ?
}
AESController;

static AESController* controllersList = NULL;
static size_t devicesNumber = 0;

static void* AsyncControl( void* );

// Axes configuration loading auxiliary function
static void LoadControllersConfig()
{
  char readBuffer[ 64 ];
  
  AESController* newController = NULL;
  
  FILE* configFile = fopen( "../config/aes_controls_config.txt", "r" );
  if( configFile != NULL ) 
  {
    while( fscanf( configFile, "%s", readBuffer ) != EOF )
    {
      if( strcmp( readBuffer, "BEGIN_AES_CONTROL" ) == 0 )
      {
        controllersList = (AESController*) realloc( controllersList, sizeof(AESController) * ( devicesNumber + 1 ) );
  
        newController = &(controllersList[ devicesNumber ]);
        
        fscanf( configFile, "%s", newController->name );
  
        DEBUG_EVENT( 0, "found axis control %s", newController->name );
        
        newController->actuator = NULL;
        newController->sensor = NULL;
        for( size_t dimensionIndex = 0; dimensionIndex < CONTROL_DIMS_NUMBER; dimensionIndex++ )
          newController->measuresList[ dimensionIndex ] = 0.0;
        for( size_t parameterIndex = 0; parameterIndex < CONTROL_PARAMS_NUMBER; parameterIndex++ )
        {
          newController->parametersList[ parameterIndex ] = 0.0;
          newController->parameterCurvesList[ parameterIndex ] = NULL;
        }
        newController->maxReach = newController->minReach = 0.0;
        newController->maxStiffness = newController->maxDamping = newController->physicalStiffness = 0.0;
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
      else if( strcmp( readBuffer, "base_stiffness:" ) == 0 )
      {
        fscanf( configFile, "%lf", &(newController->physicalStiffness) );
        
        DEBUG_EVENT( 4, "found %s base stiffness value: %g", newController->name, newController->physicalStiffness );
        
        if( newController->physicalStiffness < 0.0 ) newController->physicalStiffness = 0.0;
      }
      else if( strcmp( readBuffer, "min_max_reach:" ) == 0 )
      {
        fscanf( configFile, "%lf %lf", &(newController->minReach), &(newController->maxReach) );
        
        DEBUG_EVENT( 8, "found %s reach value limit: %g <-> %g", newController->name, newController->minReach, newController->maxReach );
        
        if( newController->physicalStiffness < 0.0 ) newController->physicalStiffness = 0.0;
      }
      else if( strcmp( readBuffer, "control_type:" ) == 0 )
      {
        fscanf( configFile, "%s", readBuffer );
        
        DEBUG_EVENT( 3, "setting %s control function to %s", newController->name, readBuffer );
        
        newController->ref_RunControl = ImpedanceControl_GetFunction( readBuffer );
      }
      else if( strcmp( readBuffer, "operation_mode:" ) == 0 )
      {
        fscanf( configFile, "%s", readBuffer );
        
        DEBUG_EVENT( 7, "setting %s operation mode to %s", newController->name, readBuffer );
        
        if( strcmp( readBuffer, "POSITION" ) == 0 )
          newController->operationMode = POSITION_MODE;
        else if( strcmp( readBuffer, "VELOCITY" ) == 0 )
          newController->operationMode = VELOCITY_MODE;
      }
      else if( strcmp( readBuffer, "parameter_curves:" ) == 0 )
      {
        for( size_t parameterIndex = 0; parameterIndex < CONTROL_PARAMS_NUMBER; parameterIndex++ )
        {
          fscanf( configFile, "%s", readBuffer );
        
          DEBUG_EVENT( 6, "setting %s parameter curve %s", newController->name, readBuffer );
        
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
void AESControl_Init()
{
  DEBUG_EVENT( 0, "Initializing AES Control on thread %x", THREAD_ID );
  
  EposNetwork_Start( CAN_DATABASE, CAN_CLUSTER );
  
  LoadControllersConfig();
  
  DEBUG_EVENT( 0, "AES Control initialized on thread %x", THREAD_ID );
}

// EPOS devices and CAN network shutdown
void AESControl_End()
{
  DEBUG_EVENT( 0, "Ending AES Control on thread %x", THREAD_ID );

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
      Spline3Interp_UnloadCurve( controllersList[ deviceID ].parameterCurvesList );
    }
  
    free( controllersList );
  }
  
  // End CAN network transmission
  EposNetwork_Stop();
  
  DEBUG_EVENT( 0, "AES Control ended on thread %x", THREAD_ID );
}

int AESControl_GetAxisID( const char* axisName )
{
  for( size_t deviceID = 0; deviceID < devicesNumber; deviceID++ )
  {
    if( strcmp( controllersList[ deviceID ].name, axisName ) == 0 )
      return (int) deviceID;
  }
  
  return -1;
}

extern inline size_t AESControl_GetDevicesNumber()
{
  return devicesNumber;
}

static inline bool CheckController( size_t deviceID )
{
  if( (int) deviceID < 0 || deviceID >= devicesNumber ) return false;
  
  if( controllersList[ deviceID ].actuator == NULL ) return false;
  
  return true;
}

extern inline void AESControl_EnableMotor( size_t deviceID )
{
  if( !CheckController( deviceID ) ) return;
  
  if( controllersList[ deviceID ].ref_RunControl != NULL )
    Motor_Enable( controllersList[ deviceID ].actuator );
  
  controllersList[ deviceID ].measuresList[ CONTROL_ERROR ] = 0.0;
}

extern inline void AESControl_DisableMotor( size_t deviceID )
{
  if( !CheckController( deviceID ) ) return;
  
  Motor_Disable( controllersList[ deviceID ].actuator );
  
  controllersList[ deviceID ].measuresList[ CONTROL_ERROR ] = 0.0;
}

extern inline void AESControl_Reset( size_t deviceID )
{
  if( !CheckController( deviceID ) ) return;
  
  if( MotorDrive_CheckState( controllersList[ deviceID ].actuator->drive, FAULT ) )
  {
    MotorDrive_Reset( controllersList[ deviceID ].actuator->drive );
    MotorDrive_Reset( controllersList[ deviceID ].sensor );
  }
  
  controllersList[ deviceID ].measuresList[ CONTROL_ERROR ] = 0.0;
}

extern inline bool* AESControl_GetMotorStatus( size_t deviceID )
{
  static bool statesList[ AXIS_STATES_NUMBER ];
  
  if( !CheckController( deviceID ) ) return NULL;
  
  for( size_t stateIndex = 0; stateIndex < AXIS_STATES_NUMBER; stateIndex++ )
    statesList[ stateIndex ] = MotorDrive_CheckState( controllersList[ deviceID ].actuator->drive, stateIndex );
  
  return (bool*) statesList;
}

extern inline double* AESControl_GetMeasuresList( size_t deviceID )
{
  if( !CheckController( deviceID ) ) return NULL;

  return (double*) controllersList[ deviceID ].measuresList;
}

extern inline double* AESControl_GetParametersList( size_t deviceID )
{
  if( !CheckController( deviceID ) ) return NULL;

  return (double*) controllersList[ deviceID ].parametersList;
}

extern inline void AESControl_SetSetpoint( size_t deviceID, double setpoint )
{
  if( !CheckController( deviceID ) ) return;
  
  AESController* controller = &(controllersList[ deviceID ]);
  double* parametersList = &(controllersList[ deviceID ].parametersList);
  Splined3Curve* parameterCurvesList = &(controllersList[ deviceID ].parameterCurvesList);
  
  parametersList[ CONTROL_SETPOINT ] = Spline3Interp_GetValue( parameterCurvesList[ CONTROL_SETPOINT ], setpoint );
  if( parametersList[ CONTROL_SETPOINT ] > controller->maxReach ) parametersList[ CONTROL_SETPOINT ] = controller->maxReach;
  else if( parametersList[ CONTROL_SETPOINT ] < controller->minReach ) parametersList[ CONTROL_SETPOINT ] = controller->minReach;
  
  parametersList[ CONTROL_STIFFNESS ] = controller->maxStiffness * Spline3Interp_GetValue( parameterCurvesList[ CONTROL_STIFFNESS ], setpoint );
  if( parametersList[ CONTROL_STIFFNESS ] > controller->maxStiffness ) parametersList[ CONTROL_STIFFNESS ] = controller->maxStiffness;
  else if( parametersList[ CONTROL_STIFFNESS ] < 0.0 ) parametersList[ CONTROL_STIFFNESS ] = 0.0;
  
  parametersList[ CONTROL_DAMPING ] = controller->maxDamping * Spline3Interp_GetValue( parameterCurvesList[ CONTROL_DAMPING ], setpoint );
  if( parametersList[ CONTROL_DAMPING ] > controller->maxDamping ) parametersList[ CONTROL_DAMPING ] = controller->maxDamping;
  else if( parametersList[ CONTROL_DAMPING ] < 0.0 ) parametersList[ CONTROL_DAMPING ] = 0.0;
}

extern inline void AESControl_SetImpedance( size_t deviceID, double referenceStiffness, double referenceDamping )
{
  if( !CheckController( deviceID ) ) return;
  
  AESController* controller = &(controllersList[ deviceID ]);
  
  if( referenceStiffness < 0.0 ) referenceStiffness = 0.0;
  else if( referenceStiffness > controller->physicalStiffness ) referenceStiffness = controller->physicalStiffness;
  controller->maxStiffness = referenceStiffness;
  
  controller->maxDamping = ( referenceDamping > 0.0 ) ? referenceDamping : 0.0;
}

extern inline const char* AESControl_GetDeviceName( size_t deviceID )
{
  if( !CheckController( deviceID ) ) return NULL;
  
  return (char*) controllersList[ deviceID ].name;
}

/////////////////////////////////////////////////////////////////////////////////
/////                           COMMON CONSTANTS                            /////
/////////////////////////////////////////////////////////////////////////////////

#ifndef PI
const double PI = 3.141592;        // Duh...
#endif

const double CONTROL_SAMPLING_INTERVAL = 0.005;           // Sampling interval

// Method that runs the control functions asyncronously
static void* AsyncControl( void* args )
{
  AESController* controller = (AESController*) args;
  
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
      
      MotorDrive_ReadValues( controller->sensor );
      if( controller->actuator->drive != controller->sensor ) MotorDrive_ReadValues( controller->actuator->drive );

      double sensorPosition = MotorDrive_GetMeasure( controller->sensor, AXIS_POSITION );

      double* filteredMeasures = SimpleKalman_Update( controller->positionFilter, sensorPosition, CONTROL_SAMPLING_INTERVAL );

      controller->measuresList[ CONTROL_POSITION ] = filteredMeasures[ KALMAN_VALUE ];// * ( 2 * PI );
      controller->measuresList[ CONTROL_VELOCITY ] = filteredMeasures[ KALMAN_DERIVATIVE ];// * ( 2 * PI );
      controller->measuresList[ CONTROL_ACCELERATION ] = filteredMeasures[ KALMAN_DERIVATIVE_2 ];// * ( 2 * PI );

      double actuatorPosition = MotorDrive_GetMeasure( controller->actuator->drive, AXIS_POSITION );
      controller->measuresList[ CONTROL_FORCE ] = controller->physicalStiffness * ( actuatorPosition - sensorPosition );// * ( 2 * PI );

      // If the motor is being actually controlled, call control pass algorhitm
      if( controller->actuator->active ) 
      {
        double controlOutput = controller->ref_RunControl( controller->measuresList, controller->parametersList, CONTROL_SAMPLING_INTERVAL );
        Motor_SetSetpoint( controller->actuator, controller->operationMode, controlOutput );  
      }
  
      Motor_WriteConfig( controller->actuator );
      
      elapsedTime = Timing_GetExecTimeMilliseconds() - execTime;
      DEBUG_UPDATE( "control pass for %s AES (before delay): elapsed time: %u ms", controller->name, elapsedTime );
      
      if( elapsedTime < (int) ( 1000 * CONTROL_SAMPLING_INTERVAL ) ) Timing_Delay( 1000 * CONTROL_SAMPLING_INTERVAL - elapsedTime );

      DEBUG_UPDATE( "control pass for %s AES (after delay): elapsed time: %u ms", controller->name, Timing_GetExecTimeMilliseconds() - execTime );
    }
  }
  
  Thread_Exit( 0 );
  return NULL;
}

#endif /* CONTROL_H */ 
