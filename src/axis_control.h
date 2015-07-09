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

typedef struct _AxisControl
{
  char name[ DEVICE_NAME_MAX_LENGTH ];
  Motor* actuator;                                                  // Active axis
  MotorDrive* sensor;                                               // Passive (extra measurement) axis
  ImpedanceControlFunction* controlFunction;                             // Control method definition structure
  Thread_Handle updateThread;                                       // Processing thread handle
  double measuresList[ CONTROL_DIMS_NUMBER ];
  SimpleKalmanFilter* positionFilter;
  double parametersList[ CONTROL_PARAMS_NUMBER ];
  double parameterPointsList[ CONTROL_PARAMS_NUMBER ];
  Splined3CurveData* parameterCurvesList[ CONTROL_PARAMS_NUMBER ];
  double physicalStiffness;                                         // Real stiffness and damping of the actuator
  bool isRunning;                                                   // Is control thread running ?
}
AxisControl;

static AxisControl* axisControlsList = NULL;
static size_t axesNumber = 0;

static void* AsyncControl( void* );

// Axes configuration loading auxiliary function
static void LoadAxisControlsConfig()
{
  char readBuffer[ 64 ];
  
  AxisControl* newAxisControl = NULL;
  
  FILE* configFile = fopen( "../config/aes_controls_config.txt", "r" );
  if( configFile != NULL ) 
  {
    while( fscanf( configFile, "%s", readBuffer ) != EOF )
    {
      if( strcmp( readBuffer, "BEGIN_AXIS_CONTROL" ) == 0 )
      {
        axisControlsList = (AxisControl*) realloc( axisControlsList, sizeof(AxisControl) * ( axesNumber + 1 ) );
  
        newAxisControl = &(axisControlsList[ axesNumber ]);
        
        fscanf( configFile, "%s", newAxisControl->name );
  
        DEBUG_EVENT( 0, "found axis control %s", newAxisControl->name );
        
        newAxisControl->actuator = NULL;
        newAxisControl->sensor = NULL;
        for( size_t dimensionIndex = 0; dimensionIndex < CONTROL_DIMS_NUMBER; dimensionIndex++ )
          newAxisControl->measuresList[ dimensionIndex ] = 0.0;
        for( size_t parameterIndex = 0; parameterIndex < CONTROL_PARAMS_NUMBER; parameterIndex++ )
        {
          newAxisControl->parametersList[ parameterIndex ] = 0.0;
          newAxisControl->parameterPointsList[ parameterIndex ] = 0.0;
          newAxisControl->parameterCurvesList[ parameterIndex ] = NULL;
        }
        newAxisControl->physicalStiffness = 0.0;
        newAxisControl->controlFunction = NULL;
        newAxisControl->updateThread = -1;
      }
      
      if( newAxisControl == NULL ) continue;
      
      if( strcmp( readBuffer, "motor:" ) == 0 )
      {
        unsigned int nodeID, encoderResolution;
        double gearReduction;
        fscanf( configFile, "%u %u %lf", &nodeID, &encoderResolution, &gearReduction );
        
        DEBUG_EVENT( 1, "found %s motor on node ID %u with encoder resolution %u and gear reduction %g", newAxisControl->name, nodeID, encoderResolution, gearReduction );
        
        newAxisControl->actuator = Motor_Connect( nodeID );
        MotorDrive_SetEncoderResolution( newAxisControl->actuator->controller, encoderResolution );
        MotorDrive_SetGearReduction( newAxisControl->actuator->controller, gearReduction );
      }
      else if( strcmp( readBuffer, "encoder:" ) == 0 )
      {
        unsigned int nodeID, encoderResolution;
        double gearReduction;
        fscanf( configFile, "%u %u %lf", &nodeID, &encoderResolution, &gearReduction );
        
        DEBUG_EVENT( 2, "found %s sensor on node ID %u with encoder resolution %u and gear reduction %g", newAxisControl->name, nodeID, encoderResolution, gearReduction );
        
        newAxisControl->sensor = MotorDrive_Connect( nodeID );
        MotorDrive_SetEncoderResolution( newAxisControl->sensor, encoderResolution );
        MotorDrive_SetGearReduction( newAxisControl->sensor, gearReduction );
      }
      else if( strcmp( readBuffer, "base_impedance:" ) == 0 )
      {
        fscanf( configFile, "%lf %lf", &(newAxisControl->baseStiffness), &(newAxisControl->baseDamping) );
        
        DEBUG_EVENT( 4, "found %s base impedance values: ( k: %g - b: %g )", newAxisControl->name, newAxisControl->baseStiffness, newAxisControl->baseDamping );
        
        if( newAxisControl->baseStiffness < 0.0 ) newAxisControl->baseStiffness = 0.0;
        if( newAxisControl->baseDamping < 0.0 ) newAxisControl->baseDamping = 0.0;
      }
      else if( strcmp( readBuffer, "function:" ) == 0 )
      {
        fscanf( configFile, "%s", readBuffer );
        
        DEBUG_EVENT( 3, "setting %s control function to %s", newAxisControl->name, readBuffer );
        
        newAxisControl->controlFunction = ImpedanceControl_GetFunction( readBuffer );
      }
      else if( strcmp( readBuffer, "parameter_curves:" ) == 0 )
      {
        for( size_t parameterIndex = 0; parameterIndex < CONTROL_PARAMS_NUMBER; parameterIndex++ )
        {
          fscanf( configFile, "%s", readBuffer );
        
          DEBUG_EVENT( 6, "setting %s parameter curve %s", newAxisControl->name, readBuffer );
        
          newAxisControl->parameterCurvesList[ parameterIndex ] = Splined3Curves_Load( readBuffer );
        }
      }
      else if( strcmp( readBuffer, "END_AXIS_CONTROL" ) == 0 )
      {
        newAxisControl->updateThread = Thread_Start( AsyncControl, (void*) newAxisControl, THREAD_JOINABLE );
        
        DEBUG_EVENT( 5, "running %s control on thread %x", newAxisControl->name, newAxisControl->updateThread );
        
        if( newAxisControl->actuator == NULL || newAxisControl->updateThread == -1 )
        {
          newAxisControl->isRunning = false;
    
          Thread_WaitExit( newAxisControl->updateThread, 5000 );
    
          Motor_Disconnect( newAxisControl->actuator );
          MotorDrive_Disconnect( newAxisControl->sensor );
        }
        else
        {
          if( newAxisControl->sensor == NULL )
            newAxisControl->sensor = newAxisControl->actuator->controller;
          
          newAxisControl->positionFilter = SimpleKalman_CreateFilter( 0.0 );
          
          axesNumber++;
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
  DEBUG_EVENT( 0, "Initializing Axis Control on thread %x", THREAD_ID );
  
  EposNetwork_Start( CAN_DATABASE, CAN_CLUSTER );
  
  LoadAxisControlsConfig();
  
  DEBUG_EVENT( 0, "Axis Control initialized on thread %x", THREAD_ID );
}

// EPOS devices and CAN network shutdown
void AxisControl_End()
{
  DEBUG_EVENT( 0, "Ending Axis Control on thread %x", THREAD_ID );

  Timing_Delay( 2000 );
  
  if( axisControlsList != NULL )
  {
    // Destroy axes data structures
    for( size_t axisID = 0; axisID < axesNumber; axisID++ )
    {
      axisControlsList[ axisID ].isRunning = false;
    
      if( axisControlsList[ axisID ].updateThread != -1 )
        Thread_WaitExit( axisControlsList[ axisID ].updateThread, 5000 );
    
      Motor_Disconnect( axisControlsList[ axisID ].actuator );
      MotorDrive_Disconnect( axisControlsList[ axisID ].sensor );
      
      SimpleKalman_DiscardFilter( axisControlsList[ axisID ].positionFilter );
    }
  
    free( axisControlsList );
  }
  
  // End CAN network transmission
  EposNetwork_Stop();
  
  DEBUG_EVENT( 0, "Axis Control ended on thread %x", THREAD_ID );
}

int AxisControl_GetAxisID( const char* axisName )
{
  for( size_t axisID = 0; axisID < axesNumber; axisID++ )
  {
    if( strcmp( axisControlsList[ axisID ].name, axisName ) == 0 )
      return (int) axisID;
  }
  
  return -1;
}

extern inline size_t AxisControl_GetActiveAxesNumber()
{
  return axesNumber;
}

static inline bool CheckAxis( size_t axisID )
{
  if( (int) axisID < 0 || axisID >= axesNumber ) return false;
  
  if( axisControlsList[ axisID ].actuator == NULL ) return false;
  
  return true;
}

extern inline void AxisControl_EnableMotor( size_t axisID )
{
  if( (int) axisID < 0 || axisID >= axesNumber ) return;
  
  if( axisControlsList[ axisID ].actuator == NULL ) return;
  
  if( axisControlsList[ axisID ].controlFunction != NULL )
    Motor_Enable( axisControlsList[ axisID ].actuator );
  
  axisControlsList[ axisID ].measuresList[ CONTROL_ERROR ] = 0.0;
}

extern inline void AxisControl_DisableMotor( size_t axisID )
{
  if( (int) axisID < 0 || axisID >= axesNumber ) return;
  
  if( axisControlsList[ axisID ].actuator == NULL ) return;
  
  Motor_Disable( axisControlsList[ axisID ].actuator );
  
  axisControlsList[ axisID ].measuresList[ CONTROL_ERROR ] = 0.0;
}

extern inline void AxisControl_Reset( size_t axisID )
{
  if( (int) axisID < 0 || axisID >= axesNumber ) return;
  
  if( axisControlsList[ axisID ].actuator == NULL ) return;
  
  if( MotorDrive_CheckState( axisControlsList[ axisID ].actuator->controller, FAULT ) )
  {
    MotorDrive_Reset( axisControlsList[ axisID ].actuator->controller );
    MotorDrive_Reset( axisControlsList[ axisID ].sensor );
  }
  
  axisControlsList[ axisID ].measuresList[ CONTROL_ERROR ] = 0.0;
}

extern inline bool* AxisControl_GetMotorStatus( size_t axisID )
{
  static bool statesList[ AXIS_STATES_NUMBER ];
  
  if( (int) axisID < 0 || axisID >= axesNumber ) return NULL;
  
  if( axisControlsList[ axisID ].actuator == NULL ) return NULL;
  
  for( size_t stateIndex = 0; stateIndex < AXIS_STATES_NUMBER; stateIndex++ )
    statesList[ stateIndex ] = MotorDrive_CheckState( axisControlsList[ axisID ].actuator->controller, stateIndex );
  
  return (bool*) statesList;
}

extern inline double* AxisControl_GetMeasuresList( size_t axisID )
{
  if( (int) axisID < 0 || axisID >= axesNumber ) return NULL;

  return (double*) axisControlsList[ axisID ].measuresList;
}

extern inline double* AxisControl_GetParametersList( size_t axisID )
{
  if( (int) axisID < 0 || axisID >= axesNumber ) return NULL;

  return (double*) axisControlsList[ axisID ].parametersList;
}

extern inline void AxisControl_SetSetpoint( size_t axisID, double setpoint )
{
  if( (int) axisID < 0 || axisID >= axesNumber ) return;
  
  if( axisControlsList[ axisID ].actuator == NULL ) return;
  
  axisControlsList[ axisID ].parameterPointsList[ CONTROL_SETPOINT ] = setpoint;
}

extern inline void AxisControl_SetImpedance( size_t axisID, double referenceStiffness, double referenceDamping )
{
  if( (int) axisID < 0 || axisID >= axesNumber ) return;
  
  if( axisControlsList[ axisID ].actuator == NULL ) return;
  
  AxisControl* axisControl = &(axisControlsList[ axisID ]);
  
  if( referenceStiffness < 0.0 ) referenceStiffness = 0.0;
  else if( referenceStiffness > axisControl->physicalStiffness ) referenceStiffness = axisControl->physicalStiffness;
  axisControl->parameterPointsList[ CONTROL_STIFFNESS ] = referenceStiffness;
  
  axisControl->parameterPointsList[ CONTROL_DAMPING ] = ( referenceDamping > 0.0 ) ? referenceDamping : 0.0;
}

extern inline double AxisControl_GetError( size_t axisID )
{
  if( (int) axisID < 0 || axisID >= axesNumber ) return 0.0;

  return axisControlsList[ axisID ].error;
}

extern inline const char* AxisControl_GetAxisName( size_t axisID )
{
  if( (int) axisID < 0 || axisID >= axesNumber ) return NULL;
  
  return (char*) axisControlsList[ axisID ].name;
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
  AxisControl* axisControl = (AxisControl*) args;
  
  // Try to correct errors
  if( axisControl->actuator != NULL ) 
  {
    MotorDrive_Reset( axisControl->actuator->controller );
    MotorDrive_Reset( axisControl->sensor );
    
    Motor_SetOperationMode( axisControl->actuator, axisControl->controlFunction->operationMode );
  
    unsigned long execTime, elapsedTime;
  
    axisControl->isRunning = true;
  
    DEBUG_EVENT( 0, "starting to run control for Axis %s", axisControl->name );
  
    while( axisControl->isRunning )
    {
      DEBUG_UPDATE( "running control for Axis %s", axisControl->name );
    
      execTime = Timing_GetExecTimeMilliseconds();
      
      MotorDrive_ReadValues( axisControl->sensor );
      if( axisControl->actuator->controller != axisControl->sensor ) MotorDrive_ReadValues( axisControl->actuator->controller );

      double sensorPosition = MotorDrive_GetMeasure( axisControl->sensor, AXIS_POSITION );

      double* filteredMeasures = SimpleKalman_Update( axisControl->positionFilter, sensorPosition, CONTROL_SAMPLING_INTERVAL );

      axisControl->measuresList[ CONTROL_POSITION ] = filteredMeasures[ KALMAN_VALUE ] * ( 2 * PI );
      axisControl->measuresList[ CONTROL_VELOCITY ] = filteredMeasures[ KALMAN_DERIVATIVE ] * ( 2 * PI );
      axisControl->measuresList[ CONTROL_ACCELERATION ] = filteredMeasures[ KALMAN_DERIVATIVE_2 ] * ( 2 * PI );

      double actuatorPosition = MotorDrive_GetMeasure( axisControl->actuator->controller, AXIS_POSITION );
      axisControl->measuresList[ CONTROL_FORCE ] = axisControl->physicalStiffness * ( actuatorPosition - sensorPosition ) * ( 2 * PI );

      for( parameterIndex = 0; parameterIndex < CONTROL_PARAMS_NUMBER; parameterIndex++ )
      {
        axisControl->parametersList[ parameterIndex ] = Splined3Curve_GetValue( axisControl->parameterCurvesList[ parameterIndex ],
                                                                                axisControl->parameterPointsList[ CONTROL_SETPOINT ], 
                                                                                axisControl->parameterPointsList[ parameterIndex ] );
      }

      // If the motor is being actually controlled, call control pass algorhitm
      if( axisControl->actuator->active ) 
      {
        double controlOutput = axisControl->controlFunction->ref_Run( axisControl->measuresList, axisControl->parametersList, CONTROL_SAMPLING_INTERVAL );
        Motor_SetSetpoint( axisControl->actuator, axisControl->controlFunction->operationMode, controlOutput );  
      }
  
      Motor_WriteConfig( axisControl->actuator );
      
      elapsedTime = Timing_GetExecTimeMilliseconds() - execTime;
      DEBUG_UPDATE( "control pass for Axis %s (before delay): elapsed time: %u ms", axisControl->name, elapsedTime );
      
      if( elapsedTime < (int) ( 1000 * CONTROL_SAMPLING_INTERVAL ) ) Timing_Delay( 1000 * CONTROL_SAMPLING_INTERVAL - elapsedTime );

      DEBUG_UPDATE( "control pass for Axis %s (after delay): elapsed time: %u ms", axisControl->name, Timing_GetExecTimeMilliseconds() - execTime );
    }
  }
  
  Thread_Exit( 0 );
  return NULL;
}

#endif /* CONTROL_H */ 
