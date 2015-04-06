#ifndef CONTROL_H
#define CONTROL_H

#include "axis.h"

#include "async_debug.h"

#include <math.h>

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
  Motor* actuator;                                     // Active axis
  MotorDrive* sensor;                                  // Passive (extra measurement) axis
  void (*ref_RunAxisControl)( Motor*, MotorDrive* );   // Control pass algorithm (function pointer)
  uint8_t operationMode;                               // Actuator operation mode (Position, velocity or current)
  Thread_Handle controlThread;                         // Processing thread handle
  bool isRunning;                                      // Is control thread running ?
}
AxisControl;

static AxisControl* axisControlsList = NULL;
static size_t axesNumber = 0;

// Control algorhitms
static void AnkleControl( Motor*, MotorDrive* );
static void KneeControl( Motor*, MotorDrive* );
static void HipsControl( Motor*, MotorDrive* );

typedef struct _ControlFunction
{
  char* name;
  uint8_t operationMode;
  void (*ref_Run)( Motor*, MotorDrive* );
}
ControlFunction; 

ControlFunction controlFunctionsList[] = { { "Ankle", VELOCITY_MODE, AnkleControl }, 
                                           { "Knee", VELOCITY_MODE, KneeControl }, 
                                           { "Hips", VELOCITY_MODE, HipsControl } };

static void* AsyncControl( void* );

// Axes configuration loading auxiliary function
static void LoadAxisControlsConfig()
{
  char readBuffer[ DEVICE_NAME_MAX_LENGTH ];
  
  AxisControl* newAxisControl = NULL;
  
  FILE* configFile = fopen( "../config/axis_control_config.txt", "r" );
  if( configFile != NULL ) 
  {
    while( fscanf( configFile, "%s", readBuffer ) != EOF )
    {
      if( strcmp( readBuffer, "CONTROL" ) == 0 )
      {
        axisControlsList = (AxisControl*) realloc( axisControlsList, sizeof(AxisControl) * ( axesNumber + 1 ) );
  
        newAxisControl = &(axisControlsList[ axesNumber ]);
        
        fscanf( configFile, "%s", newAxisControl->name );
  
        DEBUG_EVENT( 0, "found axis control %s", newAxisControl->name );
        
        newAxisControl->actuator = NULL;
        newAxisControl->sensor = NULL;
        newAxisControl->ref_RunAxisControl = NULL;
        newAxisControl->controlThread = -1;
      }
      
      if( newAxisControl == NULL ) continue;
      
      if( strcmp( readBuffer, "MOTOR" ) == 0 )
      {
        unsigned int nodeID, resolution;
        fscanf( configFile, "%u %u", &nodeID, &resolution );
        
        DEBUG_EVENT( 1, "found %s motor on node ID %u with resolution %u", newAxisControl->name, nodeID, resolution );
        
        newAxisControl->actuator = Motor_Connect( nodeID );
        MotorDrive_SetEncoderResolution( newAxisControl->actuator->controller, resolution );
      }
      else if( strcmp( readBuffer, "ENCODER" ) == 0 )
      {
        unsigned int nodeID, resolution;
        fscanf( configFile, "%u %u", &nodeID, &resolution );
        
        DEBUG_EVENT( 2, "found %s sensor on node ID %u with resolution %u", newAxisControl->name, nodeID, resolution );
        
        newAxisControl->sensor = MotorDrive_Connect( nodeID );
        MotorDrive_SetEncoderResolution( newAxisControl->sensor, resolution ); 
      }
      else if( strcmp( readBuffer, "FUNCTION" ) == 0 )
      {
        fscanf( configFile, "%s", readBuffer );
        
        DEBUG_EVENT( 3, "setting %s control function to %s", newAxisControl->name, readBuffer );
        
        size_t functionsNumber = sizeof(controlFunctionsList) / sizeof(ControlFunction);
        for( size_t functionID = 0; functionID < functionsNumber; functionID++ )
        {
          if( strcmp( readBuffer, controlFunctionsList[ functionID ].name ) == 0 )
          {
            newAxisControl->ref_RunAxisControl = controlFunctionsList[ functionID ].ref_Run;
            newAxisControl->operationMode = controlFunctionsList[ functionID ].operationMode;
            break;
          }
        }
      }
      else if( strcmp( readBuffer, "ENDCONTROL" ) == 0 )
      {
        newAxisControl->controlThread = Thread_Start( AsyncControl, (void*) newAxisControl, JOINABLE );
        
        DEBUG_EVENT( 4, "running %s control on thread %x", newAxisControl->name, newAxisControl->controlThread );
        
        if( newAxisControl->actuator == NULL || newAxisControl->ref_RunAxisControl == NULL || newAxisControl->controlThread == -1 )
        {
          newAxisControl->isRunning = false;
    
          if( newAxisControl->controlThread != -1 )
            Thread_WaitExit( newAxisControl->controlThread, 5000 );
    
          Motor_Disconnect( newAxisControl->actuator );
          MotorDrive_Disconnect( newAxisControl->sensor );
        }
        else
        {
          if( newAxisControl->sensor == NULL )
            newAxisControl->sensor = newAxisControl->actuator->controller;
          
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
  
  // Destroy axes data structures
  for( size_t axisID = 0; axisID < axesNumber; axisID++ )
  {
    axisControlsList[ axisID ].isRunning = false;
    
    if( axisControlsList[ axisID ].controlThread != -1 )
      Thread_WaitExit( axisControlsList[ axisID ].controlThread, 5000 );
    
    Motor_Disconnect( axisControlsList[ axisID ].actuator );
    MotorDrive_Disconnect( axisControlsList[ axisID ].sensor );
  }
  
  // End CAN network transmission
  EposNetwork_Stop();
  
  DEBUG_EVENT( 0, "Axis Control ended on thread %x", THREAD_ID );
}

extern inline size_t AxisControl_GetActiveAxesNumber()
{
  return axesNumber;
}

extern inline void AxisControl_EnableMotor( size_t axisID )
{
  if( axisID < 0 || axisID >= axesNumber ) return;
  
  if( axisControlsList[ axisID ].actuator != NULL )
    Motor_Enable( axisControlsList[ axisID ].actuator );
}

extern inline void AxisControl_DisableMotor( size_t axisID )
{
  if( axisID < 0 || axisID >= axesNumber ) return;
  
  if( axisControlsList[ axisID ].actuator != NULL )
    Motor_Disable( axisControlsList[ axisID ].actuator );
}

extern inline void AxisControl_Reset( size_t axisID )
{
  if( axisID < 0 || axisID >= axesNumber ) return;
  
  if( axisControlsList[ axisID ].actuator != NULL )
  {
    MotorDrive_Reset( axisControlsList[ axisID ].actuator->controller );
    MotorDrive_Reset( axisControlsList[ axisID ].sensor );
  }
}

extern inline int AxisControl_ConfigMotor( size_t axisID, double parametersList[ AXIS_N_PARAMS ] )
{
  if( axisID < 0 || axisID >= axesNumber ) return -1;
  
  if( axisControlsList[ axisID ].actuator == NULL ) return 0;
  
  for( size_t parameterIndex = 0; parameterIndex < AXIS_N_PARAMS; parameterIndex++ )
    Motor_SetParameter( axisControlsList[ axisID ].actuator, parameterIndex, parametersList[ parameterIndex ] );
  
  return (int) AXIS_N_PARAMS;
}

extern inline bool* AxisControl_GetMotorStatus( size_t axisID )
{
  static bool statesList[ AXIS_N_STATES ];
  
  if( axisID < 0 || axisID >= axesNumber ) return NULL;
  
  if( axisControlsList[ axisID ].actuator == NULL ) return NULL;
  
  for( size_t stateIndex = 0; stateIndex < AXIS_N_STATES; stateIndex++ )
    statesList[ stateIndex ] = MotorDrive_CheckState( axisControlsList[ axisID ].actuator->controller, stateIndex );
  
  return (bool*) statesList;
}

extern inline double* AxisControl_GetSensorMeasures( size_t axisID )
{
  static double measuresList[ AXIS_N_DIMS ];
  
  if( axisID < 0 || axisID >= axesNumber ) return NULL;
  
  if( axisControlsList[ axisID ].actuator == NULL ) return NULL;
  
  for( size_t dimensionIndex = 0; dimensionIndex < AXIS_N_DIMS; dimensionIndex++ )
    measuresList[ dimensionIndex ] = MotorDrive_GetMeasure( axisControlsList[ axisID ].sensor, dimensionIndex );
  
  return (double*) measuresList;
}

extern inline const char* AxisControl_GetAxisName( size_t axisID )
{
  if( axisID < 0 || axisID >= axesNumber ) return NULL;
  
  return axisControlsList[ axisID ].name;
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
    
    Motor_SetOperationMode( axisControl->actuator, axisControl->operationMode );
  
    unsigned int execTime, elapsedTime;
  
    axisControl->isRunning = true;
    
    //int axisLogID = DataLogging_CreateLog( axisControl->name, 4 );
  
    DEBUG_EVENT( 0, "starting to run control for Axis %s", axisControl->name );
  
    while( axisControl->isRunning )
    {
      DEBUG_UPDATE( "running control for Axis %s", axisControl->name );
    
      execTime = Timing_GetExecTimeMilliseconds();

      MotorDrive_ReadValues( axisControl->sensor );
      if( axisControl->actuator->controller != axisControl->sensor ) MotorDrive_ReadValues( axisControl->actuator->controller );
    
      // If the motor is being actually controlled, call control pass algorhitm
      if( axisControl->actuator->active ) axisControl->ref_RunAxisControl( axisControl->actuator, axisControl->sensor );

      Motor_WriteConfig( axisControl->actuator );
    
      /*DataLogging_Register( axisLogID, MotorDrive_GetMeasure( axisControl->actuator->controller, POSITION ), 
                                       MotorDrive_GetMeasure( axisControl->actuator->controller, VELOCITY ),
                                       ( axisControl->sensor != NULL ) ? MotorDrive_GetMeasure( axisControl->sensor, POSITION ) : 0.0, 
                                       ( axisControl->sensor != NULL ) ? MotorDrive_GetMeasure( axisControl->sensor, VELOCITY ) : 0.0 );*/
      
      elapsedTime = Timing_GetExecTimeMilliseconds() - execTime;
      DEBUG_UPDATE( "control pass for Axis %s (before delay): elapsed time: %u", axisControl->name, elapsedTime );
      
      if( elapsedTime < (int) ( 1000 * CONTROL_SAMPLING_INTERVAL ) ) Timing_Delay( 1000 * CONTROL_SAMPLING_INTERVAL - elapsedTime );
    
      DEBUG_UPDATE( "control pass for Axis %s (after delay): elapsed time: %u", axisControl->name, Timing_GetExecTimeMilliseconds() - execTime );
    }
    
    //DataLogging_CloseLog( axisLogID );
  }
  
  Thread_Exit( 0 );
  return NULL;
}


/////////////////////////////////////////////////////////////////////////////////
/////                            HIPS CONSTANTS                             /////
/////////////////////////////////////////////////////////////////////////////////

const double TNS_2_POS_RATIO = 0.0168; // mm/mV
const double TNS_2_POS_OFFSET = 2.5749; // mm

const double HIPS_FORCE_SENSOR_STIFFNESS = 78.9; // N/mm
const double HIPS_ACTUATOR_STEP = 3e-3; // Linear effector advance per screw rotation (in meters/rotation)

// Orthosis structure bars and diagonals lengths (in meters)
enum { A, B, C, D, C2D, A2B };
//const double HIPS_STRUCT_DIMS[] = { 0.063, 0.060, 0.284, 0.117, sqrt( 0.284 * 0.284 + 0.117 * 0.117 ), sqrt( 0.063 * 0.063 + 0.060 * 0.060 ) };
const double HIPS_STRUCT_DIMS[] = { 0.063, 0.060, 0.284, 0.117, 0.307, 0.087 };
// Actuator length with effector at initial position (in meters)
const double HIPS_EFFECTOR_BASE_LENGTH = 0.348;

enum { ALPHA, BETA };
//const double HIPS_STRUCT_ANGLES[] = { atan( HIPS_STRUCT_DIMS[ A ] / HIPS_STRUCT_DIMS[ B ] ), atan( HIPS_STRUCT_DIMS[ C ] / HIPS_STRUCT_DIMS[ D ] ) };
const double HIPS_STRUCT_ANGLES[] = { 0.810, 1.180 };

/////////////////////////////////////////////////////////////////////////////////
/////                    HIPS CONTROL DEFAULT VALUES                        /////
/////////////////////////////////////////////////////////////////////////////////

// Virtual impedance gains
double Kq = 10.0; // Stiffness
double Bq = 10.0; // Damping

// Sampling
const int HIPS_CONTROL_SAMPLING_NUMBER = 6;

/////////////////////////////////////////////////////////////////////////////////
/////                         HIPS CONTROL FUNCTION                         /////
/////////////////////////////////////////////////////////////////////////////////

static void HipsControl( Motor* hipsActuator, MotorDrive* hipsSensor )
{
  static double sensorTension_0, sensorPosition_0;

  // Sampling of last voltage signal values for filtering (average filter)
  static double analogSamples[ HIPS_CONTROL_SAMPLING_NUMBER ];

  static double angPositionInSetpoint;
    
  if( sensorTension_0 == 0 )
  {
    sensorTension_0 = MotorDrive_GetMeasure( hipsActuator->controller, TENSION ); // mV
    sensorPosition_0 = ( TNS_2_POS_RATIO * sensorTension_0 + TNS_2_POS_OFFSET ); // mm  
  }
  
  double sensorTension = 0;
  for( int i = 0; i < HIPS_CONTROL_SAMPLING_NUMBER; i ++ )
  {
    analogSamples[ i ] = ( i == 0 ) ? MotorDrive_GetMeasure( hipsActuator->controller, TENSION ) : analogSamples[ i - 1 ]; // mV
    sensorTension += analogSamples[ i ] / HIPS_CONTROL_SAMPLING_NUMBER;
  }

  double sensorPosition = ( TNS_2_POS_RATIO * sensorTension + TNS_2_POS_OFFSET );	// mm

  double actuatorForce = -HIPS_FORCE_SENSOR_STIFFNESS * ( sensorPosition_0 - sensorPosition ); // N

  angPositionInSetpoint = Motor_GetParameter( hipsActuator, POSITION_SETPOINT );
  printf( "Angle setpoint: %g\n", angPositionInSetpoint );

  // Relation between axis rotation and effector linear displacement
  double actuatorEncoderPosition = MotorDrive_GetMeasure( hipsActuator->controller, POSITION );
  double effectorDeltaLength = actuatorEncoderPosition * HIPS_ACTUATOR_STEP + ( sensorPosition_0 - sensorPosition ) / 1000; // m

  double effectorLength = HIPS_EFFECTOR_BASE_LENGTH + effectorDeltaLength; // m

  double cos_eta = ( pow( effectorLength, 2 ) + pow( HIPS_STRUCT_DIMS[ A2B ], 2 ) - pow( HIPS_STRUCT_DIMS[ C2D ], 2 ) )
				   / ( 2 * effectorLength * HIPS_STRUCT_DIMS[ A2B ] );
  double sin_eta = sqrt( (double)( 1 - pow( cos_eta, 2 ) ) );

  double gama = acos( ( pow( HIPS_STRUCT_DIMS[ C2D ], 2 ) + pow( HIPS_STRUCT_DIMS[ A2B ], 2 ) - pow( effectorLength, 2 ) )
					  / ( 2 * HIPS_STRUCT_DIMS[ C2D ] * HIPS_STRUCT_DIMS[ A2B ] ) );

  //double theta = HIPS_STRUCT_ANGLES[ ALPHA ] + HIPS_STRUCT_ANGLES[ BETA ] + gama - PI;

  // Current theta derivative with respect to the effector total length (base + delta)
  double lengthToThetaRatio = sqrt( pow( HIPS_STRUCT_DIMS[ C2D ], 2 ) + pow( HIPS_STRUCT_DIMS[ A2B ], 2 )
									  - 2 * HIPS_STRUCT_DIMS[ C2D ] * HIPS_STRUCT_DIMS[ A2B ] * cos( gama ) )
								/ ( HIPS_STRUCT_DIMS[ C2D ] * HIPS_STRUCT_DIMS[ A2B ] * sin( gama ) );

  Kq = Motor_GetParameter( hipsActuator, PROPORTIONAL_GAIN );
  Bq = Motor_GetParameter( hipsActuator, DERIVATIVE_GAIN );

  if( (int) Kq <= 0 ) Kq = 1.0;
  if( (int) Bq <= 0 ) Bq = 1.0;

  double actuatorStiffness = Kq * lengthToThetaRatio * ( 1 / ( HIPS_STRUCT_DIMS[ A2B ] * sin_eta ) );
  double actuatorDamping = Bq * lengthToThetaRatio * ( 1 / ( HIPS_STRUCT_DIMS[ A2B ] * sin_eta ) );

  double Fv_q = ( Kq * angPositionInSetpoint ) / ( HIPS_STRUCT_DIMS[ A2B ] * sin_eta );

  // Control law
  double velocitySetpoint = ( Fv_q - actuatorStiffness * ( actuatorEncoderPosition * HIPS_ACTUATOR_STEP )
						  + ( ( actuatorStiffness - HIPS_FORCE_SENSOR_STIFFNESS ) / HIPS_FORCE_SENSOR_STIFFNESS ) * actuatorForce ) / actuatorDamping;

  int velocityRPMSetpoint = ( velocitySetpoint / HIPS_ACTUATOR_STEP ) * 60 / ( 2 * PI ); // rpm

  Motor_SetParameter( hipsActuator, VELOCITY_SETPOINT, velocityRPMSetpoint );
}


/////////////////////////////////////////////////////////////////////////////////
/////                            KNEE CONSTANTS                             /////
/////////////////////////////////////////////////////////////////////////////////

const int KNEE_JOINT_REDUCTION = 150;       // Joint gear reduction ratio
const int KNEE_BASE_STIFFNESS = 94;        // Elastic constant for knee joint spring

// Torque Filter
const double a2 = -0.9428;
const double a3 = 0.3333;
const double b1 = 0.0976;
const double b2 = 0.1953;
const double b3 = 0.0976;

// Angular Velocity Filter
const double c2 = -1.889;
const double c3 = 0.8949;
const double d1 = 0.0015;
const double d2 = 0.0029;
const double d3 = 0.0015;

/////////////////////////////////////////////////////////////////////////////////
/////                          KNEE CONTROL FUNCTION                        /////
/////////////////////////////////////////////////////////////////////////////////

static void KneeControl( Motor* kneeIn, MotorDrive* kneeOut )
{
  static double angPositionOut[2];
  
  static double angVelocityOut[3];
  static double angVelOutFiltered[3];
  static double torque[3];
  static double torqueFiltered[3];
  static double error[3];

  static double angPositionIn, angPosInReduced, angPositionOutSetpoint;
  
  static double angVelocityInSetpoint[3];

  angPositionIn = ( MotorDrive_GetMeasure( kneeIn->controller, POSITION ) ) * 2 * PI;
  angPosInReduced = angPositionIn / KNEE_JOINT_REDUCTION;

  // Impedance control
  static double Kv, Bv;
  
  // Vitual Stiffness
  Kv = Motor_GetParameter( kneeIn, PROPORTIONAL_GAIN );
  if( Kv > KNEE_BASE_STIFFNESS ) Kv = KNEE_BASE_STIFFNESS;
  else if( Kv < 0.0 ) Kv = 0.0;
  
  // Virtual Damping
  Bv = Motor_GetParameter( kneeIn, DERIVATIVE_GAIN );
  if( Bv > 10.0 ) Bv = 10.0;
  else if( Bv < 0.0 ) Bv = 0.0;

  //MotorDrive_ReadValues( kneeOut );
  angPositionOut[0] = -MotorDrive_GetMeasure( kneeOut, POSITION ) * 2 * PI;

  angVelocityOut[0] = ( angPositionOut[0] - angPositionOut[1] ) / CONTROL_SAMPLING_INTERVAL;
  angPositionOut[1] = angPositionOut[0];

  angVelOutFiltered[0] = -c2 * angVelOutFiltered[1] - c3 * angVelOutFiltered[2] + d1 * angVelocityOut[0] + d2 * angVelocityOut[1] + d3 * angVelocityOut[2];

  angPositionOutSetpoint = -Motor_GetParameter( kneeIn, POSITION_SETPOINT ) * 2 * PI;
  double torqueSetpoint = -Kv * ( angPositionOut[0] - angPositionOutSetpoint ) - Bv * ( angVelOutFiltered[0] /*- angVelocityInSetpoint[0]*/ );

  torque[0] = KNEE_BASE_STIFFNESS * ( angPosInReduced - angPositionOut[0] );

  torqueFiltered[0] = -a2 * torqueFiltered[1] - a3 * torqueFiltered[2] + b1 * torque[0] + b2 * torque[1] + b3 * torque[2];

  error[0] = ( torqueSetpoint - torqueFiltered[0] );

  //angVelocityInSetpoint[0] += 370 * ( error[0] - error[1] ) + 3.5 * CONTROL_SAMPLING_INTERVAL * error[0] + ( 0.0 / CONTROL_SAMPLING_INTERVAL ) * ( error[0] - 2 * error[1] + error[2] );
  
  angVelocityInSetpoint[0] = 0.9884 * angVelocityInSetpoint[1] + 0.007935 * angVelocityInSetpoint[2] + 357.1 * error[1] - 355.8 * error[2]; //5ms
  
  for( int i = 2; i > 0; i-- )
  {
    error[ i ] = error[ i - 1 ];
    angVelocityOut[ i ] = angVelocityOut[ i - 1 ];
    angVelOutFiltered[ i ] = angVelOutFiltered[ i - 1 ];
    torqueFiltered[ i ] = torqueFiltered[ i - 1 ];
    angVelocityInSetpoint[ i ] = angVelocityInSetpoint[ i - 1 ];
  }
  
  Motor_SetParameter( kneeIn, VELOCITY_SETPOINT, angVelocityInSetpoint[0] );
}

/////////////////////////////////////////////////////////////////////////////////
/////                         ANKLE CONTROL FUNCTION                        /////
/////////////////////////////////////////////////////////////////////////////////

static void AnkleControl( Motor* ankleActuator, MotorDrive* ankleSensor )
{
  static int velocitySetpoint;

  if( Motor_GetParameter( ankleActuator, POSITION_SETPOINT ) > 1.0 ) Motor_SetParameter( ankleActuator, POSITION_SETPOINT, 1.0 );
  else if( Motor_GetParameter( ankleActuator, POSITION_SETPOINT ) < -1.0 ) Motor_SetParameter( ankleActuator, POSITION_SETPOINT, -1.0 );
  
  velocitySetpoint = -( MotorDrive_GetMeasure( ankleActuator->controller, POSITION ) - Motor_GetParameter( ankleActuator, POSITION_SETPOINT ) ) * 100;
  velocitySetpoint *= Motor_GetParameter( ankleActuator, PROPORTIONAL_GAIN );
  
  Motor_SetParameter( ankleActuator, VELOCITY_SETPOINT, velocitySetpoint );
}

#endif /* CONTROL_H */ 
