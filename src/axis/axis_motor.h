#ifndef AXIS_MOTOR_H
#define AXIS_MOTOR_H

#ifdef __cplusplus
    extern "C" {
#endif

#include "axis/axis_types.h"

#include "file_parsing/json_parser.h"

#include "klib/khash.h"
      
#include "debug/async_debug.h"

enum AxisMotorMeasures { MOTOR_POSITION, MOTOR_FORCE, MOTOR_MEASURES_NUMBER };
      
typedef struct _MotorData
{
  int axisID;
  AxisInterface interface;
  double measuresList[ MOTOR_MEASURES_NUMBER ];
  double measureOffsetsList[ MOTOR_MEASURES_NUMBER ];
  size_t operationMode;
  unsigned int encoderResolution;
  double currentToForceRatio;
  double gearReduction;
}
MotorData;

typedef MotorData* Motor;

KHASH_MAP_INIT_INT( MotorInt, Motor )
static khash_t( MotorInt )* motorsList = NULL;

/*static AxisMotor AxisMotor_Init( const char* );
static inline void AxisMotor_End( AxisMotor );
static inline void AxisMotor_Enable( AxisMotor );
static inline void AxisMotor_Disable( AxisMotor );
static inline void AxisMotor_Reset( AxisMotor );
static void AxisMotor_SetOffset( AxisMotor );
static inline bool AxisMotor_IsEnabled( AxisMotor );
static inline bool AxisMotor_HasError( AxisMotor );
static double* AxisMotor_ReadMeasures( AxisMotor );
static void AxisMotor_WriteControl( AxisMotor, double );*/

#define NAMESPACE AxisMotor

#define NAMESPACE_FUNCTIONS \
        NAMESPACE_FUNCTION( int, Init, const char* ) \
        NAMESPACE_FUNCTION( void, End, int ) \
        NAMESPACE_FUNCTION( void, Enable, int ) \
        NAMESPACE_FUNCTION( void, Disable, int ) \
        NAMESPACE_FUNCTION( void, Reset, int ) \
        NAMESPACE_FUNCTION( void, SetOffset, int ) \
        NAMESPACE_FUNCTION( bool, IsEnabled, int ) \
        NAMESPACE_FUNCTION( bool, HasError, int ) \
        NAMESPACE_FUNCTION( double*, ReadMeasures, int ) \
        NAMESPACE_FUNCTION( void, WriteControl, int, double )

#define NAMESPACE_FUNCTION( rvalue, name, _VA_ARGS_ ) static rvalue NAMESPACE##_##name( _VA_ARGS_ );
NAMESPACE_FUNCTIONS
#undef NAMESPACE_FUNCTION

#define NAMESPACE_FUNCTION( rvalue, name, _VA_ARGS_ ) rvalue (*name)( _VA_ARGS_ );
const struct { NAMESPACE_FUNCTIONS }
#undef NAMESPACE_FUNCTION
#define NAMESPACE_FUNCTION( rvalue, name, _VA_ARGS_ ) .name = NAMESPACE##_##name,
NAMESPACE = { NAMESPACE_FUNCTIONS };
#undef NAMESPACE_FUNCTION

#undef NAMESPACE_FUNCTIONS
#undef NAMESPACE

static inline AxisMotor LoadMotorData( const char* );
static inline void UnloadMotorData( CANInterface* );

static inline void ReadRawMeasures( AxisMotor motor );

static int AxisMotor_Init( const char* configFileName )
{
  DEBUG_EVENT( 0, "Initializing Axis Motor %s", configFileName );
  
  if( motorsList == NULL ) motorsList = kh_init( MotorInt );
  
  int configKey = (int) kh_str_hash_func( configFileName );
  
  int insertionStatus;
  khint_t newMotorID = kh_put( MotorInt, motorsList, configKey, &insertionStatus );
  if( insertionStatus > 0 )
  {
    kh_value( motorsList, newMotorID ) = LoadMotorData( configFileName );
    if( kh_value( motorsList, newMotorID ) == NULL )
    {
      AxisMotor_End( (int) newMotorID );
      return -1;
    }
  }
  
  DEBUG_EVENT( 0, "Axis Motor %s initialized (iterator: %d)", configFileName, (int) newMotorID );
  
  return (int) newMotorID;
}

static void AxisMotor_End( int motorID )
{
  if( !kh_exist( motorsList, (khint_t) motorID ) ) return;
  
  Motor motor = kh_value( motorsList, (khint_t) motorID );
  
  UnloadMotorData( motor );
    
  kh_del( MotorInt, motorsList, (khint_t) motorID );
    
  if( kh_size( motorsList ) == 0 )
  {
    kh_destroy( MotorInt, motorsList );
    motorsList = NULL;
  }
}

static void AxisMotor_Enable( int motorID )
{
  if( !kh_exist( motorsList, (khint_t) motorID ) ) return;
  
  Motor motor = kh_value( motorsList, (khint_t) motorID ) );
  
  motor->interface->Enable( motor->axisID );
}

static void AxisMotor_Disable( int motorID )
{
  if( !kh_exist( motorsList, (khint_t) motorID ) ) return;
  
  Motor motor = kh_value( motorsList, (khint_t) motorID ) );
  
  motor->interface->Disable( motor->axisID );
}

static void AxisMotor_Reset( int motorID )
{
  if( !kh_exist( motorsList, (khint_t) motorID ) ) return;
  
  Motor motor = kh_value( motorsList, (khint_t) motorID ) );
  
  motor->interface->Reset( motor->axisID );
}

static void AxisMotor_SetOffset( int motorID )
{
  static double rawMeasuresList[ AXIS_MEASURES_NUMBER ];
  
  if( !kh_exist( motorsList, (khint_t) motorID ) ) return;
  
  Motor motor = kh_value( motorsList, (khint_t) motorID ) );
  
  motor->interface->ReadMeasures( motor->axisID, rawMeasuresList );
  
  motor->measureOffsetsList[ MOTOR_POSITION ] = rawMeasuresList[ AXIS_MEASURE_ENCODER ] / ( motor->encoderResolution * motor->gearReduction );
  motor->measureOffsetsList[ MOTOR_FORCE ] = rawMeasuresList[ AXIS_MEASURE_CURRENT ] * motor->currentToForceRatio * motor->gearReduction;
}

static bool AxisMotor_IsEnabled( int motorID )
{
  if( !kh_exist( motorsList, (khint_t) motorID ) ) return false;
  
  Motor motor = kh_value( motorsList, (khint_t) motorID ) );
  
  return motor->interface->IsEnabled( motor->axisID );
}

static bool AxisMotor_HasError( int motorID )
{
  if( !kh_exist( motorsList, (khint_t) motorID ) ) return false;
  
  Motor motor = kh_value( motorsList, (khint_t) motorID ) );
  
  return motor->interface->HasError( motor->axisID );
}

static double* AxisMotor_ReadMeasures( int motorID )
{
  static double rawMeasuresList[ AXIS_MEASURES_NUMBER ];
  
  if( !kh_exist( motorsList, (khint_t) motorID ) ) return NULL;
  
  Motor motor = kh_value( motorsList, (khint_t) motorID ) );
  
  motor->interface->ReadMeasures( motor->axisID, rawMeasuresList );
  
  motor->measuresList[ MOTOR_POSITION ] = rawMeasuresList[ AXIS_MEASURE_ENCODER ] / ( motor->encoderResolution * motor->gearReduction );
  motor->measuresList[ MOTOR_FORCE ] = rawMeasuresList[ AXIS_MEASURE_CURRENT ] * motor->currentToForceRatio * motor->gearReduction;
  
  for( size_t measureIndex = 0; measureIndex < MOTOR_MEASURES_NUMBER; measureIndex++ )
    motor->measuresList[ measureIndex ] -= motor->measureOffsetsList[ measureIndex ];
}

static void AxisMotor_WriteControl( int motorID )
{
  static double rawSetpointsList[ AXIS_SETPOINTS_NUMBER ];
  
  if( !kh_exist( motorsList, (khint_t) motorID ) ) return;
  
  Motor motor = kh_value( motorsList, (khint_t) motorID ) );
  
  rawSetpointsList[ AXIS_SETPOINT_ENCODER ] = setpoint * motor->encoderResolution * motor->gearReduction + motor->measureOffsetsList[ MOTOR_POSITION ];
  rawSetpointsList[ AXIS_SETPOINT_VELOCITY ] = setpoint * motor->gearReduction;
  rawSetpointsList[ AXIS_SETPOINT_CURRENT ] = setpoint / ( motor->currentToForceRatio * motor->gearReduction ) + motor->measureOffsetsList[ MOTOR_FORCE ];
  
  motor->interface->WriteSetpoints( motor->axisID, rawSetpointsList[ motor->operationMode ] );
}

                                    
                                    
static inline Motor LoadMotorData( const char* configFileName )
{
  Motor newMotor = (Motor) malloc( sizeof(MotorData) );
  
  // File Parsing
  
  
  newMotor->interface = &AxisCANEPOSOperations;
  newMotor->axisID = newMotor->interface->Connect( "CAN Motor Teste" );
  newMotor->operationMode = AXIS_SETPOINT_CURRENT;
  newMotor->encoderResolution = 1;
  newMotor->currentToForceRatio = 151.8; // 0.0302;
  newMotor->gearReduction = 1.0; // 0.0025 / ( 2 * 2 * M_PI );
  
  return newMotor;
}

static inline void UnloadMotorData( Motor motor )
{
  if( motor == NULL ) return;
  
  motor->interface->Disconnect( motor->axisID );
  
  free( motor );
}

#ifdef __cplusplus
    }
#endif

#endif  // AXIS_MOTOR_H
