#ifndef AES_ACTUATOR_INTERFACE_H
#define AES_ACTUATOR_INTERFACE_H 

#include "../actuator_interface.h"
#include "../spline3_interpolation.h"
#include "../filters.h"

#include "../klib/khash.h"
#include "../klib/kson.h"

#include "../async_debug.h"

typedef struct _Actuator
{
  Axis motor, sensor;
  double measuresList[ AXIS_DIMENSIONS_NUMBER ];
  Splined3Curve* interactionForceCurve;
  SimpleKalmanFilter* positionFilter;
}
Actuator;

KHASH_MAP_INIT_INT( AES, Actuator );
static khash_t( AES )* actuatorsList = NULL;

static int Init( const char* );
static void End( int );
static void Enable( int );
static void Disable( int );
static void Reset( int );
static bool IsEnabled( int );
static bool HasError( int );
static bool SetOption( int, const char*, void* );
static bool GetMeasure( int, enum AxisDimensions );
static bool SetSetpoint( int, double );
static void ReadAxes( int );
static void WriteControl( int );
static void SetOperationMode( int, enum AxisDimensions );

const ActuatorInterface AESInterface = { .Init = Init,
                                         .End = End,
                                         .Enable = Enable,
                                         .Disable = Disable,
                                         .Reset = Reset,
                                         .IsEnabled = IsEnabled,
                                         .HasError = HasError,
                                         .SetOption = SetOption,
                                         .GetMeasure = GetMeasure,
                                         .SetSetpoint = SetSetpoint,
                                         .ReadAxes = ReadAxes,
                                         .WriteControl = WriteControl,
                                         .SetOperationMode = SetOperationMode };
                                         
int Init( const char* options )
{
  kson_node_t* optionNode;
  kson_node_t* subOptionNode;
  
  if( actuatorsList == NULL ) actuatorsList = kh_init( AES );
  
  int insertionStatus;
  int actuatorID = kh_put( AES, actuatorsList, kh_size( actuatorsList ), &insertionStatus );
  
  if( insertionStatus == -1 ) return -1;
  //else if( insertionStatus == 0 ) return -1;
  
  Actuator* newActuator = &(kh_value( actuatorsList, actuatorID ));
  
  kson_t* optionsBlock = kson_parse( options );
  
  if( (optionNode = kson_by_key( optionsBlock->root, "motor" )) )
  {   
    if( (subOptionNode = kson_by_key( optionNode, "interface" )) )
    {
      newActuator->motor.interface = &AxisCANEPOSInterface;
      newActuator->motor.interfaceID = newActuator->motor.interface->Connect( kson_by_index( subOptionNode, 1 ) );
    }        
  }
}


#endif // AES_ACTUATOR_INTERFACE_H
