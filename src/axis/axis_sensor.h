#ifndef AXIS_SENSOR_H
#define AXIS_SENSOR_H

#ifdef __cplusplus
    extern "C" {
#endif

#include "axis/axis_types.h"
#include "spline3_interpolation.h"
      
#include "file_parsing/json_parser.h"

#include "debug/async_debug.h"
      
typedef struct _AxisSensorData
{
  int axisID;
  AxisInterface interface;
  size_t measureIndex;
  Splined3Curve* measureConversionCurve;
  double inputBuffer[ 6 ];
  double inputOffset;
}
AxisSensorData;

typedef AxisSensorData* AxisSensor;

static AxisSensor AxisSensor_Init( const char* );
static inline void AxisSensor_End( AxisSensor );
static inline void AxisSensor_Reset( AxisSensor );
static void AxisSensor_SetOffset( AxisSensor );
static inline bool AxisSensor_IsEnabled( AxisSensor );
static inline bool AxisSensor_HasError( AxisSensor );
static double AxisSensor_Read( AxisSensor );

static inline AxisSensor LoadAxisSensorData( const char* );
static inline void UnloadAxisSensorData( AxisSensor );

static inline void ReadRawMeasures( AxisSensor sensor );

static int AxisSensor_Init( const char* configFileName )
{
  AxisSensor newAxisSensor = (AxisSensor) malloc( sizeof(AxisSensorData) );
  
  // File Parsing
  
  
  newAxisSensor->interface = &AxisCANEPOSOperations;
  newAxisSensor->axisID = newAxisSensor->interface->Connect( "CAN Sensor Teste" );
  
  return newAxisSensor;
}

static inline void AxisSensor_End( AxisSensor sensor )
{
  if( sensor == NULL ) return;
  
  sensor->interface->Disconnect( sensor->axisID );
  
  free( sensor );
}

static inline void AxisSensor_Reset( AxisSensor sensor )
{
  if( sensor == NULL ) return;
  
  sensor->interface->Reset( sensor->axisID );
}

static void AxisSensor_SetOffset( AxisSensor sensor )
{
  static double rawMeasuresList[ AXIS_DIMENSIONS_NUMBER ];
  
  if( sensor == NULL ) return;
  
  sensor->interface->ReadMeasures( sensor->axisID, rawMeasuresList );
  
  sensor->inputOffset = rawMeasuresList[ sensor->measureIndex ];
}

static inline bool AxisSensor_IsEnabled( AxisSensor sensor )
{
  if( sensor == NULL ) return false;
  
  return sensor->interface->IsEnabled( sensor->axisID );
}

static inline bool AxisSensor_HasError( AxisSensor sensor )
{
  if( sensor == NULL ) return false;
  
  return sensor->interface->HasError( sensor->axisID );
}

const double p1 = -5.6853e-024;
const double p2 = 9.5074e-020;
const double p3 = -5.9028e-016;
const double p4 = 1.6529e-012;
const double p5 = -2.0475e-009;
const double p6 = 9.3491e-007;
const double p7 = 0.0021429;
const double p8 = 2.0556;
static double AxisSensor_Read( AxisSensor sensor )
{
  static double rawMeasuresList[ AXIS_MEASURES_NUMBER ];
  
  if( sensor == NULL ) return NULL;
  
  sensor->interface->ReadMeasures( sensor->axisID, rawMeasuresList );
  
  sensor->inputBuffer[5] = sensor->inputBuffer[4];
  sensor->inputBuffer[4] = sensor->inputBuffer[3];
  sensor->inputBuffer[3] = sensor->inputBuffer[2];
  sensor->inputBuffer[2] = sensor->inputBuffer[1];
  sensor->inputBuffer[1] = sensor->inputBuffer[0];
  sensor->inputBuffer[0] = rawMeasuresList[ sensor->measureIndex ] - sensor->inputOffset;
  
  double inputFiltered = (sensor->inputBuffer[0] + sensor->inputBuffer[1] + sensor->inputBuffer[2] + sensor->inputBuffer[3]+ sensor->inputBuffer[4] + sensor->inputBuffer[5])/6;
    
  double measure = p1 * pow(inputFiltered,7) + p2 * pow(inputFiltered,6) + p3 * pow(inputFiltered,5) 
                   + p4 * pow(inputFiltered,4) + p5 * pow(inputFiltered,3) + p6 * pow(inputFiltered,2) + p7 * inputFiltered + p8;   //mm
  
  return measure;
}

//static inline AxisSensor LoadAxisSensorData( const char* );
//static inline void UnloadAxisSensorData( CANInterface* );

#ifdef __cplusplus
    }
#endif

#endif  // AXIS_SENSOR_H
