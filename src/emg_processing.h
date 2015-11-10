#ifndef EMG_PROCESSING_H
#define EMG_PROCESSING_H

#include "signal_processing.h"
#include "curve_interpolation.h"

#include "config_parser.h"

#include "klib/khash.h"

#include "debug/async_debug.h"
#include "debug/data_logging.h"

#include <math.h>
#include <stdbool.h>

enum { MUSCLE_ACTIVE_FORCE, MUSCLE_PASSIVE_FORCE, MUSCLE_MOMENT_ARM, MUSCLE_NORM_LENGTH, MUSCLE_PENATION_ANGLE, MUSCLE_CURVES_NUMBER };
enum EMGMuscleParameters{ MUSCLE_ACTIVATION, MUSCLE_OPTIMAL_LENGTH, MUSCLE_OPTIMAL_ARM, MUSCLE_PENATION, MUSCLE_MAX_FORCE, MUSCLE_PARAMETERS_NUMBER };

typedef struct _EMGMuscleData
{
  Curve curvesList[ MUSCLE_CURVES_NUMBER ];
  double parametersList[ MUSCLE_PARAMETERS_NUMBER ];
}
EMGMuscleData;

typedef EMGMuscleData* EMGMuscle;

typedef struct _EMGJointData
{
  EMGMuscle musclesList;
  int* sensorIDsList;
  size_t musclesListLength;
}
EMGJointData;

typedef EMGJointData* EMGJoint;

KHASH_MAP_INIT_INT( JointInt, EMGJoint )
static khash_t( JointInt )* jointsList = NULL;

#define EMG_PROCESSING_FUNCTIONS( namespace, function_init ) \
        function_init( int, namespace, InitJoint, const char* ) \
        function_init( void, namespace, EndJoint, int ) \
        function_init( double, namespace, GetJointMuscleSignal, int, size_t ) \
        function_init( double, namespace, GetJointTorque, int, double ) \
        function_init( double, namespace, GetJointStiffness, int, double ) \
        function_init( void, namespace, SetProcessingPhase, int, enum SignalProcessingPhase ) \
        function_init( size_t, namespace, GetJointMusclesCount, int ) \
        function_init( double, namespace, GetJointMuscleParameter, int, size_t, enum EMGMuscleParameters ) \
        function_init( double, namespace, SetJointMuscleParameter, int, size_t, enum EMGMuscleParameters, double ) \
        function_init( double, namespace, GetMuscleActivation, EMGMuscle, double ) \
        function_init( double, namespace, GetMuscleTorque, EMGMuscle, double, double )

INIT_NAMESPACE_INTERFACE( EMGProcessing, EMG_PROCESSING_FUNCTIONS )

static EMGJoint LoadEMGJointData( const char* );
static void UnloadEMGJointData( EMGJoint );

const int EMG_JOINT_INVALID_ID = 0;

static int EMGProcessing_InitJoint( const char* configFileName )
{
  if( jointsList == NULL ) jointsList = kh_init( JointInt );
  
  int configKey = (int) kh_str_hash_func( configFileName );
  
  int insertionStatus;
  khint_t newJointIndex = kh_put( JointInt, jointsList, configKey, &insertionStatus );
  if( insertionStatus > 0 )
  {
    kh_value( jointsList, newJointIndex ) = LoadEMGJointData( configFileName );
    if( kh_value( jointsList, newJointIndex ) == NULL )
    {
      DEBUG_PRINT( "EMG joint controller %s configuration failed", configFileName );
      EMGProcessing_EndJoint( (int) newJointIndex );
      return EMG_JOINT_INVALID_ID;
    }
    
    DEBUG_PRINT( "new key %d inserted (iterator: %u - total: %u)", kh_key( jointsList, newJointIndex ), newJointIndex, kh_size( jointsList ) );
  }
  else if( insertionStatus == 0 ) DEBUG_PRINT( "joint key %d already exists", configKey );
  
  return (int) kh_key( jointsList, newJointIndex );
}

void EMGProcessing_EndJoint( int jointID )
{
  khint_t jointIndex = kh_get( JointInt, jointsList, (khint_t) jointID );
  if( jointIndex == kh_end( jointsList ) ) return;
  
  UnloadEMGJointData( kh_value( jointsList, jointIndex ) );
  
  kh_del( JointInt, jointsList, jointIndex );
  
  if( kh_size( jointsList ) == 0 )
  {
    kh_destroy( JointInt, jointsList );
    jointsList = NULL;
  }
}

double EMGProcessing_GetJointMuscleSignal( int jointID, size_t muscleIndex )
{
  khint_t jointIndex = kh_get( JointInt, jointsList, (khint_t) jointID );
  if( jointIndex == kh_end( jointsList ) ) return 0.0;
  
  EMGJoint joint = kh_value( jointsList, jointIndex );
  
  if( muscleIndex >= joint->musclesListLength ) return 0.0;
  
  return SignalProcessing.GetProcessedSignal( joint->sensorIDsList[ muscleIndex ] );
}

double EMGProcessing_GetJointTorque( int jointID, double jointAngle )
{
  khint_t jointIndex = kh_get( JointInt, jointsList, (khint_t) jointID );
  if( jointIndex == kh_end( jointsList ) ) return 0.0;
  
  EMGJoint joint = kh_value( jointsList, jointIndex );
  
  double jointTorque = 0.0;
  
  for( size_t muscleIndex = 0; muscleIndex < joint->musclesListLength; muscleIndex++ )
  {
    double normalizedSignal = SignalProcessing.GetProcessedSignal( joint->sensorIDsList[ muscleIndex ] );
    jointTorque += EMGProcessing_GetMuscleTorque( joint->musclesList + muscleIndex, normalizedSignal, jointAngle );
  }
  
  return jointTorque;
}

double EMGProcessing_GetJointStiffness( int jointID, double jointAngle )
{
  khint_t jointIndex = kh_get( JointInt, jointsList, (khint_t) jointID );
  if( jointIndex == kh_end( jointsList ) ) return 0.0;
  
  EMGJoint joint = kh_value( jointsList, jointIndex );
  
  double jointStiffness = 0.0;
  
  for( size_t muscleIndex = 0; muscleIndex < joint->musclesListLength; muscleIndex++ )
  {
    double normalizedSignal = SignalProcessing.GetProcessedSignal( joint->sensorIDsList[ muscleIndex ] );
    jointStiffness += fabs( EMGProcessing_GetMuscleTorque( joint->musclesList + muscleIndex, normalizedSignal, jointAngle ) );
  }
  
  //DEBUG_PRINT( "joint stiffness: %.3f + %.3f = %.3f", EMGProcessing_GetMuscleTorque( 0, jointAngle ), EMGProcessing_GetMuscleTorque( 1, jointAngle ), jointStiffness );
  
  return jointStiffness;
}

void EMGProcessing_SetProcessingPhase( int jointID, enum SignalProcessingPhase processingPhase )
{
  khint_t jointIndex = kh_get( JointInt, jointsList, (khint_t) jointID );
  if( jointIndex == kh_end( jointsList ) ) return;
  
  EMGJoint joint = kh_value( jointsList, jointIndex );
  
  for( size_t muscleIndex = 0; muscleIndex < joint->musclesListLength; muscleIndex++ )
    SignalProcessing.ChangePhase( joint->sensorIDsList[ muscleIndex ], processingPhase );
}

size_t EMGProcessing_GetJointMusclesCount( int jointID )
{
  khint_t jointIndex = kh_get( JointInt, jointsList, (khint_t) jointID );
  if( jointIndex == kh_end( jointsList ) ) return 0;
  
  EMGJoint joint = kh_value( jointsList, jointIndex );
  
  return joint->musclesListLength;
}

double EMGProcessing_GetJointMuscleParameter( int jointID, size_t muscleIndex, enum EMGMuscleParameters parameterIndex )
{
  khint_t jointIndex = kh_get( JointInt, jointsList, (khint_t) jointID );
  if( jointIndex == kh_end( jointsList ) ) return 0.0;
  
  EMGMuscle joint = kh_value( jointsList, jointIndex );
  
  if( muscleIndex >= joint->musclesListLength ) return 0.0;
  
  if( parameterIndex < 0 && parameterIndex >= MUSCLE_PARAMETERS_NUMBER ) return 0.0;
  
  return joint->musclesList[ muscleIndex ].parametersList[ parameterIndex ];
}

const double PARAMETER_LIMITS[ MUSCLE_PARAMETERS_NUMBER ][ 2 ] = { { -3.0, -0.1 }, { 0.8, 1.2 }, { 0.8, 1.2 }, { 0.8, 1.2 }, { 0.5, 1.5 } };
double EMGProcessing_SetJointMuscleParameter( int jointID, size_t muscleIndex, enum EMGMuscleParameters parameterIndex, double value )
{
  khint_t jointIndex = kh_get( JointInt, jointsList, (khint_t) jointID );
  if( jointIndex == kh_end( jointsList ) ) return 0.0;
  
  EMGMuscle joint = kh_value( jointsList, jointIndex );
  
  if( muscleIndex >= joint->musclesListLength ) return 0.0;
  
  if( parameterIndex < 0 && parameterIndex >= MUSCLE_PARAMETERS_NUMBER ) return 0.0;
  
  EMGMuscle muscle = joint->musclesList + muscleIndex;
  if( value < PARAMETER_LIMITS[ parameterIndex ][ 0 ] ) value = PARAMETER_LIMITS[ parameterIndex ][ 0 ];
  else if( value > PARAMETER_LIMITS[ parameterIndex ][ 1 ] ) value = PARAMETER_LIMITS[ parameterIndex ][ 1 ];
  
  muscle->parametersList[ parameterIndex ] = value;
  
  return muscle->parametersList[ parameterIndex ];
}

//const double ACTIVE_FORCE_COEFFS[] = { 0.581359097121952, -1.023853722108468, -1.988129364006696, 4.208590538437200, -1.222906314763417, 0.443815500946629 };
//const double PASSIVE_FORCE_COEFFS[] = { -1.746618652817642,  8.266939015253374, -14.364808017099560, 11.877489584777614, -4.933453643876041, 0.973170918828070, -0.071230938136188 };
double EMGProcessing_GetMuscleActivation( EMGMuscle muscle, double normalizedSignal )
{
  if( muscle == NULL ) return 0.0;
  
  double activationFactor = muscle->parametersList[ MUSCLE_ACTIVATION ];
  
  return ( exp( activationFactor * normalizedSignal ) - 1 ) / ( exp( activationFactor ) - 1 );
}

double EMGProcessing_GetMuscleTorque( EMGMuscle muscle, double normalizedSignal, double jointAngle )
{
  if( muscle == NULL ) return 0.0;
  
  double activation = EMGProcessing_GetMuscleActivation( muscle, normalizedSignal );
  
  /*if( activation != 0.0 )
  {
    size_t newSamplesBufferStart = sensor->emgData.samplesBufferStartIndex % sensor->emgData.samplesBufferLength;
    size_t newSamplesBufferMaxLength = sensor->emgData.filteredSamplesBufferLength - FILTER_EXTRA_SAMPLES_NUMBER;
    DataLogging_RegisterList( sensor->logID, newSamplesBufferMaxLength, sensor->emgData.samplesBuffer + newSamplesBufferStart );
    DataLogging_RegisterList( sensor->logID, newSamplesBufferMaxLength, sensor->emgData.filteredSamplesBuffer + FILTER_EXTRA_SAMPLES_NUMBER );
    DataLogging_RegisterValues( sensor->logID, 3, sensor->emgData.processingResultsList[ EMG_ACTIVATION_PHASE ], jointAngle, Timing_GetExecTimeMilliseconds() );
  }*/
  
  double activeForce = CurveInterpolation.GetValue( muscle->curvesList[ MUSCLE_ACTIVE_FORCE ], jointAngle, 0.0 );
  double passiveForce = CurveInterpolation.GetValue( muscle->curvesList[ MUSCLE_PASSIVE_FORCE ], jointAngle, 0.0 );
  
  double normalizedLength = muscle->parametersList[ MUSCLE_OPTIMAL_LENGTH ] * CurveInterpolation.GetValue( muscle->curvesList[ MUSCLE_NORM_LENGTH ], jointAngle, 0.0 );
  double momentArm = muscle->parametersList[ MUSCLE_OPTIMAL_ARM ] * CurveInterpolation.GetValue( muscle->curvesList[ MUSCLE_MOMENT_ARM ], jointAngle, 0.0 );
  
  double initialPenationAngle = CurveInterpolation.GetValue( muscle->curvesList[ MUSCLE_PENATION_ANGLE ], jointAngle, 0.0 );
  double penationAngle = muscle->parametersList[ MUSCLE_PENATION ] * asin( sin( initialPenationAngle ) / normalizedLength );
  
  double normalizedForce = activeForce * activation + passiveForce;
  double resultingForce = muscle->parametersList[ MUSCLE_MAX_FORCE ] * cos( penationAngle ) * normalizedForce;
  
  return resultingForce * momentArm;
}


const char* MUSCLE_CURVE_NAMES[ MUSCLE_CURVES_NUMBER ] = { "active_force", "passive_force", "moment_arm", "normalized_length" };
static bool LoadEMGMuscleData( const char* configFileName, EMGMuscle newMuscle )
{
  DEBUG_PRINT( "Trying to load muscle %s EMG data", configFileName );
  
  int configFileID = ConfigParsing.LoadConfigFile( configFileName );
  if( configFileID != PARSED_DATA_INVALID_ID )
  {
    ParserInterface parser = ConfigParsing.GetParser();
    
    for( size_t curveIndex = 0; curveIndex < MUSCLE_CURVES_NUMBER; curveIndex++ )
    {
      char* curveString = parser.GetStringValue( configFileID, NULL, "muscles.%u.%s", muscleIndex, MUSCLE_CURVE_NAMES[ curveIndex ] );
      newMuscle->curvesList[ curveIndex ] = CurveInterpolation.LoadCurveString( curveString );
    }

    newMuscle->parametersList[ MUSCLE_ACTIVATION ] = -2.0;
    newMuscle->parametersList[ MUSCLE_OPTIMAL_LENGTH ] = 1.0;
    newMuscle->parametersList[ MUSCLE_OPTIMAL_ARM ] = 1.0;
    newMuscle->parametersList[ MUSCLE_PENATION_ANGLE ] = 1.0;
    newMuscle->parametersList[ MUSCLE_MAX_FORCE ] = 1.0;

    parser.UnloadData( configFileID );
  }
  else
  {
    DEBUG_PRINT( "configuration for muscle %s not found", configFileName );
    return false;
  }
  
  return true;
}

static EMGJoint LoadEMGJointData( const char* configFileName )
{
  DEBUG_PRINT( "Trying to load joint %s EMG data", configFileName );
  
  EMGJoint newJoint = NULL;
  
  int configFileID = ConfigParsing.LoadConfigFile( configFileName );
  if( configFileID != PARSED_DATA_INVALID_ID )
  {
    newJoint = (EMGJoint) malloc( sizeof(EMGJointData) );
    memset( newJoint, 0, sizeof(EMGJointData) );
    
    ParserInterface parser = ConfigParsing.GetParser();
    
    bool loadError = false;
    
    if( (newJoint->musclesListLength = (size_t) parser.GetListSize( configFileID, "muscles" )) > 0 )
    {
      DEBUG_PRINT( "%u muscles found for joint %s", newJoint->musclesListLength, configFileName );
      
      newJoint->musclesList = (EMGMuscle) calloc( newJoint->musclesListLength , sizeof(EMGMuscleData) );
      newJoint->sensorIDsList = (int*) calloc( newJoint->musclesListLength, sizeof(int) );
      for( size_t muscleIndex = 0; muscleIndex < newJoint->musclesListLength; muscleIndex++ )
      {
        newJoint->sensorIDsList[ muscleIndex ] = SignalProcessing.InitSensor( parser.GetStringValue( configFileID, "", "muscles.%u.sensor", muscleIndex ) );
        if( newJoint->sensorIDsList[ muscleIndex ] == SENSOR_INVALID_ID ) loadError = true;
        
        if( !LoadEMGMuscleData( parser.GetStringValue( configFileID, "", "muscles.%u.properties", muscleIndex ), newJoint->musclesList + muscleIndex ) ) loadError = true;
      }
    }
    else loadError = true;

    parser.UnloadData( configFileID );
    
    if( loadError )
    {
      UnloadEMGJointData( newJoint );
      return NULL;
    }
  }
  else
    DEBUG_PRINT( "configuration for joint %s not found", configFileName );
  
  return newJoint;
}

static void UnloadEMGJointData( EMGJoint joint )
{
  if( joint == NULL ) return;
  
  for( size_t muscleIndex = 0; muscleIndex < joint->musclesListLength; muscleIndex++ )
  {
    SignalProcessing.EndSensor( joint->sensorIDsList[ muscleIndex ] );
    for( size_t curveIndex = 0; curveIndex < MUSCLE_CURVES_NUMBER; curveIndex++ )
      CurveInterpolation.UnloadCurve( joint->musclesList[ muscleIndex ].curvesList[ curveIndex ] );
  }
  
  free( joint->sensorIDsList );
  free( joint->musclesList );
  
  free( joint );
}

#endif /* EMG_PROCESSING_H */
