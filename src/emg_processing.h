#ifndef EMG_PROCESSING_H
#define EMG_PROCESSING_H

#include "signal_processing.h"

#include "config_parser.h"

#include "klib/khash.h"

#include "debug/async_debug.h"
#include "debug/data_logging.h"

#include <math.h>
#include <stdbool.h>

enum EMGMuscleCurves { EMG_MUSCLE_ACTIVE_FORCE, EMG_MUSCLE_PASSIVE_FORCE, EMG_MUSCLE_MOMENT_ARM, EMG_MUSCLE_NORM_LENGTH, EMG_MUSCLE_CURVES_NUMBER };
const size_t EMG_MUSCLE_CURVE_ORDER = 5;
typedef double MuscleCurve[ EMG_MUSCLE_CURVE_ORDER ];

typedef struct _EMGMuscleData
{
  MuscleCurve curvesList[ EMG_MUSCLE_CURVES_NUMBER ];
  double initialPenationAngle;
  double activationFactor;
  double scaleFactor;
}
EMGMuscleData;

typedef EMGMuscleData* EMGMuscle;

//enum EMGMuscleGroups { EMG_MUSCLE_AGONIST, EMG_MUSCLE_ANTAGONIST, EMG_MUSCLE_GROUPS_NUMBER, EMG_MUSCLE_GROUPS_ALL };

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
        function_init( void, namespace, SetJointCalibration, int, bool ) \
        function_init( EMGMuscle, namespace, GetJointMusclesList, int, size_t* ) \
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

void EMGProcessing_SetJointCalibration( int jointID, bool calibrationOn )
{
  khint_t jointIndex = kh_get( JointInt, jointsList, (khint_t) jointID );
  if( jointIndex == kh_end( jointsList ) ) return;
  
  EMGJoint joint = kh_value( jointsList, jointIndex );
  
  enum SignalProcessingPhase processingPhase = ( calibrationOn ) ? SIGNAL_PROCESSING_PHASE_CALIBRATION : SIGNAL_PROCESSING_PHASE_MEASUREMENT;
  
  for( size_t muscleIndex = 0; muscleIndex < joint->musclesListLength; muscleIndex++ )
    SignalProcessing.ChangePhase( joint->sensorIDsList[ muscleIndex ], processingPhase );
}

EMGMuscle EMGProcessing_GetJointMusclesList( int jointID, size_t* ref_musclesCount )
{
  khint_t jointIndex = kh_get( JointInt, jointsList, (khint_t) jointID );
  if( jointIndex == kh_end( jointsList ) ) return NULL;
  
  EMGJoint joint = kh_value( jointsList, jointIndex );
  
  if( ref_musclesCount != NULL ) *ref_musclesCount = joint->musclesListLength;
  
  return joint->musclesList;
}


double EMGProcessing_GetMuscleActivation( EMGMuscle muscleParameters, double normalizedSignal )
{
  if( muscleParameters == NULL ) return 0.0;
  
  if( muscleParameters->activationFactor >= 0.0 ) muscleParameters->activationFactor = -0.1;
  else if( muscleParameters->activationFactor < -3.0 ) muscleParameters->activationFactor = -3.0;
  
  return ( exp( muscleParameters->activationFactor * normalizedSignal ) - 1 ) / ( exp( muscleParameters->activationFactor ) - 1 );
}

double EMGProcessing_GetMuscleTorque( EMGMuscle muscleParameters, double normalizedSignal, double jointAngle )
{
  static double measuresList[ EMG_MUSCLE_CURVES_NUMBER ];
  
  if( muscleParameters == NULL ) return 0.0;
  
  double activation = EMGProcessing_GetMuscleActivation( muscleParameters, normalizedSignal );
  
  /*if( activation != 0.0 )
  {
    size_t newSamplesBufferStart = sensor->emgData.samplesBufferStartIndex % sensor->emgData.samplesBufferLength;
    size_t newSamplesBufferMaxLength = sensor->emgData.filteredSamplesBufferLength - FILTER_EXTRA_SAMPLES_NUMBER;
    DataLogging_RegisterList( sensor->logID, newSamplesBufferMaxLength, sensor->emgData.samplesBuffer + newSamplesBufferStart );
    DataLogging_RegisterList( sensor->logID, newSamplesBufferMaxLength, sensor->emgData.filteredSamplesBuffer + FILTER_EXTRA_SAMPLES_NUMBER );
    DataLogging_RegisterValues( sensor->logID, 3, sensor->emgData.processingResultsList[ EMG_ACTIVATION_PHASE ], jointAngle, Timing_GetExecTimeMilliseconds() );
  }*/
  
  if( muscleParameters->curvesList[ EMG_MUSCLE_ACTIVE_FORCE ][ 0 ] < 0.0 ) muscleParameters->curvesList[ EMG_MUSCLE_ACTIVE_FORCE ][ 0 ] = 0.0;
  if( muscleParameters->curvesList[ EMG_MUSCLE_PASSIVE_FORCE ][ 0 ] < 0.0 ) muscleParameters->curvesList[ EMG_MUSCLE_PASSIVE_FORCE ][ 0 ] = 0.0;
  if( muscleParameters->scaleFactor < 0.0 ) muscleParameters->scaleFactor = 0.0;
  
  for( size_t curveIndex = 0; curveIndex < EMG_MUSCLE_CURVES_NUMBER; curveIndex++ )
  {
    measuresList[ curveIndex ] = 0.0;
    for( size_t factorIndex = 0; factorIndex < EMG_MUSCLE_CURVE_ORDER; factorIndex++ )
      measuresList[ curveIndex ] += muscleParameters->curvesList[ curveIndex ][ factorIndex ] * pow( jointAngle, factorIndex );
  }
  
  /*double normalizedSin = sin( muscleParameters->initialPenationAngle ) / measuresList[ EMG_MUSCLE_NORM_LENGTH ];
  if( normalizedSin > 1.0 ) normalizedSin == 1.0;
  else if( normalizedSin < -1.0 ) normalizedSin == -1.0;
  
  double penationAngle = asin( normalizedSin );*/
  double penationAngle = 0.0;
  
  double activationForce = measuresList[ EMG_MUSCLE_ACTIVE_FORCE ] * activation + measuresList[ EMG_MUSCLE_PASSIVE_FORCE ];
  double resultingForce = muscleParameters->scaleFactor * cos( penationAngle ) * activationForce;
  
  return resultingForce * measuresList[ EMG_MUSCLE_MOMENT_ARM ];
}



const char* MUSCLE_LIST_NAME = "muscles";
static EMGJoint LoadEMGJointData( const char* configFileName )
{
  DEBUG_PRINT( "Trying to load joint %s EMG data", configFileName );
  
  static char searchPath[ PARSER_MAX_FILE_PATH_LENGTH ];
  
  bool loadError = false;
  
  EMGJoint newJoint = (EMGJoint) malloc( sizeof(EMGJointData) );
  memset( newJoint, 0, sizeof(EMGJointData) );
  
  int configFileID = ConfigParser.LoadFileData( configFileName );
  if( configFileID != -1 )
  {
    if( (newJoint->musclesListLength = (size_t) ConfigParser.GetListSize( configFileID, MUSCLE_LIST_NAME )) > 0 )
    {
      DEBUG_PRINT( "%u muscles found for joint %s", newJoint->musclesListLength, configFileName );
      
      newJoint->musclesList = (EMGMuscle) calloc( newJoint->musclesListLength , sizeof(EMGMuscleData) );
      newJoint->sensorIDsList = (int*) calloc( newJoint->musclesListLength, sizeof(int) );
      
      for( size_t muscleIndex = 0; muscleIndex < newJoint->musclesListLength; muscleIndex++ )
      {
        sprintf( searchPath, "%s.%u", MUSCLE_LIST_NAME, muscleIndex );
        newJoint->sensorIDsList[ muscleIndex ] = SignalProcessing.InitSensor( ConfigParser.GetStringValue( configFileID, searchPath, "" ) );
        if( newJoint->sensorIDsList[ muscleIndex ] == SENSOR_INVALID_ID ) loadError = true;
      }
    }
    else loadError = true;

    ConfigParser.UnloadData( configFileID );
  }
  else
  {
    DEBUG_PRINT( "configuration for joint %s not found", configFileName );
    loadError = true;
  }
    
  if( loadError )
  {
    UnloadEMGJointData( newJoint );
    return NULL;
  }
  
  return newJoint;
}

static void UnloadEMGJointData( EMGJoint joint )
{
  if( joint == NULL ) return;
  
  for( size_t muscleIndex = 0; muscleIndex < joint->musclesListLength; muscleIndex++ )
    SignalProcessing.EndSensor( joint->sensorIDsList[ muscleIndex ] );
  free( joint->sensorIDsList );
  
  free( joint->musclesList );
  
  free( joint );
}

#endif /* EMG_PROCESSING_H */
