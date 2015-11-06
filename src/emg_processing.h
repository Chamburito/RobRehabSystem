#ifndef EMG_PROCESSING_H
#define EMG_PROCESSING_H

#include "signal_processing.h"

#include "config_parser.h"

#include "klib/khash.h"

#include "debug/async_debug.h"
#include "debug/data_logging.h"

#include <math.h>
#include <stdbool.h>

typedef struct _EMGMuscleData
{
  double normalizedLengthFactor[ 2 ];
  double momentArmFactor[ 2 ];
  double penationAngleFactor;
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
        function_init( void, namespace, SetProcessingPhase, int, enum SignalProcessingPhase ) \
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

void EMGProcessing_SetProcessingPhase( int jointID, enum SignalProcessingPhase processingPhase )
{
  khint_t jointIndex = kh_get( JointInt, jointsList, (khint_t) jointID );
  if( jointIndex == kh_end( jointsList ) ) return;
  
  EMGJoint joint = kh_value( jointsList, jointIndex );
  
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


const double ACTIVE_FORCE_COEFFS[] = { 0.581359097121952, -1.023853722108468, -1.988129364006696, 4.208590538437200, -1.222906314763417, 0.443815500946629 };
const double PASSIVE_FORCE_COEFFS[] = { -1.746618652817642,  8.266939015253374, -14.364808017099560, 11.877489584777614, -4.933453643876041, 0.973170918828070, -0.071230938136188 };
double EMGProcessing_GetMuscleActivation( EMGMuscle muscleParameters, double normalizedSignal )
{
  if( muscleParameters == NULL ) return 0.0;
  
  if( muscleParameters->activationFactor >= 0.0 ) muscleParameters->activationFactor = -0.1;
  else if( muscleParameters->activationFactor < -3.0 ) muscleParameters->activationFactor = -3.0;
  
  return ( exp( muscleParameters->activationFactor * normalizedSignal ) - 1 ) / ( exp( muscleParameters->activationFactor ) - 1 );
}

double EMGProcessing_GetMuscleTorque( EMGMuscle muscleParameters, double normalizedSignal, double jointAngle )
{
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
  
  double normalizedLength = muscleParameters->normalizedLengthFactor[ 1 ] * jointAngle + muscleParameters->normalizedLengthFactor[ 0 ];
  double momentArm = muscleParameters->momentArmFactor[ 1 ] * jointAngle + muscleParameters->momentArmFactor[ 0 ];
  
  double activeForce = ACTIVE_FORCE_COEFFS[ 5 ];
  for( int coeffIndex = 4; coeffIndex >= 0; coeffIndex-- )
    activeForce += ACTIVE_FORCE_COEFFS[ coeffIndex ] * pow( normalizedLength, 5 - coeffIndex );
  double passiveForce = PASSIVE_FORCE_COEFFS[ 6 ];
  for( int coeffIndex = 5; coeffIndex >= 0; coeffIndex-- )
    passiveForce += PASSIVE_FORCE_COEFFS[ coeffIndex ] * pow( momentArm, 5 - coeffIndex );
  
  double phi = asin( sin( muscleParameters->penationAngleFactor ) / normalizedLength );
  
  double normalizedForce = activeForce * activation + passiveForce;
  double resultingForce = muscleParameters->scaleFactor * cos( phi ) * normalizedForce;
  
  return resultingForce * momentArm;
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
        newJoint->sensorIDsList[ muscleIndex ] = SignalProcessing.InitSensor( parser.GetStringValue( configFileID, "", "muscles.%u", muscleIndex ) );
        if( newJoint->sensorIDsList[ muscleIndex ] == SENSOR_INVALID_ID ) loadError = true;
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
    SignalProcessing.EndSensor( joint->sensorIDsList[ muscleIndex ] );
  free( joint->sensorIDsList );
  
  free( joint->musclesList );
  
  free( joint );
}

#endif /* EMG_PROCESSING_H */
