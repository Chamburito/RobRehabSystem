#ifndef EMG_PROCESSING_H
#define EMG_PROCESSING_H

#include "sensors.h"
#include "curve_interpolation.h"

#include "config_parser.h"

#include "klib/khash.h"

#include "debug/async_debug.h"
#include "debug/data_logging.h"

#include <math.h>
#include <stdbool.h>

enum { MUSCLE_ACTIVE_FORCE, MUSCLE_PASSIVE_FORCE, MUSCLE_MOMENT_ARM, MUSCLE_NORM_LENGTH, MUSCLE_PENATION_ANGLE, MUSCLE_CURVES_NUMBER };
enum EMGMuscleGains { MUSCLE_GAIN_ACTIVATION, MUSCLE_GAIN_LENGTH, MUSCLE_GAIN_ARM, MUSCLE_GAIN_PENATION, MUSCLE_GAIN_FORCE, MUSCLE_GAINS_NUMBER };

const double JOINT_MIN_GAIN = 0.1;
const double JOINT_MAX_GAIN = 2.0;

const double MUSCLE_MAX_GAINS[ MUSCLE_GAINS_NUMBER ] = { -0.1, 1.2, 1.2, 1.2, 1.5 };
const double MUSCLE_MIN_GAINS[ MUSCLE_GAINS_NUMBER ] = { -3.0, 0.8, 0.8, 0.8, 0.5 };

typedef struct _EMGMuscleData
{
  Sensor emgSensor;
  Curve curvesList[ MUSCLE_CURVES_NUMBER ];
  double gainsList[ MUSCLE_GAINS_NUMBER ];
}
EMGMuscleData;

typedef EMGMuscleData* EMGMuscle;

typedef struct _EMGJointData
{
  EMGMuscle* musclesList;
  size_t musclesListLength;
  double scaleFactor;
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
        function_init( double, namespace, SetJointGain, int, double ) \
        function_init( void, namespace, SetProcessingPhase, int, enum SignalProcessingPhase ) \
        function_init( size_t, namespace, GetJointMusclesCount, int ) \
        function_init( double, namespace, SetJointMuscleGain, int, size_t, enum EMGMuscleGains, double ) \
        function_init( double, namespace, GetJointMuscleTorque, int, size_t, double, double )

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
  
  //DEBUG_PRINT( "updating sensor %d-%lu (%p)", jointID, muscleIndex, joint->musclesList[ muscleIndex ]->emgSensor );
  
  double normalizedSignal = Sensors.Update( joint->musclesList[ muscleIndex ]->emgSensor );
  
  return normalizedSignal;
}

double GetMuscleTorque( EMGMuscle muscle, double normalizedSignal, double jointAngle )
{
  if( muscle == NULL ) return 0.0;
  
  double activationFactor = muscle->gainsList[ MUSCLE_GAIN_ACTIVATION ];
  double activation = ( exp( activationFactor * normalizedSignal ) - 1 ) / ( exp( activationFactor ) - 1 );
  
  if( activation != 0.0 )
  {
    Sensor emgSensor = muscle->emgSensor;
    size_t newSamplesBufferStart = emgSensor->emgData.samplesBufferStartIndex % emgSensor->emgData.samplesBufferLength;
    size_t newSamplesBufferMaxLength = emgSensor->emgData.filteredSamplesBufferLength - FILTER_EXTRA_SAMPLES_NUMBER;
    DataLogging_RegisterValues( emgSensor->logID, 3, emgSensor->emgData.processingResultsList[ EMG_ACTIVATION_PHASE ], jointAngle, Timing_GetExecTimeMilliseconds() );
    DataLogging_RegisterList( emgSensor->logID, newSamplesBufferMaxLength, emgSensor->emgData.samplesBuffer + newSamplesBufferStart );
    DataLogging_RegisterList( emgSensor->logID, newSamplesBufferMaxLength, emgSensor->emgData.filteredSamplesBuffer + FILTER_EXTRA_SAMPLES_NUMBER );
  }
  
  double activeForce = CurveInterpolation.GetValue( muscle->curvesList[ MUSCLE_ACTIVE_FORCE ], jointAngle, 0.0 );
  double passiveForce = CurveInterpolation.GetValue( muscle->curvesList[ MUSCLE_PASSIVE_FORCE ], jointAngle, 0.0 );
  
  double normalizedLength = muscle->gainsList[ MUSCLE_GAIN_LENGTH ] * CurveInterpolation.GetValue( muscle->curvesList[ MUSCLE_NORM_LENGTH ], jointAngle, 0.0 );
  double momentArm = muscle->gainsList[ MUSCLE_GAIN_ARM ] * CurveInterpolation.GetValue( muscle->curvesList[ MUSCLE_MOMENT_ARM ], jointAngle, 0.0 );
  
  double initialPenationAngle = CurveInterpolation.GetValue( muscle->curvesList[ MUSCLE_PENATION_ANGLE ], jointAngle, 0.0 );
  double penationAngle = muscle->gainsList[ MUSCLE_GAIN_PENATION ] * asin( sin( initialPenationAngle ) / normalizedLength );
  
  double normalizedForce = activeForce * activation + passiveForce;
  double resultingForce = muscle->gainsList[ MUSCLE_GAIN_FORCE ] * cos( penationAngle ) * normalizedForce;
  
  return resultingForce * momentArm;
}

double EMGProcessing_GetJointTorque( int jointID, double jointAngle )
{
  khint_t jointIndex = kh_get( JointInt, jointsList, (khint_t) jointID );
  if( jointIndex == kh_end( jointsList ) ) return 0.0;
  
  EMGJoint joint = kh_value( jointsList, jointIndex );
  
  double jointTorque = 0.0;
  
  for( size_t muscleIndex = 0; muscleIndex < joint->musclesListLength; muscleIndex++ )
  {
    double normalizedSignal = Sensors.Update( joint->musclesList[ muscleIndex ]->emgSensor );
    jointTorque += GetMuscleTorque( joint->musclesList[ muscleIndex ], normalizedSignal, jointAngle );
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
    double normalizedSignal = Sensors.Update( joint->musclesList[ muscleIndex ]->emgSensor );
    jointStiffness += fabs( GetMuscleTorque( joint->musclesList[ muscleIndex ], normalizedSignal, jointAngle ) );
  }
  
  //DEBUG_PRINT( "joint stiffness: %.3f + %.3f = %.3f", EMGProcessing_GetMuscleTorque( 0, jointAngle ), EMGProcessing_GetMuscleTorque( 1, jointAngle ), jointStiffness );
  
  return jointStiffness;
}

double EMGProcessing_SetJointGain( int jointID, double value )
{
  khint_t jointIndex = kh_get( JointInt, jointsList, (khint_t) jointID );
  if( jointIndex == kh_end( jointsList ) ) return 0.0;
  
  EMGJoint joint = kh_value( jointsList, jointIndex );
  
  if( value > JOINT_MAX_GAIN ) value = JOINT_MAX_GAIN;
  else if( value < JOINT_MIN_GAIN ) value = JOINT_MIN_GAIN;
  
  joint->scaleFactor = value;
  
  return joint->scaleFactor;
}

void EMGProcessing_SetProcessingPhase( int jointID, enum SignalProcessingPhase processingPhase )
{
  khint_t jointIndex = kh_get( JointInt, jointsList, (khint_t) jointID );
  if( jointIndex == kh_end( jointsList ) ) return;
  
  EMGJoint joint = kh_value( jointsList, jointIndex );
  
  for( size_t muscleIndex = 0; muscleIndex < joint->musclesListLength; muscleIndex++ )
    Sensors.SetState( joint->musclesList[ muscleIndex ]->emgSensor, processingPhase );
}

size_t EMGProcessing_GetJointMusclesCount( int jointID )
{
  khint_t jointIndex = kh_get( JointInt, jointsList, (khint_t) jointID );
  if( jointIndex == kh_end( jointsList ) ) return 0;
  
  EMGJoint joint = kh_value( jointsList, jointIndex );
  
  return joint->musclesListLength;
}

double EMGProcessing_SetJointMuscleGain( int jointID, size_t muscleIndex, enum EMGMuscleGains gainIndex, double value )
{
  khint_t jointIndex = kh_get( JointInt, jointsList, (khint_t) jointID );
  if( jointIndex == kh_end( jointsList ) ) return 0.0;
  
  EMGJoint joint = kh_value( jointsList, jointIndex );
  
  if( muscleIndex >= joint->musclesListLength ) return 0.0;
  
  if( gainIndex < 0 && gainIndex >= MUSCLE_GAINS_NUMBER ) return 0.0;
  
  EMGMuscle muscle = joint->musclesList[ muscleIndex ];
  if( value < MUSCLE_MIN_GAINS[ gainIndex ] ) value = MUSCLE_MIN_GAINS[ gainIndex ];
  else if( value > MUSCLE_MAX_GAINS[ gainIndex ] ) value = MUSCLE_MAX_GAINS[ gainIndex ];
  
  muscle->gainsList[ gainIndex ] = value;
  
  return muscle->gainsList[ gainIndex ];
}

double EMGProcessing_GetJointMuscleTorque( int jointID, size_t muscleIndex, double normalizedSignal, double jointAngle )
{
  khint_t jointIndex = kh_get( JointInt, jointsList, (khint_t) jointID );
  if( jointIndex == kh_end( jointsList ) ) return 0.0;
  
  EMGJoint joint = kh_value( jointsList, jointIndex );
  
  if( muscleIndex >= joint->musclesListLength ) return 0.0;
  
  return GetMuscleTorque( joint->musclesList[ muscleIndex ], normalizedSignal, jointAngle );
}


const char* MUSCLE_CURVE_NAMES[ MUSCLE_CURVES_NUMBER ] = { "active_force", "passive_force", "moment_arm", "normalized_length", "penation_angle" };
static EMGMuscle LoadEMGMuscleData( const char* configFileName )
{
  DEBUG_PRINT( "Trying to load muscle %s EMG data", configFileName );
  
  EMGMuscle newMuscle = NULL;
  
  int configFileID = ConfigParsing.LoadConfigFile( configFileName );
  if( configFileID != PARSED_DATA_INVALID_ID )
  {
    newMuscle = (EMGMuscle) malloc( sizeof(EMGMuscleData) );
    
    ParserInterface parser = ConfigParsing.GetParser();
    
    for( size_t curveIndex = 0; curveIndex < MUSCLE_CURVES_NUMBER; curveIndex++ )
    {
      char* curveString = parser.GetStringValue( configFileID, NULL, "curves.%s", MUSCLE_CURVE_NAMES[ curveIndex ] );
      newMuscle->curvesList[ curveIndex ] = CurveInterpolation.LoadCurveString( curveString );
    }

    newMuscle->gainsList[ MUSCLE_GAIN_ACTIVATION ] = -2.0;
    newMuscle->gainsList[ MUSCLE_GAIN_LENGTH ] = 1.0;
    newMuscle->gainsList[ MUSCLE_GAIN_ARM ] = 1.0;
    newMuscle->gainsList[ MUSCLE_GAIN_PENATION ] = 1.0;
    newMuscle->gainsList[ MUSCLE_GAIN_FORCE ] = 1.0;

    parser.UnloadData( configFileID );
  }
  else
  {
    DEBUG_PRINT( "configuration for muscle %s not found", configFileName );
    return false;
  }
  
  return newMuscle;
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
      
      newJoint->musclesList = (EMGMuscle*) calloc( newJoint->musclesListLength , sizeof(EMGMuscle) );
      for( size_t muscleIndex = 0; muscleIndex < newJoint->musclesListLength; muscleIndex++ )
      {
        newJoint->musclesList[ muscleIndex ] = LoadEMGMuscleData( parser.GetStringValue( configFileID, "", "muscles.%u.properties", muscleIndex ) );
        if( newJoint->musclesList[ muscleIndex ] != NULL )
        {
          newJoint->musclesList[ muscleIndex ]->emgSensor = Sensors.Init( parser.GetStringValue( configFileID, "", "muscles.%u.sensor", muscleIndex ) );
          if( newJoint->musclesList[ muscleIndex ]->emgSensor == NULL ) loadError = true;
        }
        else loadError = true;
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
    Sensors.End( joint->musclesList[ muscleIndex ]->emgSensor );
    for( size_t curveIndex = 0; curveIndex < MUSCLE_CURVES_NUMBER; curveIndex++ )
      CurveInterpolation.UnloadCurve( joint->musclesList[ muscleIndex ]->curvesList[ curveIndex ] );
  }
  
  free( joint->musclesList );
  
  free( joint );
}

#endif /* EMG_PROCESSING_H */
