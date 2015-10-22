#ifndef EMG_JOINT_CONTROL_H
#define EMG_JOINT_CONTROL_H

#include "signal_processing.h"
#include "emg_processing.h"

#include "config_parser.h"

#include "klib/khash.h"

#include "debug/async_debug.h"

enum MuscleGroups { MUSCLE_AGONIST, MUSCLE_ANTAGONIST, MUSCLE_GROUPS_NUMBER, MUSCLE_GROUPS_ALL };

enum { JOINT_TORQUE, JOINT_STIFFNESS, JOINT_MEASURES_NUMBER };

typedef struct _EMGJointData
{
  EMGMuscleParameters muscleParametersList;
  EMGMuscleParameters muscleParametersTable[ MUSCLE_GROUPS_NUMBER ];
  int* sensorIDsTable[ MUSCLE_GROUPS_NUMBER ];
  size_t muscleGroupsSizesList[ MUSCLE_GROUPS_NUMBER ];
}
EMGJointData;

typedef EMGJointData* EMGJoint;

KHASH_MAP_INIT_INT( JointInt, EMGJoint )
static khash_t( JointInt )* jointsList = NULL;

#define EMG_JOINT_CONTROL_FUNCTIONS( namespace, function_init ) \
        function_init( int, namespace, InitJoint, const char* ) \
        function_init( void, namespace, EndJoint, int ) \
        function_init( double, namespace, GetJointTorque, int, double ) \
        function_init( double, namespace, GetJointStiffness, int, double ) \
        function_init( EMGMuscleParameters, namespace, GetJointMusclesList, int, enum MuscleGroups, size_t* )

INIT_NAMESPACE_INTERFACE( EMGJointControl, EMG_JOINT_CONTROL_FUNCTIONS )

static EMGJoint LoadEMGJointData( const char* );
static void UnloadEMGJointData( EMGJoint );

const int EMG_JOINT_INVALID_ID = 0;

static int EMGJointControl_InitJoint( const char* configFileName )
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
      EMGJointControl_EndJoint( (int) newJointIndex );
      return EMG_JOINT_INVALID_ID;
    }
    
    DEBUG_PRINT( "new key %d inserted (iterator: %u - total: %u)", kh_key( jointsList, newJointIndex ), newJointIndex, kh_size( jointsList ) );
  }
  else if( insertionStatus == 0 ) DEBUG_PRINT( "joint key %d already exists", configKey );
  
  return (int) kh_key( jointsList, newJointIndex );
}

void EMGJointControl_EndJoint( int jointID )
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

double EMGJointControl_GetJointTorque( int jointID, double jointAngle )
{
  khint_t jointIndex = kh_get( JointInt, jointsList, (khint_t) jointID );
  if( jointIndex == kh_end( jointsList ) ) return 0.0;
  
  EMGJoint joint = kh_value( jointsList, jointIndex );
  
  double jointTorque = 0.0;
  
  EMGMuscleParameters muscleParameters = joint->muscleParametersList;
  
  for( size_t muscleIndex = 0; muscleIndex < joint->muscleGroupsSizesList[ MUSCLE_AGONIST ]; muscleIndex++ )
  {
    double normalizedSignal = SignalProcessing.GetProcessedSignal( joint->sensorIDsTable[ MUSCLE_AGONIST ][ muscleIndex ] );
    jointTorque += fabs( EMGProcessing.GetMuscleTorque( joint->muscleParametersTable[ MUSCLE_AGONIST ][ muscleIndex ], normalizedSignal, jointAngle ) );
  }
  
  for( size_t muscleIndex = 0; muscleIndex < joint->muscleGroupsSizesList[ MUSCLE_ANTAGONIST ]; muscleIndex++ )
  {
    double normalizedSignal = SignalProcessing.GetProcessedSignal( joint->sensorIDsTable[ MUSCLE_ANTAGONIST ][ muscleIndex ] );
    jointTorque -= fabs( EMGProcessing.GetMuscleTorque( joint->muscleParametersTable[ MUSCLE_ANTAGONIST ][ muscleIndex ], normalizedSignal, jointAngle ) );
  }
  
  return jointTorque;
}

double EMGJointControl_GetJointStiffness( int jointID, double jointAngle )
{
  khint_t jointIndex = kh_get( JointInt, jointsList, (khint_t) jointID );
  if( jointIndex == kh_end( jointsList ) ) return 0.0;
  
  EMGJoint joint = kh_value( jointsList, jointIndex );
  
  double jointStiffness = 0.0;
  
  for( size_t muscleGroupID = 0; muscleGroupID < MUSCLE_GROUPS_NUMBER; muscleGroupID++ )
  {
    for( size_t muscleIndex = 0; muscleIndex < joint->muscleGroupsSizesList[ muscleGroupID ]; muscleIndex++ )
    {
      double normalizedSignal = SignalProcessing.GetProcessedSignal( joint->sensorIDsTable[ muscleGroupID ][ muscleIndex ] );
      jointStiffness += fabs( EMGProcessing.GetMuscleTorque( joint->muscleParametersTable[ muscleGroupID ][ muscleIndex ], normalizedSignal, jointAngle ) );
    }
  }
  
  //DEBUG_PRINT( "joint stiffness: %.3f + %.3f = %.3f", EMGProcessing_GetMuscleTorque( 0, jointAngle ), EMGProcessing_GetMuscleTorque( 1, jointAngle ), jointStiffness );
  
  if( jointStiffness < 0.0 ) jointStiffness = 0.0;
  
  return jointStiffness;
}

EMGMuscleParameters EMGJointControl_GetJointMusclesList( int jointID, enum MuscleGroups muscleGroupID, size_t* ref_musclesCount )
{
  khint_t jointIndex = kh_get( JointInt, jointsList, (khint_t) jointID );
  if( jointIndex == kh_end( jointsList ) ) return NULL;
  
  EMGJoint joint = kh_value( jointsList, jointIndex );
  
  if( muscleGroupID >= 0 && muscleGroupID < MUSCLE_GROUPS_NUMBER )
  {
    if( ref_musclesCount != NULL ) *ref_musclesCount = joint->muscleGroupsSizesList[ muscleGroupID ];
    return joint->muscleIDsTable[ muscleGroupID ];
  }
  
  if( ref_musclesCount != NULL ) *ref_musclesCount = joint->muscleGroupsSizesList[ MUSCLE_AGONIST ] + joint->muscleGroupsSizesList[ MUSCLE_ANTAGONIST ];
  
  return joint->muscleParametersList;
}

void EMGJointControl_ChangeJointState( int jointID, enum MuscleGroups muscleGroupID, enum EMGProcessPhase emgProcessingPhase )
{
  khint_t jointIndex = kh_get( JointInt, jointsList, (khint_t) jointID );
  if( jointIndex == kh_end( jointsList ) ) return;
  
  EMGJoint joint = kh_value( jointsList, jointIndex );
  
  if( muscleGroupID < 0 || muscleGroupID >= MUSCLE_GROUPS_NUMBER ) return;

  for( size_t muscleIndex = 0; muscleIndex < joint->muscleGroupsSizesList[ muscleGroupID ]; muscleIndex++ )
    EMGProcessing.ChangePhase( joint->muscleIDsTable[ muscleGroupID ][ muscleIndex ], emgProcessingPhase );
}


const char* MUSCLE_GROUP_NAMES[ MUSCLE_GROUPS_NUMBER ] = { "agonist", "antagonist" };
static EMGJoint LoadEMGJointData( const char* configFileName )
{
  DEBUG_PRINT( "Trying to load joint %s EMG data", configFileName );
  
  static char searchPath[ FILE_PARSER_MAX_PATH_LENGTH ];
  
  bool loadError = false;
  
  EMGJoint newJoint = (EMGJoint) malloc( sizeof(EMGJointData) );
  memset( newJoint, 0, sizeof(EMGJointData) );
  
  int configFileID = ConfigParser.LoadFile( configFileName );
  if( configFileID != -1 )
  {
    ConfigParser.SetBaseKey( configFileID, "muscle_groups" );
    for( size_t muscleGroupIndex = 0; muscleGroupIndex < MUSCLE_GROUPS_NUMBER; muscleGroupIndex++ )
    {
      if( (newJoint->muscleGroupsSizesList[ muscleGroupIndex ] = (size_t) ConfigParser.GetListSize( configFileID, MUSCLE_GROUP_NAMES[ muscleGroupIndex ] )) > 0 )
      {
        DEBUG_PRINT( "%u %s muscles found for joint %s", newJoint->muscleGroupsSizesList[ muscleGroupIndex ], MUSCLE_GROUP_NAMES[ muscleGroupIndex ], configFileName );

        newJoint->muscleIDsTable[ muscleGroupIndex ] = (int*) calloc( newJoint->muscleGroupsSizesList[ muscleGroupIndex ], sizeof(int) );

        for( size_t muscleIndex = 0; muscleIndex < newJoint->muscleGroupsSizesList[ muscleGroupIndex ]; muscleIndex++ )
        {
          sprintf( searchPath, "%s.%u", MUSCLE_GROUP_NAMES[ muscleGroupIndex ], muscleIndex );
          newJoint->muscleIDsTable[ muscleGroupIndex ][ muscleIndex ] = EMGProcessing.InitSensor( ConfigParser.GetStringValue( configFileID, searchPath ) );
          if( newJoint->muscleIDsTable[ muscleGroupIndex ][ muscleIndex ] == EMG_SENSOR_INVALID_ID ) loadError = true;
        }
      }
    }

    ConfigParser.UnloadFile( configFileID );
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
  
  for( size_t muscleGroupIndex = 0; muscleGroupIndex < MUSCLE_GROUPS_NUMBER; muscleGroupIndex++ )
  {
    for( size_t muscleIndex = 0; muscleIndex < joint->muscleGroupsSizesList[ muscleGroupIndex ]; muscleIndex++ )
      EMGProcessing.EndSensor( joint->muscleIDsTable[ muscleGroupIndex ][ muscleIndex ] );
    free( joint->muscleIDsTable[ muscleGroupIndex ] );
  }
  
  free( joint );
}

#endif /* EMG_JOINT_CONTROL_H */
