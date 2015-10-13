#ifndef EMG_JOINT_CONTROL_H
#define EMG_JOINT_CONTROL_H

#include "emg_processing.h"

#include "config_parser.h"

#include "klib/khash.h"

#include "debug/async_debug.h"

enum MuscleGroups { MUSCLE_AGONIST, MUSCLE_ANTAGONIST, MUSCLE_GROUPS_NUMBER };

enum { JOINT_TORQUE, JOINT_STIFFNESS, JOINT_MEASURES_NUMBER };

typedef struct _EMGJointData
{
  int* muscleIDsTable[ MUSCLE_GROUPS_NUMBER ];
  size_t muscleGroupsSizesList[ MUSCLE_GROUPS_NUMBER ];
}
EMGJointData;

typedef EMGJointData* EMGJoint;

KHASH_MAP_INIT_INT( JointInt, EMGJoint )
static khash_t( JointInt )* jointsList = NULL;

#define EMG_JOINT_CONTROL_FUNCTIONS( namespace, function_init ) \
        function_init( int, namespace, InitJoint, const char* ) \
        function_init( void, namespace, EndJoint, int ) \
        function_init( double, namespace, GetTorque, int, double ) \
        function_init( double, namespace, GetStiffness, int, double ) \
        function_init( void, namespace, ChangeState, int, enum MuscleGroups, enum EMGProcessPhase )

INIT_NAMESPACE_INTERFACE( EMGJointControl, EMG_JOINT_CONTROL_FUNCTIONS )

static EMGJoint LoadEMGJointData( const char* );
static void UnloadEMGJointData( EMGJoint );

static int EMGJointControl_InitJoint( const char* configFileName )
{
  if( jointsList == NULL ) jointsList = kh_init( JointInt );
  
  int configKey = (int) kh_str_hash_func( configFileName );
  
  int insertionStatus;
  khint_t newJointID = kh_put( JointInt, jointsList, configKey, &insertionStatus );
  if( insertionStatus > 0 )
  {
    kh_value( jointsList, newJointID ) = LoadEMGJointData( configFileName );
    if( kh_value( jointsList, newJointID ) == NULL )
    {
      DEBUG_PRINT( "EMG joint controller %s configuration failed", configFileName );
      EMGJointControl_EndJoint( (int) newJointID );
      return -1;
    }
    
    DEBUG_PRINT( "new key %d inserted (iterator: %u - total: %u)", kh_key( jointsList, newJointID ), newJointID, kh_size( jointsList ) );
  }
  else if( insertionStatus == 0 ) { DEBUG_PRINT( "joint key %d already exists", configKey ); }
  
  return (int) newJointID;
}

void EMGJointControl_EndJoint( int jointID )
{
  if( !kh_exist( jointsList, (khint_t) jointID ) ) return;
  
  UnloadEMGJointData( kh_value( jointsList, (khint_t) jointID ) );
  
  kh_del( JointInt, jointsList, (khint_t) jointID );
  
  if( kh_size( jointsList ) == 0 )
  {
    kh_destroy( JointInt, jointsList );
    jointsList = NULL;
  }
}

double EMGJointControl_GetTorque( int jointID, double jointAngle )
{
  if( !kh_exist( jointsList, (khint_t) jointID ) ) return 0.0;
  
  double jointTorque = 0.0;
  
  EMGJoint joint = kh_value( jointsList, (khint_t) jointID );
  
  for( size_t muscleIndex = 0; muscleIndex < joint->muscleGroupsSizesList[ MUSCLE_AGONIST ]; muscleIndex++ )
    jointTorque += fabs( EMGProcessing.GetMuscleTorque( joint->muscleIDsTable[ MUSCLE_AGONIST ][ muscleIndex ], jointAngle ) );
  
  for( size_t muscleIndex = 0; muscleIndex < joint->muscleGroupsSizesList[ MUSCLE_ANTAGONIST ]; muscleIndex++ )
    jointTorque -= fabs( EMGProcessing.GetMuscleTorque( joint->muscleIDsTable[ MUSCLE_ANTAGONIST ][ muscleIndex ], jointAngle ) );
  
  return jointTorque;
}

double EMGJointControl_GetStiffness( int jointID, double jointAngle )
{
  if( !kh_exist( jointsList, (khint_t) jointID ) ) return 0.0;
  
  const double scaleFactor = 1.0;
  
  double jointStiffness = 0.0;
  
  EMGJoint joint = kh_value( jointsList, (khint_t) jointID );
  
  for( size_t muscleGroupID = 0; muscleGroupID < MUSCLE_GROUPS_NUMBER; muscleGroupID++ )
  {
    for( size_t muscleIndex = 0; muscleIndex < joint->muscleGroupsSizesList[ muscleGroupID ]; muscleIndex++ )
      jointStiffness += fabs( EMGProcessing.GetMuscleTorque( joint->muscleIDsTable[ muscleGroupID ][ muscleIndex ], jointAngle ) );
  }
  
  jointStiffness *= scaleFactor;
  
  //DEBUG_PRINT( "joint stiffness: %.3f + %.3f = %.3f", EMGProcessing_GetMuscleTorque( 0, jointAngle ), EMGProcessing_GetMuscleTorque( 1, jointAngle ), jointStiffness );
  
  if( jointStiffness < 0.0 ) jointStiffness = 0.0;
  
  return jointStiffness;
}

void EMGJointControl_ChangeState( int jointID, enum MuscleGroups muscleGroupID, enum EMGProcessPhase emgProcessingPhase )
{
  if( !kh_exist( jointsList, (khint_t) jointID ) ) return;
  
  EMGJoint joint = kh_value( jointsList, (khint_t) jointID );
  
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
          if( newJoint->muscleIDsTable[ muscleGroupIndex ][ muscleIndex ] == -1 ) loadError = true;
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
